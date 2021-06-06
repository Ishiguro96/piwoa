/*
 * PIWOA
 * Dawid Tobor
 * Michał Gronka
 * Aleksandra Wojtowicz
 */

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <random>
#include <cmath>

#include "rplidar.h"

using namespace rp::standalone::rplidar;

// Make a measurement cone of given limits
constexpr float MinimalAngle = -60.f;
constexpr float MaximalAngle = 60.f;

enum class WageFunction : uint8_t {
  Rectangle,
  NormalDist
};

void print_usage(int argc, const char * argv[])
{
    printf("Simple LIDAR data grabber for RPLIDAR.\n"
           "Version: " RPLIDAR_SDK_VERSION "\n"
           "Usage:\n"
           "%s <com port> [baudrate]\n"
           "The default baudrate is 115200(for A2) or 256000(for A3). Please refer to the datasheet for details.\n"
           , argv[0]);
}

/*
 * Function to get wage for concrete measurement based on angle
 *
 * Angle must be in form of [-MinimalAngle; MaximalAngle]
 */
float getWage(float angle, WageFunction function)
{
  float returnWage = 0.f;
  switch (function) {
    case WageFunction::Rectangle:
    {
      constexpr float Wage = 1.f;
      returnWage = Wage;
      break;
    }

    case WageFunction::NormalDist:
    {
      constexpr double Sigma = 20.0;
      returnWage = 1.0 / (Sigma * sqrt(2.0 * M_PI)) * exp(-1.0 / 2.0 * pow((angle / Sigma), 2.0));
      break;
    }
  }

  return returnWage;
}

u_result capture_and_display(RPlidarDriver* drv)
{
  u_result ans;

  rplidar_response_measurement_node_t nodes[8192];
  size_t count = sizeof(nodes) / sizeof(nodes[0]);

  printf("waiting for data...\n");

  // fetch exactly one 0-360 degrees' scan
  ans = drv->grabScanData(nodes, count);
  if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
    drv->ascendScanData(nodes, count); // Reorder received data

    float sum = 0.f;

    for (int pos = 0; pos < (int)count ; ++pos) {
      constexpr uint32_t MinimalDistance = 1000; // [mm]
      constexpr WageFunction function = WageFunction::NormalDist;

      float theta = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
      if (theta > 180.f) theta -= 360.f;

      if (nodes[pos].distance_q2 && nodes[pos].distance_q2 / 4.0f < MinimalDistance) {
        if (theta >= MinimalAngle && theta <= 0.f) {
          sum -= 1000.f / (nodes[pos].distance_q2 / 4.0f) * getWage(theta, function);
          //std::cout << getWage(theta, function) << std::endl;
        } else if (theta <= MaximalAngle && theta >= 0.f) {
          sum += 1000.f / (nodes[pos].distance_q2 / 4.0f) * getWage(theta, function);
          //std::cout << getWage(theta, function) << std::endl;
        }
      }
        /*printf("%s theta: %03.2f Dist: %08.2f \n",
         (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
         (,
         nodes[pos].distance_q2/4.0f);*/
    }

    std::cout << "Suma: " << sum << "\n";
  }

  return ans;
}

int main(int argc, const char * argv[]) {
  const char * opt_com_path = NULL;
  _u32         opt_com_baudrate = 115200;
  u_result     op_result;

  std::default_random_engine generator;

  if (argc < 2) {
    print_usage(argc, argv);
    return -1;
  }
  opt_com_path = argv[1];
  if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);

  // create the driver instance
  RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

  if (!drv) {
    fprintf(stderr, "insufficent memory, exit\n");
    exit(-2);
  }

  rplidar_response_device_health_t healthInfo;
  rplidar_response_device_info_t devInfo;

  // try to connect
  if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
    fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
        , opt_com_path);
    goto clean;
  }

  // retrieving the device info
  ////////////////////////////////////////
  op_result = drv->getDeviceInfo(devInfo);

  if (IS_FAIL(op_result)) {
    if (op_result == RESULT_OPERATION_TIMEOUT) {
      // you can check the detailed failure reason
      fprintf(stderr, "Error, operation time out.\n");
    } else {
      fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
      // other unexpected result
    }
    goto clean;
  }

  // print out the device serial number, firmware and hardware version number..
  printf("RPLIDAR S/N: ");
  for (int pos = 0; pos < 16 ;++pos) {
      printf("%02X", devInfo.serialnum[pos]);
  }

  printf("\n"
          "Version: " RPLIDAR_SDK_VERSION"\n"
          "Firmware Ver: %d.%02d\n"
          "Hardware Rev: %d\n"
          , devInfo.firmware_version>>8
          , devInfo.firmware_version & 0xFF
          , (int)devInfo.hardware_version);


  // check the device health
  ////////////////////////////////////////
  op_result = drv->getHealth(healthInfo);
  if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
    printf("RPLidar health status : ");
    switch (healthInfo.status) {
    case RPLIDAR_STATUS_OK:
      printf("OK.");
      break;
    case RPLIDAR_STATUS_WARNING:
      printf("Warning.");
      break;
    case RPLIDAR_STATUS_ERROR:
      printf("Error.");
      break;
    }
    printf(" (errorcode: %d)\n", healthInfo.error_code);

  } else {
    fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
    goto clean;
  }


  if (healthInfo.status == RPLIDAR_STATUS_ERROR) {
    fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
    // enable the following code if you want rplidar to be reboot by software
    // drv->reset();
    goto clean;
  }

  drv->startMotor();

  // take only one 360 deg scan and display the result as a histogram
  ////////////////////////////////////////////////////////////////////////////////
  if (IS_FAIL(drv->startScan(0, 1))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
  {
    fprintf(stderr, "Error, cannot start the scan operation.\n");
    goto clean;;
  }

  drv->setLidarSpinSpeed(100);

  while (!IS_FAIL(capture_and_display(drv))) {
    //fprintf(stderr, "Error, cannot grab scan data.\n");
    //goto clean;
  }

clean:
  drv->stop();
  drv->stopMotor();

  RPlidarDriver::DisposeDriver(drv);
  return 0;
}
