#ifndef PERIPH_IMU_H
#define PERIPH_IMU_H

#include "BNO055ESP32.h"

class Imu {
public:
  void init();
  Imu();
  void start_producer();
  static void produce(TimerHandle_t xTimer);
  bno055_calibration_t getCalibration();
  euler_angles_t getAnglesEuler();

  node_data_t node_data;
private:
  BNO055 bno;
  TimerHandle_t imu_timer_handle;
};

inline Imu imu;

#endif //PERIPH_IMU_H
