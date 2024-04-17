#ifndef PERIPH_IMU_H
#define PERIPH_IMU_H

#include "BNO055ESP32.h"

using quaternion_t = std::array<float, 4>;

class Imu {
public:
  void init(QueueHandle_t *q_handle);

  Imu();
  void start_producer();
  static void produce(TimerHandle_t xTimer);
  bno055_calibration_t getCalibration();
  quaternion_t getQuaternion();

  quaternion_t orientation_quaternion{1, 0, 0, 0};
private:
  BNO055 bno;
  TimerHandle_t imu_timer_handle;
  QueueHandle_t *gatt_queue_handle;
};

inline Imu imu;

#endif //PERIPH_IMU_H
