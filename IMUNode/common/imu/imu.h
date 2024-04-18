#ifndef PERIPH_IMU_H
#define PERIPH_IMU_H

#include <array>
#include "BNO055ESP32.h"

using quaternion_t = std::array<float, 4>;
using measurement_t = std::array<float, 3>;

enum class SensorFusionAlgorithm {
    BNO055_BUILTIN,
    MADGWICK_FILTER
};

class Imu {
public:
  void init(SensorFusionAlgorithm mode);

  Imu();
  void start_producer(QueueHandle_t *gatt_queue_handle);
  static void produce(TimerHandle_t xTimer);
  bno055_calibration_t getCalibration();
  quaternion_t getQuaternion();
  void print_quaternion(quaternion_t q);
  void print_quaternion_using_euler_angles(quaternion_t q);
  measurement_t readAccelerometer();
  measurement_t readGyroscope();
  measurement_t readMagnetometer();

  quaternion_t orientation_quaternion{1, 0, 0, 0};
private:
  BNO055 bno;
  TimerHandle_t imu_timer_handle;
  QueueHandle_t *gatt_queue_handle;
  SensorFusionAlgorithm fusion_algorithm;
};

inline Imu imu;

#endif //PERIPH_IMU_H
