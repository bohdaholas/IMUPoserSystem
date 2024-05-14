#ifndef PERIPH_IMU_H
#define PERIPH_IMU_H

#include <array>
#include "BNO055ESP32.h"

using quaternion_t = std::array<float, 4>;
using measurement_t = std::array<float, 3>;

struct CalibrationParams {
    std::array<float, 3> accel_offsets;
    std::array<float, 9> accel_scale_factors;

    std::array<float, 3> gyro_offsets;

    std::array<float, 3> mag_offsets;
    std::array<float, 9> mag_scale_factors;
};

union imu_calib_params_t {
  char raw_bytes[sizeof(CalibrationParams)]{};
  CalibrationParams custom_calib_params;
  bno055_offsets_t bno055_calib_params;
};

enum class SensorFusionAlgorithm {
    BNO055_BUILTIN,
    MADGWICK_FILTER
};

class Imu {
public:
  /** Methods for initializing IMU peripheral **/
  Imu();
  void init(SensorFusionAlgorithm mode);

  /** Methods for passing calibration stage **/
  void run_calibration(bool recalibrate_accel_gyro=false);
  static void run_custom_calibration(void *pvParameters);
  static void run_bno055_calibration(void *pvParameters);
  void wait_till_calibrated();

  /** Custom algorithms for accelerometer, gyroscope and magnetometer calibration **/
  void calibrate_gyro();
  void calibrate_accelerometer();
  void calibrate_magnetometer();

  /** Helper functions for working with calibration data **/
  void nvs_save_calib_params();
  void nvs_restore_calib_params();
  void print_calib_params(const imu_calib_params_t *calib_params_ptr);
  imu_calib_params_t get_nvs_calib_params();
  bno055_offsets_t get_curr_bno055_calib_params();
  void apply_bno055_calibration(const bno055_offsets_t *bno055_calib_params_ptr);
  bno055_calibration_t get_calibration_status();

  /** Methods for streaming IMU data **/
  void start_measurements(QueueHandle_t *gatt_queue_handle);
  static void get_measurements(TimerHandle_t xTimer);

  /** Helper functions for retrieving data from IMU **/
  quaternion_t getQuaternion();
  void print_quaternion(quaternion_t q);
  void print_quaternion_using_euler_angles(quaternion_t q);
  measurement_t readAccelerometer();
  measurement_t readGyroscope();
  measurement_t readMagnetometer();

  quaternion_t orientation_quaternion{1, 0, 0, 0};
  EventGroupHandle_t imu_event_group = nullptr;
  static constexpr int IS_CALIBRATED_BIT = BIT0;
  bool is_customly_calibrated = false;
private:
  BNO055 bno;
  TimerHandle_t imu_timer_handle = nullptr;
  QueueHandle_t *gatt_queue_handle = nullptr;
  SensorFusionAlgorithm fusion_algorithm;
  imu_calib_params_t calib_params;
  std::string calibration_file;
  bool recalibrate_accel_gyro;
};

inline Imu imu;

#endif //PERIPH_IMU_H
