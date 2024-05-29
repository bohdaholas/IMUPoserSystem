#include <nvs_utils.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "button.h"

#define RUN_CALIBRATION      true
#define RECALIB_ACCEL_GYRO   true
#define USE_ACCELEROMETER    true
#define USE_GYROSCOPE        false
#define USE_MAGNETOMETER     false

extern "C" void app_main() {
  nvs_init();
  imu.init(SensorFusionAlgorithm::MADGWICK_FILTER);


#if RUN_CALIBRATION
  builtin_button.init();
  builtin_button.conf_btn_press_detection();
  imu.run_calibration(RECALIB_ACCEL_GYRO);
  imu.wait_till_calibrated();
#endif

#if USE_ACCELEROMETER
  while (true) {
    auto [ax, ay, az] = imu.readAccelerometer();
    printf("X: %f Y: %f Z: %f\n", ax, ay, az);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
#endif

#if USE_GYROSCOPE
  while (true) {
    auto [gx, gy, gz] = imu.readGyroscope();
    printf("X: %f Y: %f Z: %f\n", gx, gy, gz);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
#endif

#if USE_MAGNETOMETER
  while (true) {
    auto [mx, my, mz] = imu.readMagnetometer();
    printf("X: %f Y: %f Z: %f\n", mx, my, mz);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
#endif
}
