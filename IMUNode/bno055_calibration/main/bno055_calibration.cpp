#include <nvs_utils.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

extern "C" void app_main() {
  bool old_config = true;
  nvs_init();
  imu.init(SensorFusionAlgorithm::BNO055_BUILTIN);

  auto params = imu.get_nvs_calib_params();
  imu.print_calib_params(&params);

  while (true) {
    bno055_calibration_t cal_status = imu.get_calibration_status();
    printf("Calibration status: SYS: %u GYRO: %u ACC:%u MAG:%u\n",
             cal_status.sys, cal_status.gyro, cal_status.accel, cal_status.mag);
    if (old_config && cal_status.accel == 3 &&
        cal_status.gyro == 3 && cal_status.mag == 3) {
      params = imu.get_curr_calib_params();
      imu.print_calib_params(&params);
      imu.nvs_save_calib_params(&params);
      old_config = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
