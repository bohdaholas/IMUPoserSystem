#include "imu.h"

constexpr const char *tag = "BNO055";

void Imu::init() {
  try {
    bno.begin();
    bno.setUnits(BNO055_UNIT_ACCEL_MS2, BNO055_UNIT_ANGULAR_RATE_DPS, BNO055_UNIT_EULER_RADIANS,
                 BNO055_UNIT_TEMP_C, BNO055_DATA_FORMAT_ANDROID);
    bno.enableExternalCrystal();
    bno.setOprModeNdof();
    ESP_LOGI(tag, "Setup Done.");
  } catch (BNO055BaseException& ex) {
    ESP_LOGE(tag, "Setup Failed, Error: %s", ex.what());
    return;
  } catch (std::exception& ex) {
    ESP_LOGE(tag, "Setup Failed, Error: %s", ex.what());
    return;
  }

  try {
    bno055_self_test_result_t res = bno.getSelfTestResult();
    ESP_LOGI(tag, "Self-Test Results: MCU: %u, GYR:%u, MAG:%u, ACC: %u", res.mcuState, res.gyrState, res.magState,
             res.accState);
  } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
    ESP_LOGE(tag, "Something bad happened: %s", ex.what());
    return;
  } catch (std::exception& ex) {
    ESP_LOGE(tag, "Something bad happened: %s", ex.what());
    return;
  }
}

Imu::Imu() : bno(UART_NUM_1, GPIO_NUM_4, GPIO_NUM_5) {}

bno055_calibration_t Imu::getCalibration() {
  return bno.getCalibration();
}

bno055_vector_t Imu::getVectorEuler() {
  return bno.getVectorEuler();
}
