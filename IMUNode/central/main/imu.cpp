#include "gatt.h"
#include "imu.h"
#include "udp_client.h"

constexpr const char *tag = "BNO055";

void Imu::init() {
  try {
    bno.begin();
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
  } catch (BNO055BaseException& ex) {
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

euler_angles_t Imu::getAnglesEuler() {
//  static float ax = 0.0f;
//  static float ay = 0.0f;
//  static float az = 0.0f;
//
//  ax += 0.01f;
//  ay += 0.05f;
//  az += 0.02f;
//
//  return {ax, ay, az};

  static float offset = 0.0;
  static size_t n = 0;

  bno055_calibration_t cal = bno.getCalibration();
  bno055_vector_t v = bno.getVectorEuler();
  ESP_LOGI(tag, "Euler: X: %.1f Y: %.1f Z: %.1f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u",
           v.x, v.y, v.z,
           cal.sys, cal.gyro, cal.accel, cal.mag);

  if (cal.mag == 3) {
    if (n < 10) {
      offset += static_cast<float>(v.x);
      n++;
    } else if (n == 10) {
      offset /= (float) n;
      n++;
      printf("****************************\n");
      printf("****************************\n");
      printf("****************************\n");
    } else {
      return {static_cast<float>(v.x) - offset, static_cast<float>(v.y), static_cast<float>(v.z)};
    }
  }
  return {0.0, 0.0, 0.0};
}

void Imu::produce(TimerHandle_t xTimer) {
  imu.node_data.orientation = imu.getAnglesEuler();

  if(xQueueSend(udp_client.nodeDataQueue, &imu.node_data, portMAX_DELAY) != pdPASS) {
    printf("Error: Failed to push data to the queue\n");
  }
}

void Imu::start_producer() {
  strcpy(imu.node_data.body_loc_str, "pelvis");
  imu_timer_handle = xTimerCreate("udp_timer_handle", pdMS_TO_TICKS(100), pdTRUE, (void *)0, produce);
  if (imu_timer_handle == nullptr) {
    ESP_LOGE("app_main", "Timer create failed");
  } else {
    if (xTimerStart(imu_timer_handle, 10) != pdPASS) {
      ESP_LOGE("app_main", "Timer start failed");
    }
  }
}
