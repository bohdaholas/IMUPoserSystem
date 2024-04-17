#include "eigen3/Eigen/Dense"
#include "imu.h"

constexpr const char *tag = "BNO055";

void Imu::init(QueueHandle_t *q_handle) {
  try {
    bno.begin();
    bno.enableExternalCrystal();
    bno.setOprModeNdof();
//    bno.setOprModeAMG();
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

  gatt_queue_handle = q_handle;
  start_producer();
}

Imu::Imu() : bno(UART_NUM_1, GPIO_NUM_4, GPIO_NUM_5) {}

bno055_calibration_t Imu::getCalibration() {
  return bno.getCalibration();
}

quaternion_t Imu::getQuaternion() {
//  bno055_vector_t acc = bno.getVectorAccelerometer();
//  a_x = static_cast<float>(acc.x);
//  a_y = static_cast<float>(acc.y);
//  a_z = static_cast<float>(acc.z);
//  bno055_vector_t gyro = bno.getVectorGyroscope();
//  w_x = static_cast<float>(gyro.x);
//  w_y = static_cast<float>(gyro.y);
//  w_z = static_cast<float>(gyro.z);
//  bno055_vector_t mag = bno.getVectorMagnetometer();
//  m_x = static_cast<float>(mag.x);
//  m_y = static_cast<float>(mag.y);
//  m_z = static_cast<float>(mag.z);
//  ESP_LOGI(TAG, "Quaternion: W: %.1f X: %.1f Y: %.1f Z: %.1f", SEq[0], SEq[1], SEq[2], SEq[3]);
//  filterUpdate();
////  filterUpdate(acc, gyro, mag);
//  return {SEq[0], SEq[1], SEq[2], SEq[3]};
  bno055_calibration_t cal = bno.getCalibration();
  bno055_quaternion_t q = bno.getQuaternion();
  ESP_LOGI(tag, "Quaternion: W: %.2f X: %.2f Y: %.2f Z: %.2f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u",
           q.w, q.x, q.y, q.z, cal.sys, cal.gyro, cal.accel, cal.mag);

  float roll,pitch,yaw;
  float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);
  else
    pitch = asin(sinp);

  float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  yaw = atan2(siny_cosp, cosy_cosp);

  roll = roll * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw = yaw * 180.0 / M_PI;

  ESP_LOGI(tag, "Euler: X: %.1f Y: %.1f Z: %.1f", roll, pitch, yaw);

  return {static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z)};
}

void Imu::produce(TimerHandle_t xTimer) {
  imu.orientation_quaternion = imu.getQuaternion();
  bno055_calibration_t cal = imu.getCalibration();

  if (cal.gyro != 3 || cal.mag != 3) return;

  if (*imu.gatt_queue_handle == nullptr) return;

  if(xQueueSend(*imu.gatt_queue_handle, &imu.orientation_quaternion, portMAX_DELAY) != pdPASS) {
    printf("Error: Failed to push data to the queue\n");
  }
}

void Imu::start_producer() {
  imu_timer_handle = xTimerCreate("imu_timer_handle", pdMS_TO_TICKS(100), pdTRUE, (void *)0, produce);
  if (imu_timer_handle == nullptr) {
    ESP_LOGE("app_main", "Timer create failed");
  } else {
    if (xTimerStart(imu_timer_handle, 10) != pdPASS) {
      ESP_LOGE("app_main", "Timer start failed");
    }
  }
}
