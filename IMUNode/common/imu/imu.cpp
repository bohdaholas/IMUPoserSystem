#include "eigen3/Eigen/Dense"
#include "imu.h"

constexpr const char *tag = "BNO055";

void Imu::init(SensorFusionAlgorithm algorithm) {
  try {
    bno.begin();
    bno.enableExternalCrystal();

    fusion_algorithm = algorithm;
    if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN)
      bno.setOprModeNdof();
    else if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER)
      bno.setOprModeAMG();
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

Imu::Imu() : bno(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16) {
}

void Imu::calibrate_gyro() {
  constexpr size_t READING_PERIOD_MS = 50;
  constexpr size_t GYRO_CALIBRATION_TIME_MS = 10000;
  constexpr size_t READINGS_NUM = GYRO_CALIBRATION_TIME_MS / READING_PERIOD_MS;
  measurement_t measurement_error{};
  size_t elapsed_time = 0;
  while (elapsed_time <= GYRO_CALIBRATION_TIME_MS) {
    auto measurement = readGyroscope();
    for (size_t i = 0; i < 3; ++i)
      measurement_error[i] += measurement[i];
    elapsed_time += READING_PERIOD_MS;
    vTaskDelay(pdMS_TO_TICKS(READING_PERIOD_MS));
  }
  for (size_t i = 0; i < 3; ++i)
    measurement_error[i] /= READINGS_NUM;
  calib_params.custom_calib_params.gyro_offsets = measurement_error;
  auto [g_offset_x, g_offset_y, g_offset_z] = measurement_error;
  printf("Offset X: %f Offset Y: %f Offset Z: %f\n", g_offset_x, g_offset_y, g_offset_z);
}

void Imu::calibrate_accelerometer() {
  imu.calib_params.custom_calib_params.accel_scale_factors = {
      1.01836032, -0.0416865, 0.01271423,
      0.00257276, 1.0226605, -0.05041788,
      0.04132167, -0.00537557, 1.01458358
  };

  imu.calib_params.custom_calib_params.accel_offsets = {
      0.06511262, 0.58956167, 0.2874256
  };
}

void Imu::calibrate_magnetometer() {
}

void Imu::run_calibration(bool recalibrate_accelerometer_gyroscope) {
  if (imu_event_group != nullptr) {
    // calibration already performed so no action
    return;
  }
  recalibrate_accel_gyro = recalibrate_accelerometer_gyroscope;
  imu_event_group = xEventGroupCreate();
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN) {
    calibration_file = "bno055_calib";
    xTaskCreate(run_bno055_calibration, "run_bno055_calibration",
                4096, nullptr, 1, nullptr);
  }
  if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER) {
    calibration_file = "madgwick_calib";
    xTaskCreate(run_custom_calibration, "run_custom_calibration",
                4096, nullptr, 1, nullptr);
  }
}

void Imu::run_custom_calibration(void *pvParameters) {
  if (!imu.recalibrate_accel_gyro) {
    imu.nvs_restore_calib_params();
  }
  for(;;) {
    if (imu.recalibrate_accel_gyro) {
      printf("Calibrating gyro...\n");
//      imu.calibrate_gyro();

      printf("Calibrating accelerometer...\n");
      imu.calibrate_accelerometer();
    }
    printf("Calibrating magnetometer...\n");
//    imu.calibrate_magnetometer();
    vTaskDelay(pdMS_TO_TICKS(500));

    xEventGroupSetBits(imu.imu_event_group, IS_CALIBRATED_BIT);
    imu.is_customly_calibrated = true;
    vTaskDelete(nullptr);
  }
}

void Imu::run_bno055_calibration(void *pvParameters) {
  bool calibration_done;
  if (!imu.recalibrate_accel_gyro) {
    imu.nvs_restore_calib_params();
  }
  for(;;) {
    bno055_calibration_t calibration = imu.get_calibration_status();
    if (imu.recalibrate_accel_gyro)
      calibration_done = calibration.accel == 3 && calibration.mag == 3 && calibration.gyro == 3;
    else
      calibration_done = calibration.gyro == 3;
    printf("Calibration sys %d acc %d gyro %d mag %d\n",
           calibration.sys, calibration.accel, calibration.gyro, calibration.mag);
    if (!calibration_done) continue;
    imu.calib_params.bno055_calib_params = imu.get_curr_bno055_calib_params();
    imu.print_calib_params(&imu.calib_params);
    imu.nvs_save_calib_params();

    xEventGroupSetBits(imu.imu_event_group, IS_CALIBRATED_BIT);
    vTaskDelete(nullptr);
  }
}

void Imu::wait_till_calibrated() {
  xEventGroupWaitBits(imu_event_group, IS_CALIBRATED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

void Imu::start_measurements(QueueHandle_t *queue_handle) {
  imu.wait_till_calibrated();
  printf("Calibration done\n");
  gatt_queue_handle = queue_handle;
  imu_timer_handle = xTimerCreate("imu_timer_handle", pdMS_TO_TICKS(16), pdTRUE, (void *)0, get_measurements);
  if (imu_timer_handle == nullptr) {
    ESP_LOGE("app_main", "Timer create failed");
  } else {
    if (xTimerStart(imu_timer_handle, 10) != pdPASS) {
      ESP_LOGE("app_main", "Timer start failed");
    }
  }
}

void Imu::get_measurements(TimerHandle_t xTimer) {
  imu.orientation_quaternion = imu.getQuaternion();
//  imu.print_quaternion(imu.orientation_quaternion);
//  imu.print_quaternion_using_euler_angles(imu.orientation_quaternion);

  if (*imu.gatt_queue_handle == nullptr) return;

  if(xQueueSend(*imu.gatt_queue_handle, &imu.orientation_quaternion, portMAX_DELAY) != pdPASS) {
    printf("Error: Failed to push data to the queue\n");
  }
}

// Math library required for ‘sqrt’
#include <math.h>
#include <nvs_utils.h>
// System constants
#define deltat 0.0018f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.3f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta
// Global system variables
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float m_x, m_y, m_z; // magnetometer measurements
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error
// Function to compute one filter iteration
void filterUpdate()
{
  // local system variables
  float norm; // vector norm
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
  float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
  J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
  float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
  float h_x, h_y, h_z; // computed flux in the earth frame
  // axulirary variables to avoid reapeated calcualtions
  float halfSEq_1 = 0.5f * SEq_1;
  float halfSEq_2 = 0.5f * SEq_2;
  float halfSEq_3 = 0.5f * SEq_3;
  float halfSEq_4 = 0.5f * SEq_4;
  float twoSEq_1 = 2.0f * SEq_1;
  float twoSEq_2 = 2.0f * SEq_2;
  float twoSEq_3 = 2.0f * SEq_3;
  float twoSEq_4 = 2.0f * SEq_4;
  float twob_x = 2.0f * b_x;
  float twob_z = 2.0f * b_z;
  float twob_xSEq_1 = 2.0f * b_x * SEq_1;
  float twob_xSEq_2 = 2.0f * b_x * SEq_2;
  float twob_xSEq_3 = 2.0f * b_x * SEq_3;
  float twob_xSEq_4 = 2.0f * b_x * SEq_4;
  float twob_zSEq_1 = 2.0f * b_z * SEq_1;
  float twob_zSEq_2 = 2.0f * b_z * SEq_2;
  float twob_zSEq_3 = 2.0f * b_z * SEq_3;
  float twob_zSEq_4 = 2.0f * b_z * SEq_4;
  float SEq_1SEq_2;
  float SEq_1SEq_3 = SEq_1 * SEq_3;
  float SEq_1SEq_4;
  float SEq_2SEq_3;
  float SEq_2SEq_4 = SEq_2 * SEq_4;
  float SEq_3SEq_4;
  float twom_x = 2.0f * m_x;
  float twom_y = 2.0f * m_y;
  float twom_z = 2.0f * m_z;
  // normalise the accelerometer measurement
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x /= norm;
  a_y /= norm;
  a_z /= norm;
  // normalise the magnetometer measurement
  norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
  m_x /= norm;
  m_y /= norm;
  m_z /= norm;
  // compute the objective function and Jacobian
  f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
  f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
  f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
  f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
  f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
  J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * SEq_4;
  J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21; // negated in matrix multiplication
  J_33 = 2.0f * J_11or24; // negated in matrix multiplication
  J_41 = twob_zSEq_3; // negated in matrix multiplication
  J_42 = twob_zSEq_4;
  J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
  J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
  J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
  J_52 = twob_xSEq_3 + twob_zSEq_1;
  J_53 = twob_xSEq_2 + twob_zSEq_4;
  J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
  J_61 = twob_xSEq_3;
  J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
  J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
  J_64 = twob_xSEq_2;
  // compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
  // normalise the gradient to estimate direction of the gyroscope error
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 = SEqHatDot_1 / norm;
  SEqHatDot_2 = SEqHatDot_2 / norm;
  SEqHatDot_3 = SEqHatDot_3 / norm;
  SEqHatDot_4 = SEqHatDot_4 / norm;
  // compute angular estimated direction of the gyroscope error
  w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
  w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
  w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
  // compute and remove the gyroscope baises
  w_bx += w_err_x * deltat * zeta;
  w_by += w_err_y * deltat * zeta;
  w_bz += w_err_z * deltat * zeta;
  w_x -= w_bx;
  w_y -= w_by;
  w_z -= w_bz;
  // compute the quaternion rate measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
  // compute then integrate the estimated quaternion rate
  SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
  SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
  SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
  SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
  // normalise quaternion
  norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
  SEq_1 /= norm;
  SEq_2 /= norm;
  SEq_3 /= norm;
  SEq_4 /= norm;
  // compute flux in the earth frame
  SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
  SEq_1SEq_3 = SEq_1 * SEq_3;
  SEq_1SEq_4 = SEq_1 * SEq_4;
  SEq_3SEq_4 = SEq_3 * SEq_4;
  SEq_2SEq_3 = SEq_2 * SEq_3;
  SEq_2SEq_4 = SEq_2 * SEq_4;
  h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
  h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
  h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
  // normalise the flux vector to have only components in the x and z
  b_x = sqrt((h_x * h_x) + (h_y * h_y));
  b_z = h_z;
}

quaternion_t Imu::getQuaternion() {
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN) {
    bno055_quaternion_t bno055_q = bno.getQuaternion();
    quaternion_t q = {static_cast<float>(bno055_q.w), static_cast<float>(bno055_q.x), static_cast<float>(bno055_q.y), static_cast<float>(bno055_q.z)};
    return q;
  }
  if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER) {
    bno055_vector_t acc = bno.getVectorAccelerometer();
    a_x = static_cast<float>(acc.x);
    a_y = static_cast<float>(acc.y);
    a_z = static_cast<float>(acc.z);
    bno055_vector_t gyro = bno.getVectorGyroscope();
    w_x = static_cast<float>(gyro.x);
    w_y = static_cast<float>(gyro.y);
    w_z = static_cast<float>(gyro.z);
    bno055_vector_t mag = bno.getVectorMagnetometer();
    m_x = static_cast<float>(mag.x);
    m_y = static_cast<float>(mag.y);
    m_z = static_cast<float>(mag.z);
    filterUpdate();
    return {SEq_1, SEq_2, SEq_3, SEq_4};
  }
  return {1, 0, 0, 0};
}

void Imu::print_quaternion(quaternion_t q) {
  auto [qw, qx, qy, qz] = q;
  bno055_calibration_t cal = bno.getCalibration();
  ESP_LOGI(tag, "Quaternion: W: %.2f X: %.2f Y: %.2f Z: %.2f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u",
           qw, qx, qy, qz, cal.sys, cal.gyro, cal.accel, cal.mag);
}

void Imu::print_quaternion_using_euler_angles(quaternion_t q) {
  auto [qw, qx, qy, qz] = q;
  float roll,pitch,yaw;
  float sinr_cosp = 2 * (qw * qx + qy * qz);
  float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2 * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);
  else
    pitch = asin(sinp);

  float siny_cosp = 2 * (qw * qz + qx * qy);
  float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  yaw = atan2(siny_cosp, cosy_cosp);

  roll = roll * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw = yaw * 180.0 / M_PI;

  ESP_LOGI(tag, "Euler: X: %.1f Y: %.1f Z: %.1f", roll, pitch, yaw);
}

measurement_t Imu::readAccelerometer() {
  bno055_vector_t v = bno.getVectorAccelerometer();
  if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER && is_customly_calibrated) {
    Eigen::Vector3f accel_raw_meas(v.x, v.y, v.z);
    Eigen::Map<Eigen::Vector3f> offsets(calib_params.custom_calib_params.accel_offsets.data());
    Eigen::Map<Eigen::Matrix3f> scale_factors(calib_params.custom_calib_params.accel_scale_factors.data());
    Eigen::Vector3f accel_calib_meas = scale_factors * accel_raw_meas + offsets;
    v.x = accel_calib_meas[0];
    v.y = accel_calib_meas[1];
    v.z = accel_calib_meas[2];
  }
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

measurement_t Imu::readGyroscope() {
  bno055_vector_t v = bno.getVectorGyroscope();
  if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER && is_customly_calibrated) {
    auto [g_offset_x, g_offset_y, g_offset_z] = calib_params.custom_calib_params.gyro_offsets;
    v.x -= g_offset_x;
    v.y -= g_offset_y;
    v.z -= g_offset_z;
  }
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

measurement_t Imu::readMagnetometer() {
  bno055_vector_t v = bno.getVectorMagnetometer();
  if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER && is_customly_calibrated) {
//    auto [m_offset_x, m_offset_y, m_offset_z] = calib_params.custom_calib_params.mag_offsets;
//    v.x -= m_offset_x;
//    v.y -= m_offset_y;
//    v.z -= m_offset_z;
  }
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

void Imu::nvs_save_calib_params() {
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN) {
    if (recalibrate_accel_gyro) {
      printf("writing calb params to %s \n", calibration_file.c_str());
      write_bytes_to_nvs(calibration_file.c_str(), calib_params.raw_bytes,
                         sizeof(calib_params.bno055_calib_params));
    } else {
      auto imu_calib_params = get_nvs_calib_params();
      const size_t BNO055_MAGNETOMETER_OFFSET_START_IDX = 6;
      const size_t BNO055_MAGNETOMETER_OFFSET_LENGTH = 6;
      const size_t BNO055_MAGNETOMETER_RADIUS_START_IDX = 20;
      const size_t BNO055_MAGNETOMETER_RADIUS_LENGTH = 2;
      memcpy(&imu_calib_params.raw_bytes[BNO055_MAGNETOMETER_OFFSET_START_IDX],
             &calib_params.raw_bytes[BNO055_MAGNETOMETER_OFFSET_START_IDX],
             BNO055_MAGNETOMETER_OFFSET_LENGTH);
      memcpy(&imu_calib_params.raw_bytes[BNO055_MAGNETOMETER_RADIUS_START_IDX],
             &calib_params.raw_bytes[BNO055_MAGNETOMETER_RADIUS_START_IDX],
             BNO055_MAGNETOMETER_RADIUS_LENGTH);
      write_bytes_to_nvs(calibration_file.c_str(), imu_calib_params.raw_bytes,
                         sizeof(imu_calib_params.bno055_calib_params));
    }
  }
  else if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER) {
    if (recalibrate_accel_gyro) {
      write_bytes_to_nvs(calibration_file.c_str(), calib_params.raw_bytes,
                         sizeof(calib_params.custom_calib_params));
    } else {
      auto imu_calib_params = get_nvs_calib_params();
      const size_t CUSTOM_MAGNETOMETER_START_IDX = 15 * sizeof(float);
      memcpy(&imu_calib_params.raw_bytes[CUSTOM_MAGNETOMETER_START_IDX],
             &calib_params.raw_bytes[CUSTOM_MAGNETOMETER_START_IDX],
             12 * sizeof(float));
      write_bytes_to_nvs(calibration_file.c_str(), imu_calib_params.raw_bytes,
                         sizeof(imu_calib_params.bno055_calib_params));
    }
  }
}

void Imu::nvs_restore_calib_params() {
  printf("Restoring calibration params...\n");
  auto params = get_nvs_calib_params();
  print_calib_params(&params);
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN)
    apply_bno055_calibration(&params.bno055_calib_params);
}

void Imu::print_calib_params(const imu_calib_params_t *calib_params_ptr) {
  printf("\n*************************\n");
  printf("*************************\n\n");

  printf("Accelerometer Offsets:\n");
  printf("  Offset X: %d\n", calib_params_ptr->bno055_calib_params.accelOffsetX);
  printf("  Offset Y: %d\n", calib_params_ptr->bno055_calib_params.accelOffsetY);
  printf("  Offset Z: %d\n", calib_params_ptr->bno055_calib_params.accelOffsetZ);

  printf("Magnetometer Offsets:\n");
  printf("  Offset X: %d\n", calib_params_ptr->bno055_calib_params.magOffsetX);
  printf("  Offset Y: %d\n", calib_params_ptr->bno055_calib_params.magOffsetY);
  printf("  Offset Z: %d\n", calib_params_ptr->bno055_calib_params.magOffsetZ);

  printf("Gyroscope Offsets:\n");
  printf("  Offset X: %d\n", calib_params_ptr->bno055_calib_params.gyroOffsetX);
  printf("  Offset Y: %d\n", calib_params_ptr->bno055_calib_params.gyroOffsetY);
  printf("  Offset Z: %d\n", calib_params_ptr->bno055_calib_params.gyroOffsetZ);

  printf("Radius Values:\n");
  printf("  Accelerometer Radius: %d\n", calib_params_ptr->bno055_calib_params.accelRadius);
  printf("  Magnetometer Radius: %d\n", calib_params_ptr->bno055_calib_params.magRadius);

  printf("\n*************************\n");
  printf("*************************\n\n");
}

imu_calib_params_t Imu::get_nvs_calib_params() {
  size_t length = 0;
  char arr[sizeof(imu_calib_params_t)]{};
  printf("reading calib from  %s\n", calibration_file.c_str());
  read_bytes_from_nvs(calibration_file.c_str(), arr, &length);
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN && sizeof(bno055_offsets_t) != length) {
    ESP_LOGE("BNO", "calibration parameter length doesn't match the expected one: "
                    "sizeof(bno055_offsets_t)=%d, length=%d", sizeof(CalibrationParams), length);
  }
  if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER && sizeof(CalibrationParams) != length) {
    ESP_LOGE("BNO", "calibration parameter length doesn't match the expected one: "
                    "sizeof(CalibrationParams)=%d, length=%d", sizeof(CalibrationParams), length);
  }
  memcpy(&calib_params, arr, length);
  return calib_params;
}

bno055_offsets_t Imu::get_curr_bno055_calib_params() {
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN) {
    bno.setOprModeConfig(true);
    auto bno055_params = bno.getSensorOffsets();
    bno.setOprModeNdof(true);
    return bno055_params;
  }
  assert(0);
  return {};
}

void Imu::apply_bno055_calibration(const bno055_offsets_t *bno055_calib_params_ptr) {
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN) {
    bno.setOprModeConfig(true);
    bno.setSensorOffsets(*bno055_calib_params_ptr);
    bno.setOprModeNdof(true);
  }
}

bno055_calibration_t Imu::get_calibration_status() {
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN) {
    return bno.getCalibration();
  }
  throw std::runtime_error("This API is available when BNO055 filtering algorithm is selected");
}







