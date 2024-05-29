#include <nvs_utils.h>
#include <iostream>
#include <numbers>
#include "eigen3/Eigen/Dense"
#include "imu.h"
#include "button.h"

using std::cout, std::endl;

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
  constexpr size_t N_MEAS_POSITIONS = 6;
  Eigen::Matrix<float, 6, 4> W;
  W.setConstant(1.0f);
  for (size_t i = 0; i < N_MEAS_POSITIONS; ++i) {
    builtin_button.wait_for_btn_press();

    auto measurement = readAccelerometer();
    cout << "Pos " << i << " Measurement: ";
    for (const auto &axis_accel: measurement) {
      cout << axis_accel << " ";
    }
    cout << endl;

    for (size_t j = 0; j < 3; ++j) {
      W(i, j) = measurement[j];
    }
  }

  Eigen::Matrix<float, 6, 3> Y;
  Y << 9.81,  0,      0,
      -9.81,  0,      0,
       0,     9.81,   0,
       0,    -9.81,   0,
       0,     0,     9.81,
       0,     0,    -9.81;

  Eigen::Matrix<float, 4, 3> X = (W.transpose() * W).inverse() * W.transpose() * Y;
  Eigen::Matrix3f scale_factors = X.topLeftCorner<3, 3>();
  Eigen::Vector3f biases = X.bottomRows<1>().transpose();

  auto &custom_calib_params = imu.calib_params.custom_calib_params;
  for (int i = 0; i < 3; ++i) {
    custom_calib_params.accel_offsets[i] = static_cast<float>(biases(i));
    for (int j = 0; j < 3; ++j) {
      custom_calib_params.accel_scale_factors[i * 3 + j] = static_cast<float>(scale_factors(i, j));
    }
  }
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
    imu.nvs_save_calib_params();

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
  imu_timer_handle = xTimerCreate("imu_timer_handle", pdMS_TO_TICKS(100), pdTRUE, (void *)0, get_measurements);
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

Eigen::Vector4f SEq(1, 0, 0, 0); // estimated orientation quaternion elements with initial conditions
Eigen::Vector2f b(1, 0); // reference direction of flux in earth frame
Eigen::Vector3f gyro_bias(0, 0, 0); // estimate gyroscope biases error
void filterUpdate(Eigen::Vector3f &acc, Eigen::Vector3f &gyro, Eigen::Vector3f &mag)
{
  static const float deltat = 0.0018f; // sampling period in seconds (shown as 1 ms)
  static const float gyroMeasError = std::numbers::pi * (5.0f / 180.0f); // gyroscope measurement error in rad/s (shown as 5 deg/s)
  static const float gyroMeasDrift = std::numbers::pi * (0.3f / 180.0f); // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
  static const float beta = std::sqrt(3.0f / 4.0f) * gyroMeasError * 1.2f; // compute beta
  static const float zeta = std::sqrt(3.0f / 4.0f) * gyroMeasDrift * 1.5; // compute zeta
  Eigen::Vector4f SEqDot_omega(1, 0, 0, 0); // quaternion rate from gyroscopes elements
  Eigen::VectorXf f(6);
  Eigen::Matrix<float, 6, 4> J;
  Eigen::Vector4f SEqHatDot(1, 0, 0, 0); // estimated direction of the gyroscope error
  Eigen::Vector3f gyro_err;
  Eigen::Vector3f h; // computed flux in the earth frame
  // axulirary variables to avoid reapeated calcualtions
  auto half_SEq = 0.5f * SEq;
  auto two_SEq = 0.5f * SEq;
  auto two_b = 2.0f * b;
  auto two_bx_SEq = two_b(0) * SEq;
  auto two_bz_SEq = two_b(1) * SEq;
  float SEq_1SEq_2;
  float SEq_1SEq_3 = SEq(0) * SEq(2);
  float SEq_1SEq_4;
  float SEq_2SEq_3;
  float SEq_2SEq_4 = SEq(1) * SEq(3);
  float SEq_3SEq_4;
  // normalise the accelerometer measurement
  acc.normalize();
  mag.normalize();
  auto two_mag = 2.0f * mag;
  // compute the objective function and Jacobian
  f(0) = two_SEq(1) * SEq(3) - two_SEq(0) * SEq(2) - acc(0);
  f(1) = two_SEq(0) * SEq(1) + two_SEq(2) * SEq(3) - acc(1);
  f(2) = 1.0f - two_SEq(1) * SEq(1) - two_SEq(2) * SEq(2) - acc(2);
  f(3) = two_b(0) * (0.5f - SEq(2) * SEq(2) - SEq(3) * SEq(3)) + two_b(1) * (SEq_2SEq_4 - SEq_1SEq_3) - mag(0);
  f(4) = two_b(0) * (SEq(1) * SEq(2) - SEq(0) * SEq(3)) + two_b(1) * (SEq(0) * SEq(1) + SEq(2) * SEq(3)) - mag(1);
  f(5) = two_b(0) * (SEq_1SEq_3 + SEq_2SEq_4) + two_b(1) * (0.5f - SEq(1) * SEq(1) - SEq(2) * SEq(2)) - mag(2);
  J(0, 0) = J(1, 3) = two_SEq(2);
  J(0, 1) = J(1, 2) = 2.0f * SEq(3);
  J(0, 2) = J(1, 1) = two_SEq(0);
  J(0, 3) = J(1, 0) = two_SEq(1);
  J(2, 1) = 2.0f * J(0, 3); // negated in matrix multiplication
  J(2, 2) = 2.0f * J(0, 0); // negated in matrix multiplication
  J(3, 0) = two_bz_SEq(2); // negated in matrix multiplication
  J(3, 1) = two_bz_SEq(3);
  J(3, 2) = 2.0f * two_bx_SEq(2) + two_bz_SEq(0); // negated in matrix multiplication
  J(3, 3) = 2.0f * two_bx_SEq(3) - two_bz_SEq(1); // negated in matrix multiplication
  J(4, 0) = two_bx_SEq(3) - two_bz_SEq(1); // negated in matrix multiplication
  J(4, 1) = two_bx_SEq(2) + two_bz_SEq(0);
  J(4, 2) = two_bx_SEq(1) + two_bz_SEq(3);
  J(4, 3) = two_bx_SEq(0) - two_bz_SEq(2); // negated in matrix multiplication
  J(5, 0) = two_bx_SEq(2);
  J(5, 1) = two_bx_SEq(3) - 2.0f * two_bz_SEq(1);
  J(5, 2) = two_bx_SEq(0) - 2.0f * two_bz_SEq(2);
  J(5, 3) = two_bx_SEq(1);
  // compute the gradient (matrix multiplication)
  SEqHatDot = J.transpose() * f;
  // normalise the gradient to estimate direction of the gyroscope error
  SEqHatDot.normalize();
  // compute angular estimated direction of the gyroscope error
  gyro_err(0) = two_SEq(0) * SEqHatDot(1) - two_SEq(1) * SEqHatDot(0) - two_SEq(2) * SEqHatDot(3) + two_SEq(3) * SEqHatDot(2);
  gyro_err(1) = two_SEq(0) * SEqHatDot(2) + two_SEq(1) * SEqHatDot(3) - two_SEq(2) * SEqHatDot(0) - two_SEq(3) * SEqHatDot(1);
  gyro_err(2) = two_SEq(0) * SEqHatDot(3) - two_SEq(1) * SEqHatDot(2) + two_SEq(2) * SEqHatDot(1) - two_SEq(3) * SEqHatDot(0);
  // compute and remove the gyroscope baises
  gyro_bias += gyro_err * deltat * zeta;
  gyro -= gyro_bias;
  // compute the quaternion rate measured by gyroscopes
  SEqDot_omega(0) = -half_SEq(1) * gyro(0) - half_SEq(2) * gyro(1) - half_SEq(3) * gyro(2);
  SEqDot_omega(1) = half_SEq(0) * gyro(0) + half_SEq(2) * gyro(2) - half_SEq(3) * gyro(1);
  SEqDot_omega(2) = half_SEq(0) * gyro(1) - half_SEq(1) * gyro(2) + half_SEq(3) * gyro(0);
  SEqDot_omega(3) = half_SEq(0) * gyro(2) + half_SEq(1) * gyro(1) - half_SEq(2) * gyro(0);
  // compute then integrate the estimated quaternion rate
  SEq += (SEqDot_omega - beta * SEqHatDot) * deltat;
  // normalise quaternion
  SEq.normalize();
  // compute flux in the earth frame
  SEq_1SEq_2 = SEq(0) * SEq(1); // recompute axulirary variables
  SEq_1SEq_3 = SEq(0) * SEq(2);
  SEq_1SEq_4 = SEq(0) * SEq(3);
  SEq_3SEq_4 = SEq(2) * SEq(3);
  SEq_2SEq_3 = SEq(1) * SEq(2);
  SEq_2SEq_4 = SEq(1) * SEq(3);
  h(0) = two_mag(0) * (0.5f - SEq(2) * SEq(2) - SEq(3) * SEq(3)) + two_mag(1) * (SEq_2SEq_3 - SEq_1SEq_4) + two_mag(2) * (SEq_2SEq_4 + SEq_1SEq_3);
  h(1) = two_mag(0) * (SEq_2SEq_3 + SEq_1SEq_4) + two_mag(1) * (0.5f - SEq(1) * SEq(1) - SEq(3) * SEq(3)) + two_mag(2) * (SEq_3SEq_4 - SEq_1SEq_2);
  h(2) = two_mag(0) * (SEq_2SEq_4 - SEq_1SEq_3) + two_mag(1) * (SEq_3SEq_4 + SEq_1SEq_2) + two_mag(2) * (0.5f - SEq(1) * SEq(1) - SEq(2) * SEq(2));
  // normalise the flux vector to have only components in the x and z
  b(0) = std::sqrt((h(0) * h(0)) + (h(1) * h(1)));
  b(1) = h(2);
}

quaternion_t Imu::getQuaternion() {
  if (fusion_algorithm == SensorFusionAlgorithm::BNO055_BUILTIN) {
    bno055_quaternion_t bno055_q = bno.getQuaternion();
    quaternion_t q = {static_cast<float>(bno055_q.w), static_cast<float>(bno055_q.x), static_cast<float>(bno055_q.y), static_cast<float>(bno055_q.z)};
    return q;
  }
  if (fusion_algorithm == SensorFusionAlgorithm::MADGWICK_FILTER) {
    bno055_vector_t acc_bno = bno.getVectorAccelerometer();
    Eigen::Vector3f acc(
      static_cast<float>(acc_bno.x),
      static_cast<float>(acc_bno.y),
      static_cast<float>(acc_bno.z)
    );
    bno055_vector_t gyro_bno = bno.getVectorGyroscope();
    Eigen::Vector3f gyro(
        static_cast<float>(gyro_bno.x),
        static_cast<float>(gyro_bno.y),
        static_cast<float>(gyro_bno.z)
    );
    bno055_vector_t mag_bno = bno.getVectorMagnetometer();
    Eigen::Vector3f mag(
        static_cast<float>(mag_bno.x),
        static_cast<float>(mag_bno.y),
        static_cast<float>(mag_bno.z)
    );
    if (acc.norm() && mag.norm())
      filterUpdate(acc, gyro, mag);
    return {SEq(0), SEq(1), SEq(2), SEq(3)};
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







