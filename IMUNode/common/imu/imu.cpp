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
    else
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

Imu::Imu() : bno(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16) {}

void Imu::start_producer(QueueHandle_t *queue_handle) {
  gatt_queue_handle = queue_handle;
  imu_timer_handle = xTimerCreate("imu_timer_handle", pdMS_TO_TICKS(100), pdTRUE, (void *)0, produce);
  if (imu_timer_handle == nullptr) {
    ESP_LOGE("app_main", "Timer create failed");
  } else {
    if (xTimerStart(imu_timer_handle, 10) != pdPASS) {
      ESP_LOGE("app_main", "Timer start failed");
    }
  }
}

void Imu::produce(TimerHandle_t xTimer) {
  imu.orientation_quaternion = imu.getQuaternion();
  imu.print_quaternion(imu.orientation_quaternion);
  imu.print_quaternion_using_euler_angles(imu.orientation_quaternion);
  bno055_calibration_t cal = imu.getCalibration();

//  if (cal.gyro != 3 || cal.mag != 3) return;

  if (*imu.gatt_queue_handle == nullptr) return;

  if(xQueueSend(*imu.gatt_queue_handle, &imu.orientation_quaternion, portMAX_DELAY) != pdPASS) {
    printf("Error: Failed to push data to the queue\n");
  }
}

bno055_calibration_t Imu::getCalibration() {
  return bno.getCalibration();
}

// Math library required for ‘sqrt’
#include <math.h>
// System constants
#define deltat 0.001f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
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
  else {
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
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

measurement_t Imu::readGyroscope() {
  bno055_vector_t v = bno.getVectorGyroscope();
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

measurement_t Imu::readMagnetometer() {
  bno055_vector_t v = bno.getVectorMagnetometer();
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

