#include <host/ble_gatt.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <host/ble_hs_mbuf.h>
#include <cstring>
#include "host/ble_hs.h"
#include "esp_log.h"
#include "freertos/FreeRTOSConfig.h"
#include <os/os_mbuf.h>
#include "gatt.h"
#include "gap.h"
#include "nvs_utils.h"

constexpr const char *tag = "BNO055";

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Device Information Service */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *) &DIS_SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Characteristic: Manufacturer name */
                .uuid = (const ble_uuid_t *) &DIS_SVC_CHR_MFC_NAME_UUID.u,
                .access_cb = GATT::execute_gatt_callback,
                .arg = (void *) &DIS_CB_ID,
                .flags = BLE_GATT_CHR_F_READ
            },
            {
                /* Characteristic: Model number string */
                .uuid = (const ble_uuid_t *) &DIS_SVC_CHR_MODEL_NO_UUID.u,
                .access_cb = GATT::execute_gatt_callback,
                .arg = (void *) &DIS_CB_ID,
                .flags = BLE_GATT_CHR_F_READ
            },
            {
                /* Characteristic: System ID */
                .uuid = (const ble_uuid_t *) &DIS_SVC_CHR_SYS_ID_UUID.u,
                .access_cb = GATT::execute_gatt_callback,
                .arg = (void *) &DIS_CB_ID,
                .flags = BLE_GATT_CHR_F_READ
            },
            {
                0, /* No more characteristics in this service */
            }
        },
    },

    /* Custom IMU Measurements Service */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *) &IMU_SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Characteristic: IMU Measurements */
                .uuid = (const ble_uuid_t *) &IMU_SVC_CHR_DATA_UUID.u,
                .access_cb = GATT::execute_gatt_callback,
                .arg = (void *) &IMU_MEAS_CB_ID,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt.char_handle_imu_data
            },
            {
                /* Characteristic: Body Sensor Location Characteristic */
                .uuid = (const ble_uuid_t *) &IMU_SVC_CHR_BODYLOC_UUID.u,
                .access_cb = GATT::execute_gatt_callback,
                .arg = (void *) &IMU_BODYLOC_CB_ID,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &gatt.char_handle_body_sensor_location
            },
            {
                /* Characteristic: Body Sensor Location Characteristic */
                .uuid = nullptr,
                .access_cb = nullptr,
                .flags = 0,
                .val_handle = nullptr
            }
        }
    },

    {
        0, /* No more services */
    },

};

void GATT::ble_imu_prph_tx_stop() const {
  xTimerStop(ble_imu_prph_tx_timer, 1000 / portTICK_PERIOD_MS );
}

void GATT::ble_imu_prph_tx_reset() const {
  int rc;

  if (xTimerReset(ble_imu_prph_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
    rc = 0;
  } else {
    rc = 1;
  }

  assert(rc == 0);
}

void GATT::ble_imu_prph_tx(TimerHandle_t ev) {
  int rc;

  if (!gap.get_sub()) {
    gatt.ble_imu_prph_tx_stop();
    printf("\nreturn");
    return;
  }

  rc = gatt.ble_svc_imu_notify(gap.get_conn_handle());
  if (rc == 0) {
    MODLOG_DFLT(INFO, "Notification sent successfully");
  } else {
    MODLOG_DFLT(INFO, "Error in sending notification");
  }

  gatt.ble_imu_prph_tx_reset();
}

int GATT::dis_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
  constexpr const char *manuf_name = "ESP32";
  constexpr const char *model_num = "IMU node";
  constexpr const char *system_id = "imu#21312";

  uint16_t uuid;
  int rc;

  uuid = ble_uuid_u16(ctxt->chr->uuid);

  switch (uuid) {
    case DIS_SVC_CHR_MODEL_NO_UUID.value:
      rc = os_mbuf_append(ctxt->om, model_num, strlen(model_num));
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    case DIS_SVC_CHR_MFC_NAME_UUID.value:
      rc = os_mbuf_append(ctxt->om, manuf_name, strlen(manuf_name));
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    case DIS_SVC_CHR_SYS_ID_UUID.value:
      rc = os_mbuf_append(ctxt->om, system_id, strlen(system_id));
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    default:
      return BLE_ATT_ERR_UNLIKELY;
  }
}

int GATT::imu_meas_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
  printf("Oh I'm here\n");
  return 0;
}

int GATT::imu_bodyloc_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg) {
  constexpr size_t MAX_BODYLOC_SIZE = 16;
  static bool body_loc_already_retrieved = false;
  static char body_loc_str[MAX_BODYLOC_SIZE];
  static size_t actual_size;

  if (!body_loc_already_retrieved) {
    read_string_from_nvs("BodyLoc", body_loc_str, &actual_size);
    body_loc_already_retrieved = true;
  }
  int rc = os_mbuf_append(ctxt->om, &body_loc_str, actual_size - 1); // actual_size - 1 == strlen(body_loc_str)
  if (rc == 0) {
    printf("BodyLocation characteristics is read successfully\n");
  } else {
    printf("Error reading BodyLocation characteristics\n");
  }
  return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

int GATT::ble_svc_imu_notify(uint16_t conn_handle) {
  int rc;
  struct os_mbuf *om;
  static uint8_t imu_payload[12];

  static float x_angle = 5.0f;
  static float y_angle = 150.0f;
  static float z_angle = 50.0f;

  x_angle += 0.1f;
  y_angle += 0.2f;
  z_angle += 0.4f;

  memcpy(&imu_payload[0], &x_angle, sizeof(float));
  memcpy(&imu_payload[4], &y_angle, sizeof(float));
  memcpy(&imu_payload[8], &z_angle, sizeof(float));

//  bno055_calibration_t cal = imu.getCalibration();
//  bno055_vector_t v = imu.getVectorEuler();
//  ESP_LOGI(tag, "Euler: X: %.1f Y: %.1f Z: %.1f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u",
//           v.x, v.y, v.z,
//           cal.sys, cal.gyro, cal.accel, cal.mag);
//
//  auto ax = static_cast<float>(v.x);
//  auto ay = static_cast<float>(v.y);
//  auto az = static_cast<float>(v.z);
//  memcpy(&imu_payload[0], &ax, sizeof(float));
//  memcpy(&imu_payload[4], &ay, sizeof(float));
//  memcpy(&imu_payload[8], &az, sizeof(float));

  for (unsigned char i : imu_payload) {
    printf("%02x ", i);
  }
  printf("\n");

  om = ble_hs_mbuf_from_flat(imu_payload, sizeof(imu_payload));

  rc = ble_gatts_notify_custom(conn_handle, gatt.char_handle_imu_data, om);
  return rc;
}

int GATT::execute_gatt_callback(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
  size_t cb_id = *static_cast<size_t *>(arg);
  if (cb_id == DIS_CB_ID) {
    return gatt.dis_chr_access_cb(conn_handle, attr_handle, ctxt, nullptr);
  } else if (cb_id == IMU_MEAS_CB_ID) {
    return gatt.imu_meas_chr_access_cb(conn_handle, attr_handle, ctxt, nullptr);
  } else if (cb_id == IMU_BODYLOC_CB_ID) {
    return gatt.imu_bodyloc_chr_access_cb(conn_handle, attr_handle, ctxt, nullptr);
  } else {
    assert(0);
    return -1;
  }
}

int GATT::init() {
  int rc;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  rc = ble_gatts_count_cfg(gatt_svr_svcs);
  if (rc != 0) {
    return rc;
  }

  rc = ble_gatts_add_svcs(gatt_svr_svcs);
  if (rc != 0) {
    return rc;
  }

  ble_imu_prph_tx_timer = xTimerCreate("ble_imu_prph_tx_timer", pdMS_TO_TICKS(1000), pdTRUE,
                                            (void *)0, GATT::ble_imu_prph_tx);

  return 0;
}

