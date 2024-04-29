#include <host/ble_gap.h>
#include <services/gap/ble_svc_gap.h>
#include "gap.h"
#include "gatt.h"
#include "imu.h"

void GAP::ble_prph_on_sync() {
  int rc;

  rc = ble_hs_id_infer_auto(0, &gap.ble_imu_prph_addr_type);
  assert(rc == 0);

  uint8_t addr_val[6] = {0};
  rc = ble_hs_id_copy_addr(gap.ble_imu_prph_addr_type, addr_val, nullptr);
  assert(rc == 0);

  MODLOG_DFLT(INFO, "Device Address: ");
  MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
              addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
  MODLOG_DFLT(INFO, "\n");

  gap.ble_prph_advertise();
}

void GAP::ble_prph_on_reset(int reason) {
  MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void GAP::ble_prph_advertise() {
  struct ble_gap_adv_params adv_params{};
  struct ble_hs_adv_fields fields{};
  int rc;

  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  fields.name = (uint8_t *) dev_name_short.c_str();
  fields.name_len = strlen(dev_name_short.c_str());
  fields.name_is_complete = 0;

  ble_uuid128_t svc_uuid_array[] = {IMU_SVC_UUID};
  fields.uuids128 = svc_uuid_array;
  fields.num_uuids128 = 1;
  fields.uuids128_is_complete = 1;

  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
    return;
  }

  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  rc = ble_gap_adv_start(ble_imu_prph_addr_type, nullptr, BLE_HS_FOREVER,
                         &adv_params, GAP::ble_prph_gap_event, nullptr);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
    return;
  }
}

int GAP::ble_prph_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                  event->connect.status == 0 ? "established" : "failed",
                  event->connect.status);

      if (event->connect.status != 0) {
        gap.ble_prph_advertise();
      }
      gap.conn_handle = event->connect.conn_handle;
      break;

    case BLE_GAP_EVENT_DISCONNECT:
      MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

      gap.ble_prph_advertise();
      vTaskSuspend(gatt.ble_imu_prph_tx_handle);

      break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      MODLOG_DFLT(INFO, "adv complete\n");
      gap.ble_prph_advertise();
      break;

    case BLE_GAP_EVENT_SUBSCRIBE:
      MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                        "val_handle=%d\n",
                  event->subscribe.cur_notify, event->subscribe.attr_handle);

      gap.sub = true;

      if (event->subscribe.cur_notify) {
        vTaskResume(gatt.ble_imu_prph_tx_handle);
      }

      ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", gap.conn_handle);
      break;
  }

  return 0;
}

uint16_t GAP::get_conn_handle() const {
  return conn_handle;
}

bool GAP::get_sub() const {
  return sub;
}

void GAP::init(const std::string &cmplt_name, const std::string &short_name) {
  dev_name_short = short_name;
  dev_name_cmplt = cmplt_name;

  ble_svc_gap_device_name_set(dev_name_cmplt.c_str());

  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_mitm = 1;
}



