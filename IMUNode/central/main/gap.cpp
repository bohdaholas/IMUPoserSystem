#include <host/ble_gap.h>
#include <services/gap/ble_svc_gap.h>
#include <host/util/util.h>
#include "gap.h"
#include "gatt.h"
#include "esp_central.h"

void GAP::ble_central_on_sync() {
  int rc;

  rc = ble_hs_util_ensure_addr(0);
  assert(rc == 0);

  gap.ble_central_scan();
}

void GAP::ble_central_on_reset(int reason) {
  MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

int GAP::ble_central_gap_event(struct ble_gap_event *event, void *arg) {
  struct ble_gap_conn_desc desc{};
  struct ble_hs_adv_fields fields{};
  int rc;

  switch (event->type) {
    case BLE_GAP_EVENT_DISC:
      rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                   event->disc.length_data);
      if (rc != 0) {
        return 0;
      }

      print_adv_fields(&fields);

      gap.ble_central_connect_if_interesting(&event->disc);
      return 0;

    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        MODLOG_DFLT(INFO, "Connection established ");

        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");

        rc = peer_add(event->connect.conn_handle);
        if (rc != 0) {
          MODLOG_DFLT(ERROR, "Failed to add peer; rc=%d\n", rc);
          return 0;
        }

        rc = peer_disc_all(event->connect.conn_handle,
                           gap.ble_central_on_disc_complete, nullptr);
        if (rc != 0) {
          MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
          return 0;
        }
      } else {

        MODLOG_DFLT(ERROR, "Error: Connection failed; status=%d\n",
                    event->connect.status);
        gap.ble_central_scan();
      }

      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
      print_conn_desc(&event->disconnect.conn);
      MODLOG_DFLT(INFO, "\n");

      peer_delete(event->disconnect.conn.conn_handle);

      gap.ble_central_scan();
      return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
      MODLOG_DFLT(INFO, "discovery complete; reason=%d\n",
                  event->disc_complete.reason);
      return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
      MODLOG_DFLT(INFO, "encryption change event; status=%d ",
                  event->enc_change.status);
      rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
      assert(rc == 0);
      print_conn_desc(&desc);
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
      MODLOG_DFLT(INFO, "received %s; conn_handle=%d attr_handle=%d "
                        "attr_len=%d\n",
                  event->notify_rx.indication ?
                  "indication" :
                  "notification",
                  event->notify_rx.conn_handle,
                  event->notify_rx.attr_handle,
                  OS_MBUF_PKTLEN(event->notify_rx.om));
      gatt.handle_subscription_rx(event);
      return 0;
    }
    default:
      return 0;
  }
}

void GAP::ble_central_scan() {
  uint8_t own_addr_type;
  struct ble_gap_disc_params disc_params{};
  int rc;

  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
    return;
  }

  disc_params.filter_duplicates = 1;
  disc_params.passive = 1;
  disc_params.itvl = 0;
  disc_params.window = 0;
  disc_params.filter_policy = 0;
  disc_params.limited = 0;

  rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                    GAP::ble_central_gap_event, nullptr);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                rc);
  }
}

void GAP::ble_central_connect_if_interesting(void *disc) {
  uint8_t own_addr_type;
  int rc;
  ble_addr_t *addr;

  if (!ble_central_should_connect((struct ble_gap_disc_desc *) disc)) {
    return;
  }

  rc = ble_gap_disc_cancel();
  if (rc != 0) {
    MODLOG_DFLT(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
    return;
  }

  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
    return;
  }

  addr = &((struct ble_gap_disc_desc *) disc)->addr;
  rc = ble_gap_connect(own_addr_type, addr, 30000, nullptr,
                       GAP::ble_central_gap_event, nullptr);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "Error: Failed to connect to device; addr_type=%d "
                       "addr=%s; rc=%d\n",
                addr->type, addr_str(addr->val), rc);
    return;
  }
}

int GAP::ble_central_should_connect(const struct ble_gap_disc_desc *disc) {
  struct ble_hs_adv_fields fields{};
  int rc;
  int i;

  if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
      disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {

    return 0;
  }

  rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
  if (rc != 0) {
    return 0;
  }

  for (i = 0; i < fields.num_uuids128; i++) {
    if (ble_uuid_cmp(&fields.uuids128[i].u, &IMU_SVC_UUID.u) == 0) {
      return 1;
    }
  }
  return 0;
}

void GAP::ble_central_on_disc_complete(const struct peer *peer, int status, void *arg) {

  if (status != 0) {

    MODLOG_DFLT(ERROR, "Error: Service discovery failed; status=%d "
                       "conn_handle=%d\n", status, peer->conn_handle);
    ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return;
  }

  MODLOG_DFLT(INFO, "Service discovery complete; status=%d "
                    "conn_handle=%d\n", status, peer->conn_handle);

  gatt.ble_central_subscribe(peer);
}

void GAP::init(const std::string &cmplt_name) {
  int rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
  assert(rc == 0);
  dev_name_cmplt = cmplt_name;

  ble_svc_gap_device_name_set(dev_name_cmplt.c_str());
}
