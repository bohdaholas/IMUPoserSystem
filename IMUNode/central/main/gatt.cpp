#include <host/ble_gatt.h>
#include <services/gap/ble_svc_gap.h>
#include <host/ble_hs_mbuf.h>
#include <cstring>
#include "host/ble_hs.h"
#include "esp_log.h"
#include <os/os_mbuf.h>
#include "gatt.h"
#include "esp_central.h"

void GATT::handle_subscription_rx(struct ble_gap_event *event) {
  int rc;
  int data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
  char data_buf[data_len];

  rc = os_mbuf_copydata(event->notify_rx.om, 0, data_len, data_buf);
  if (rc == 0) {
    memcpy(&ax, &data_buf[0], sizeof(float));
    memcpy(&ay, &data_buf[4], sizeof(float));
    memcpy(&az, &data_buf[8], sizeof(float));

    printf("Received floats: ax=%f, ay=%f, az=%f\n", ax, ay, az);
  } else {
    printf("Error: Failed to copy data from mbuf.\n");
  }
}

void GATT::ble_central_subscribe(const struct peer *peer) {
  int rc;

  const struct peer_dsc *dsc;
  uint8_t value[2];

  dsc = peer_dsc_find_uuid(peer,
                           &IMU_SVC_UUID.u,
                           &IMU_SVC_CHR_DATA_UUID.u,
                           &CCCD.u);
  if (dsc == nullptr) {
    MODLOG_DFLT(ERROR, "Error: Peer lacks a CCCD characteristic\n ");
    goto err;
  }

  value[0] = 1;
  value[1] = 0;

  rc = ble_gattc_write_flat(peer->conn_handle, dsc->dsc.handle,
                            &value, sizeof value, nullptr, nullptr);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "Error: Failed to subscribe to characteristic; "
                       "rc=%d\n", rc);
    goto err;
  }

  return;
  err:

  ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

void GATT::init() {
  ax = ay = az = 0;
}

