#include <host/ble_gatt.h>
#include <services/gap/ble_svc_gap.h>
#include <host/ble_hs_mbuf.h>
#include <cstring>
#include "host/ble_hs.h"
#include "esp_log.h"
#include <os/os_mbuf.h>
#include "gatt.h"
#include "esp_central.h"
#include "udp_client.h"
#include "nvs_utils.h"

void GATT::handle_notification(struct ble_gap_event *event) {
  int rc;
  int data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
  char data_buf[data_len];
  uint16_t conn_handle = event->notify_rx.conn_handle;

  rc = os_mbuf_copydata(event->notify_rx.om, 0, data_len, data_buf);
  if (rc == 0) {
    quaternion_t orientation_quaternion;
    for(size_t i = 0; i < orientation_quaternion.size(); ++i) {
      memcpy(&orientation_quaternion[i], data_buf + (i * sizeof(float)), sizeof(float));
    }
    nodes_data[conn_handle].orientation_quaternion = orientation_quaternion;

    if(xQueueSend(udp_client.nodeDataQueue, &gatt.nodes_data[conn_handle], portMAX_DELAY) != pdPASS) {
      printf("Error: Failed to push data to the queue\n");
    }
  } else {
    printf("Error: Failed to copy data from mbuf.\n");
  }
}

void GATT::handle_imu_data(void *pvParameters) {
  for(;;) {
    quaternion_t orientation_quaternion{};
    if(xQueueReceive(gatt.imu_queue_handle, &orientation_quaternion, portMAX_DELAY) != pdPASS) {
      printf("Error when reading from a queue\n");
      continue;
    }
    if (udp_client.nodeDataQueue == nullptr) continue;

    gatt.this_node_data.orientation_quaternion = orientation_quaternion;
    if(xQueueSend(udp_client.nodeDataQueue, &gatt.this_node_data, portMAX_DELAY) != pdPASS) {
      printf("Error: Failed to push data to the queue\n");
    }
  }
}

void GATT::ble_central_read(const struct peer *peer) {
  int rc;
  const struct peer_chr *chr;

  chr = peer_chr_find_uuid(peer,
                           &IMU_SVC_UUID.u,
                           &IMU_SVC_CHR_BODYLOC_UUID.u);
  if (chr == nullptr) {
    MODLOG_DFLT(ERROR, "Error: Peer doesn't support the BodyLoc characteristic\n");
    goto err;
  }

  rc = ble_gattc_read(peer->conn_handle, chr->chr.val_handle,
                      GATT::ble_central_read_cb, nullptr);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "Error: Failed to read the characteristic; "
                       "rc=%d\n", rc);
    goto err;
  }

  return;
err:
  ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

int GATT::ble_central_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr, void *arg) {
  MODLOG_DFLT(INFO, "Read body location completed; status=%d conn_handle=%d\n",
              error->status, conn_handle);
  if (error->status != 0) {
    MODLOG_ERROR(INFO, "Error reading body location characteristic");
    return error->status;
  }

  if (attr->om == nullptr) {
    MODLOG_ERROR(INFO, "No data received");
    return BLE_ATT_ERR_UNLIKELY;
  }

  size_t data_len = OS_MBUF_PKTLEN(attr->om);
  if (data_len == 0) {
    MODLOG_ERROR(INFO, "No data received");
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (data_len > MAX_BODYLOC_SIZE - 1) {
    MODLOG_ERROR(INFO, "Too much data received");
    return BLE_ATT_ERR_UNLIKELY;
  }

  char body_loc_cstr[data_len + 1];
  os_mbuf_copydata(attr->om, 0, static_cast<int>(data_len), body_loc_cstr);
  body_loc_cstr[data_len] = '\0';
  strcpy(gatt.nodes_data[conn_handle].body_loc_str, body_loc_cstr);

  const struct peer *peer = peer_find(conn_handle);
  gatt.ble_central_subscribe(peer);

  return 0;
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
  imu_queue_handle = xQueueCreate(10, sizeof(quaternion_t));
  if (imu_queue_handle == nullptr) {
    printf("Failed to create queue instance\n");
    return;
  }

  size_t actual_size;
  read_string_from_nvs("BodyLoc", this_node_data.body_loc_str, &actual_size);

  xTaskCreate(handle_imu_data, "handle_imu_data", 2048, nullptr, 1, nullptr);
}




