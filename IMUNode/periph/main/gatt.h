#ifndef PERIPH_GATT_H
#define PERIPH_GATT_H

#include <host/ble_uuid.h>
#include <functional>
#include "imu.h"

extern "C" {

const size_t DIS_CB_ID = 0;
const size_t IMU_MEAS_CB_ID = 1;
const size_t IMU_BODYLOC_CB_ID = 2;

constexpr ble_uuid16_t DIS_SVC_UUID = BLE_UUID16_INIT(0x180A);
constexpr ble_uuid16_t DIS_SVC_CHR_MFC_NAME_UUID = BLE_UUID16_INIT(0x2A23);
constexpr ble_uuid16_t DIS_SVC_CHR_MODEL_NO_UUID = BLE_UUID16_INIT(0x2A24);
constexpr ble_uuid16_t DIS_SVC_CHR_SYS_ID_UUID = BLE_UUID16_INIT(0x2A29);

constexpr ble_uuid128_t IMU_SVC_UUID =
    BLE_UUID128_INIT(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                     0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15);
constexpr ble_uuid128_t IMU_SVC_CHR_DATA_UUID =
    BLE_UUID128_INIT(0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22,
                     0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30);
constexpr ble_uuid128_t IMU_SVC_CHR_BODYLOC_UUID =
    BLE_UUID128_INIT(0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
                     0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46);
constexpr ble_uuid16_t CCCD = BLE_UUID16_INIT(0x2902);
constexpr size_t MAX_BODYLOC_SIZE = 16;

struct node_data_t {
    char body_loc_str[MAX_BODYLOC_SIZE]{};
    quaternion_t orientation_quaternion{};
};

class GATT {
public:
    uint16_t char_handle_imu_data;
    uint16_t char_handle_body_sensor_location;

    static void ble_imu_prph_tx(void *pvParameters);

    int dis_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);
    int imu_meas_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg);
    int imu_bodyloc_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                         struct ble_gatt_access_ctxt *ctxt, void *arg);

    static int execute_gatt_callback(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg);
    int init();
    QueueHandle_t imu_queue_handle = nullptr;
    TaskHandle_t ble_imu_prph_tx_handle;
    node_data_t node_data{};
private:

    int ble_svc_imu_notify(node_data_t *p_node_data, uint16_t conn_handle);
};

inline GATT gatt;

}


#endif //PERIPH_GATT_H
