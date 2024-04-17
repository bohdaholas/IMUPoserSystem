#ifndef PERIPH_GATT_H
#define PERIPH_GATT_H

#include <host/ble_uuid.h>
#include <array>
#include <map>
#include <string>
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

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

using conn_handle_t = uint16_t;
struct node_data_t {
    char body_loc_str[MAX_BODYLOC_SIZE];
    quaternion_t orientation_quaternion{};
};

class GATT {
public:
    void handle_notification(struct ble_gap_event *event);
    static void handle_imu_data(void *pvParameters);
    void ble_central_read(const struct peer *peer);
    static int ble_central_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                   struct ble_gatt_attr *attr, void *arg);
    void ble_central_subscribe(const struct peer *peer);

    void init();

    QueueHandle_t imu_queue_handle = nullptr;
private:
    std::map<conn_handle_t, node_data_t> nodes_data;
    node_data_t this_node_data{};
};

inline GATT gatt;

#ifdef __cplusplus
}
#endif

#endif //PERIPH_GATT_H
