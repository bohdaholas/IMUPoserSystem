#ifndef PERIPH_GATT_H
#define PERIPH_GATT_H

#include <host/ble_uuid.h>
#include <array>
#include <map>

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
enum class BodyLoc {
    PELVIS          = 0,
    SPINE           = 1,
    HEAD            = 2,
    LEFT_HIP        = 3,
    RIGHT_HIP       = 4,
    LEFT_KNEE       = 5,
    RIGHT_KNEE      = 6,
    LEFT_SHOULDER   = 7,
    RIGHT_SHOULDER  = 8,
    LEFT_ELBOW      = 9,
    RIGHT_ELBOW     = 10
};

constexpr ble_uuid16_t CCCD = BLE_UUID16_INIT(0x2902);

class GATT {
public:
    void handle_subscription_rx(struct ble_gap_event *event);
    void ble_central_subscribe(const struct peer *peer);
    void init();

private:
    using conn_handle_t = uint16_t;
    using euler_angles_t = std::array<float, 3>;
    struct node_data_t {
        BodyLoc location;
        euler_angles_t orientation;
    };
    std::map<conn_handle_t, node_data_t> nodes_data;
};

inline GATT gatt;

#endif //PERIPH_GATT_H
