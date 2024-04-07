#ifndef PERIPH_GAP_H
#define PERIPH_GAP_H

#include <string>

#ifdef __cplusplus
extern "C" {
#endif

constexpr size_t MAX_CONNECTIONS_NUM = MYNEWT_VAL(BLE_MAX_CONNECTIONS);
constexpr size_t MAX_SVCS = MYNEWT_VAL(BLE_MAX_CONNECTIONS) * 2;
constexpr size_t MAX_CHRS = MYNEWT_VAL(BLE_MAX_CONNECTIONS) * 7;
constexpr size_t MAX_DSCS = MYNEWT_VAL(BLE_MAX_CONNECTIONS) * 1;

class GAP {
public:
    static void ble_central_on_sync();
    static void ble_central_on_reset(int reason);
    static int ble_central_gap_event(struct ble_gap_event *event, void *arg);
    void init(const std::string &cmplt_name);
private:
    void ble_central_scan();
    void ble_central_connect_if_interesting(void *disc);
    int ble_central_should_connect(const struct ble_gap_disc_desc *disc);
    static void ble_central_on_disc_complete(const struct peer *peer, int status, void *arg);

    std::string dev_name_cmplt;
    uint8_t connections_num = 0;
};

inline GAP gap;

#ifdef __cplusplus
}
#endif

#endif //PERIPH_GAP_H
