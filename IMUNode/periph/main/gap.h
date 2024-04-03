#ifndef PERIPH_GAP_H
#define PERIPH_GAP_H

#include <string>

#ifdef __cplusplus
extern "C" {
#endif

class GAP {
public:
    static void ble_prph_on_sync();
    static void ble_prph_on_reset(int reason);
    void ble_prph_advertise();
    static int ble_prph_gap_event(struct ble_gap_event *event, void *arg);

    [[nodiscard]] uint16_t get_conn_handle() const;
    [[nodiscard]] bool get_sub() const;

    void init(const std::string &cmplt_name, const std::string &short_name);
private:
    uint8_t ble_imu_prph_addr_type;
    uint16_t conn_handle;
    bool sub = false;
    std::string dev_name_cmplt;
    std::string dev_name_short;
};

inline GAP gap;

#ifdef __cplusplus
}
#endif

#endif //PERIPH_GAP_H
