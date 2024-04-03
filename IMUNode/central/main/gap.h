#ifndef PERIPH_GAP_H
#define PERIPH_GAP_H

#include <string>

#ifdef __cplusplus
extern "C" {
#endif

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
};

inline GAP gap;

#ifdef __cplusplus
}
#endif

#endif //PERIPH_GAP_H
