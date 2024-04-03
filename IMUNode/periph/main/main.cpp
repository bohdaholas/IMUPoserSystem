#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "gap.h"
#include "gatt.h"

static constexpr const char *device_name_cmplt = "leaf_imu_prph";
static constexpr const char *device_name_short = "imu";

void nvs_init() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void ble_htp_prph_host_task(void *param)
{
    nimble_port_run(); // returns only when nimble_port_stop() is executed
    nimble_port_freertos_deinit();
}

void nimble_init() {
  // initialize controller and NimBLE host stack
  int ret = nimble_port_init();
  if (ret != ESP_OK) {
    MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
    nimble_port_stop();
  }

  // gets executed when the host and controller become synced
  ble_hs_cfg.sync_cb = GAP::ble_prph_on_sync;
  // gets executed when the host resets itself and the controller due to fatal error
  ble_hs_cfg.reset_cb = GAP::ble_prph_on_reset;
}

extern "C" void app_main()
{
    nvs_init();
    nimble_init();
    gap.init(device_name_cmplt, device_name_short);
    gatt.init();

    nimble_port_freertos_init(ble_htp_prph_host_task);
}

