#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_central.h"
#include "gap.h"
#include "gatt.h"
#include "imu.h"
#include "nvs_utils.h"
#include "wifi.h"
#include "udp_client.h"
#include "mqtt.h"

static constexpr const char *device_name_cmplt = "imu_central";

// must be true when device has not passed calibration yet,
// or has not passed calibration for a long time
constexpr bool RECALIB_ACCEL_GYRO = false;

extern "C" void ble_store_config_init();

void ble_imu_central_host_task(void *param)
{
  ble_store_config_init();
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
  ble_hs_cfg.sync_cb = GAP::ble_central_on_sync;
  // gets executed when the host resets itself and the controller due to fatal error
  ble_hs_cfg.reset_cb = GAP::ble_central_on_reset;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
}

extern "C" void app_main() {
  nvs_init();
  imu.init(SensorFusionAlgorithm::BNO055_BUILTIN);
  imu.run_calibration(RECALIB_ACCEL_GYRO);
  imu.start_measurements(&gatt.imu_queue_handle);
  nimble_init();
  gap.init(device_name_cmplt);
  gatt.init();
  wifi.init();
  udp_client.init();
  mqtt_manager.init();
  nimble_port_freertos_init(ble_imu_central_host_task);
}
