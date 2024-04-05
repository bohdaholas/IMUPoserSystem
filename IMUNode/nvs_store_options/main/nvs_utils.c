#include <esp_err.h>
#include <nvs_flash.h>
#include "nvs_utils.h"

void nvs_init() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void write_string_to_nvs(const char* key, const char* value) {
  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  ESP_ERROR_CHECK(ret);

  ret = nvs_set_str(nvs_handle, key, value);
  ESP_ERROR_CHECK(ret);

  nvs_close(nvs_handle);
}

void read_string_from_nvs(const char* key, char* value, size_t* length) {
  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open("storage", NVS_READONLY, &nvs_handle);
  ESP_ERROR_CHECK(ret);

  // Read the size of memory needed for the string into "length"
  ret = nvs_get_str(nvs_handle, key, NULL, length);
  ESP_ERROR_CHECK(ret);

  // Read the string into "value"
  ret = nvs_get_str(nvs_handle, key, value, length);
  ESP_ERROR_CHECK(ret);
  nvs_close(nvs_handle);
}
