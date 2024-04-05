#include <esp_err.h>
#include "nvs_utils.h"

#define MAX_STR_SIZE 16

void app_main() {
  char body_loc_str[MAX_STR_SIZE];
  size_t str_size = MAX_STR_SIZE;

  nvs_init();

  write_string_to_nvs("BodyLoc", "pelvis");

  read_string_from_nvs("BodyLoc", body_loc_str, &str_size);
  printf("Read BodyLoc from NVS: %s\n", body_loc_str);
}
