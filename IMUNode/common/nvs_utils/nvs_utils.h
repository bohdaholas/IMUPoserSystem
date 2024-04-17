#ifndef NVS_STORE_OPTIONS_NVS_H
#define NVS_STORE_OPTIONS_NVS_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void nvs_init();
void write_string_to_nvs(const char *key, const char *value);
void read_string_from_nvs(const char *key, char *value, size_t *length);
void write_bytes_to_nvs(const char *key, const char *value, size_t length);
void read_bytes_from_nvs(const char *key, char *value, size_t *length);

#ifdef __cplusplus
};
#endif

#endif //NVS_STORE_OPTIONS_NVS_H
