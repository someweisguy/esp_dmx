#include "include/nvs.h"

#include "dmx/struct.h"
#include "esp_dmx.h"
#include "nvs_flash.h"

#ifndef CONFIG_DMX_NVS_PARTITION_NAME
#define DMX_NVS_PARTITION_NAME "nvs"
#else
#define DMX_NVS_PARTITION_NAME CONFIG_DMX_NVS_PARTITION_NAME
#endif

static const char *dmx_nvs_namespace = "esp_dmx";

void dmx_nvs_init(dmx_port_t dmx_num) {
  nvs_flash_init_partition(DMX_NVS_PARTITION_NAME);
}

size_t dmx_nvs_get(dmx_port_t dmx_num, rdm_pid_t pid,
                   rdm_sub_device_t sub_device, rdm_ds_t ds, void *param,
                   size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0 && pid <= 0xffff);
  assert(sub_device < 513);
  assert(param != NULL);

  if (size == 0) {
    return size;
  }

  // Get the NVS key
  char key[9];
  const int w = snprintf(key, sizeof(key), "%x%x%x", dmx_num, pid, sub_device);
  assert(w < sizeof(key));

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(dmx_nvs_namespace, NVS_READONLY, &nvs);
  if (!err) {
#ifndef DMX_ISR_IN_IRAM
    // Track which drivers are currently enabled and disable those which are
    bool driver_is_enabled[DMX_NUM_MAX];
    for (int i = 0; i < DMX_NUM_MAX; ++i) {
      driver_is_enabled[i] = dmx_driver_is_enabled(i);
      if (dmx_driver_is_installed(i)) {
        dmx_driver_disable(i);
      }
    }
#endif

    // Read the parameter from NVS depending on its type
    switch (ds) {
      case RDM_DS_ASCII:
        err = nvs_get_str(nvs, key, param, &size);
        break;
      case RDM_DS_UNSIGNED_BYTE:
        err = nvs_get_u8(nvs, key, param);
        size = sizeof(uint8_t);
        break;
      case RDM_DS_SIGNED_BYTE:
        err = nvs_get_i8(nvs, key, param);
        size = sizeof(int8_t);
        break;
      case RDM_DS_UNSIGNED_WORD:
        err = nvs_get_u16(nvs, key, param);
        size = sizeof(uint16_t);
        break;
      case RDM_DS_SIGNED_WORD:
        err = nvs_get_i16(nvs, key, param);
        size = sizeof(int16_t);
        break;
      case RDM_DS_UNSIGNED_DWORD:
        err = nvs_get_u32(nvs, key, param);
        size = sizeof(uint32_t);
        break;
      case RDM_DS_SIGNED_DWORD:
        err = nvs_get_i32(nvs, key, param);
        size = sizeof(int32_t);
        break;
      default:
        err = nvs_get_blob(nvs, key, param, &size);
    }

#ifndef DMX_ISR_IN_IRAM
    for (int i = 0; i < DMX_NUM_MAX; ++i) {
      if (driver_is_enabled[i]) {
        dmx_driver_enable(i);
      }
    }
#endif

    nvs_close(nvs);
  }

  if (err) {
    size = 0;
  }

  return size;
}

bool dmx_nvs_set(dmx_port_t dmx_num, rdm_pid_t pid, rdm_sub_device_t sub_device,
                 rdm_ds_t ds, const void *param, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0 && pid <= 0xffff);
  assert(sub_device < 513);
  assert(param != NULL);

  if (size == 0) {
    return true;
  }

  // Get the NVS key
  char key[9];
  const int w = snprintf(key, sizeof(key), "%x%x%x", dmx_num, pid, sub_device);
  assert(w < sizeof(key));

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(dmx_nvs_namespace, NVS_READWRITE, &nvs);
  if (!err) {
#ifndef DMX_ISR_IN_IRAM
    // Track which drivers are currently enabled and disable those which are
    bool driver_is_enabled[DMX_NUM_MAX];
    for (int i = 0; i < DMX_NUM_MAX; ++i) {
      driver_is_enabled[i] = dmx_driver_is_enabled(i);
      if (dmx_driver_is_installed(i)) {
        dmx_driver_disable(i);
      }
    }
#endif

    // Write the parameter to NVS depending on its type
    switch (ds) {
      case RDM_DS_ASCII:
        err = nvs_set_str(nvs, key, param);
        break;
      case RDM_DS_UNSIGNED_BYTE:
        err = nvs_set_u8(nvs, key, *(uint8_t *)param);
        break;
      case RDM_DS_SIGNED_BYTE:
        err = nvs_set_i8(nvs, key, *(int8_t *)param);
        break;
      case RDM_DS_UNSIGNED_WORD:
        err = nvs_set_u16(nvs, key, *(uint16_t *)param);
        break;
      case RDM_DS_SIGNED_WORD:
        err = nvs_set_i16(nvs, key, *(int16_t *)param);
        break;
      case RDM_DS_UNSIGNED_DWORD:
        err = nvs_set_u32(nvs, key, *(uint32_t *)param);
        break;
      case RDM_DS_SIGNED_DWORD:
        err = nvs_set_i32(nvs, key, *(int32_t *)param);
        break;
      default:
        err = nvs_set_blob(nvs, key, param, size);
    }
    if (!err) {
      err = nvs_commit(nvs);
    }

#ifndef DMX_ISR_IN_IRAM
    for (int i = 0; i < DMX_NUM_MAX; ++i) {
      if (driver_is_enabled[i]) {
        dmx_driver_enable(i);
      }
    }
#endif

    nvs_close(nvs);
  }

  return (err == ESP_OK);
}
