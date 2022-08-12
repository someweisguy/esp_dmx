#include "esp_rdm.h"

#include <string.h>

#include "endian.h"
#include "esp_system.h"

static rdm_uid_t rdm_uid = {};  // The 48-bit unique ID of this device.

rdm_uid_t rdm_get_uid() { 
  // Initialize the RDM UID
  if (rdm_uid.raw == 0) {
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    rdm_uid.manufacturer_id = 0xbeef;  // FIXME: use real manufacturer ID
    rdm_uid.device_id = bswap32(*(uint32_t *)(mac + 2));  // Don't use MAC OUI
  }

  return rdm_uid;
}

void rdm_set_uid(rdm_uid_t uid) { 
  rdm_uid = uid;
}