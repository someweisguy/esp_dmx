#include "esp_rdm.h"

#include <string.h>

#include "esp_system.h"

static uint64_t rdm_uid = 0;  // The 48-bit unique ID of this device.

uint64_t rdm_get_uid() { 
  // Initialize the RDM UID
  if (rdm_uid == 0) {
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    rdm_uid = (uint64_t)0xbeef << 32;  // FIXME: use real manufacturer ID
    memcpy(&rdm_uid, mac, 4);
  }

  return rdm_uid;
}

void rdm_set_uid(uint64_t uid) { 
  rdm_uid = uid;
}