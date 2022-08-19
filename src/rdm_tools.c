#include "rdm_tools.h"

#include <string.h>

#include "dmx_caps.h"
#include "endian.h"
#include "esp_system.h"

/**
 * @brief This is the RDM Manufacturer ID that was registered with ESTA for this 
 * use with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID or may
 * choose to register their own manufacturer ID.
 */
#define RDM_DEFAULT_MANUFACTURER_ID (0x05e0)

static uint64_t rdm_uid = 0;  // The 48-bit unique ID of this device.

uint64_t rdm_get_uid() { 
  // Initialize the RDM UID
  if (rdm_uid == 0) {
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    ((rdm_uid_t *)(&rdm_uid))->manufacturer_id = RDM_DEFAULT_MANUFACTURER_ID;
    ((rdm_uid_t *)(&rdm_uid))->device_id = bswap32(*(uint32_t *)(mac + 2));
  }

  return rdm_uid;
}

void rdm_set_uid(uint64_t uid) { 
  rdm_uid = uid;
}

bool rdm_parse(void *data, size_t size, rdm_event_t *event) {

  const rdm_data_t *const rdm = (rdm_data_t *)data;
  
  if ((rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER) && size > 17) {
    // Find the length of the discovery response preamble (0-7 bytes)
    int preamble_len = 0;
    const uint8_t *response = data;
    for (; preamble_len < 7; ++preamble_len) {
      if (response[preamble_len] == RDM_DELIMITER) {
        break;
      }
    }
    if (response[preamble_len] != RDM_DELIMITER || size < preamble_len + 17) {
      return false;  // Not a valid discovery response
    }

    // Decode the 6-byte UID and get the packet sum
    uint64_t uid = 0;
    uint16_t sum = 0;
    response = &((uint8_t *)data)[preamble_len + 1];
    for (int i = 5, j = 0; i >= 0; --i, j += 2) {
      ((uint8_t *)&uid)[i] = response[j] & 0x55;
      ((uint8_t *)&uid)[i] |= response[j + 1] & 0xaa;
      sum += ((uint8_t *)&uid)[i] + 0xff;
    }

    // Decode the checksum received in the response
    uint16_t checksum;
    for (int i = 1, j = 12; i >= 0; --i, j += 2) {
      ((uint8_t *)&checksum)[i] = response[j] & 0x55;
      ((uint8_t *)&checksum)[i] |= response[j + 1] & 0xaa;
    }

    // Return DMX data to the caller
    event->source_uid = uid;
    event->checksum_is_valid = (sum == checksum);
    return true;

  } else if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC &&
             rdm->message_len >= size) {
    // TODO:
  }

  return false;
}