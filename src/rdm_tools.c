#include "rdm_tools.h"

#include <stdint.h>
#include <string.h>

#include "dmx_constants.h"
#include "endian.h"
#include "esp_log.h"
#include "esp_system.h"
#include "impl/driver.h"

/**
 * @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID or may
 * choose to register their own manufacturer ID.
 */
#define RDM_DEFAULT_MANUFACTURER_ID (0x05e0)

static const char *TAG = "rdm";

static uint64_t rdm_uid = 0;  // The 48-bit unique ID of this device.

inline __attribute__((always_inline)) uint64_t uidcpy(const void *buf) {
  uint64_t val;
  ((uint8_t *)&val)[5] = ((uint8_t *)buf)[0];
  ((uint8_t *)&val)[4] = ((uint8_t *)buf)[1];
  ((uint8_t *)&val)[3] = ((uint8_t *)buf)[2];
  ((uint8_t *)&val)[2] = ((uint8_t *)buf)[3];
  ((uint8_t *)&val)[1] = ((uint8_t *)buf)[4];
  ((uint8_t *)&val)[0] = ((uint8_t *)buf)[5];
  return val;
}

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

bool rdm_send_discovery_response(dmx_port_t dmx_num, TickType_t ticks_to_wait) {
  // TODO: check args

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Block until the mutex can be taken and decrement timeout accordingly
  const TickType_t start_tick = xTaskGetTickCount();
  if (!xSemaphoreTakeRecursive(driver->mux, ticks_to_wait)) {
    return false;
  }
  ticks_to_wait -= xTaskGetTickCount() - start_tick;

  // Build the discovery response packet
  uint8_t disc_response[24] = {RDM_PREAMBLE, RDM_PREAMBLE, RDM_PREAMBLE,
                               RDM_PREAMBLE, RDM_PREAMBLE, RDM_PREAMBLE,
                               RDM_PREAMBLE, RDM_DELIMITER};
  const uint64_t uid = rdm_get_uid();
  uint16_t checksum = 0;
  for (int i = 8, j = 5; i < 20; i += 2, --j) {
    disc_response[i] = ((uint8_t *)&uid)[j] | 0xaa;
    disc_response[i + 1] = ((uint8_t *)&uid)[j] | 0x55;
    checksum += ((uint8_t *)&uid)[j] + (0xaa + 0x55);
  }
  disc_response[20] = (checksum >> 8) | 0xaa;
  disc_response[21] = (checksum >> 8) | 0x55;
  disc_response[22] = checksum | 0xaa;
  disc_response[23] = checksum | 0x55;

  // Write and send the response
  dmx_write(dmx_num, disc_response, sizeof(disc_response));
  const size_t sent = dmx_send(dmx_num, sizeof(disc_response), ticks_to_wait);

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return sent > 0;
}
