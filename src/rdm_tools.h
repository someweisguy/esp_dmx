#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dmx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief A type for evaluating RDM UIDs. Allows for easy string formatting UIDs
 * to RDM specification.
 */
typedef struct __attribute__((__packed__)) {
  uint32_t device_id;        // The device ID of the RDM device.
  uint16_t manufacturer_id;  // The manufacturer ID of the RDM device.
} dmx_uid_t;

/**
 * @brief UID which indicates an RDM packet is being broadcast. Responders shall
 * not respond to RDM broadcast messages.
 */
static const uint64_t DMX_BROADCAST_UID = 0xffffffffffff;

/**
 * @brief Returns the 48 bit unique ID of this device.
 * 
 * @return The UID of this device.
 */
uint64_t dmx_get_uid();

/**
 * @brief Set the device UID to a custom value. Setting the UID to 0 will reset 
 * the UID to its default value.
 * 
 * @param uid The custom value to which to set the device UID.
 */
void dmx_set_uid(uint64_t uid);

/**
 * @brief // TODO
 * 
 * @param data 
 * @param size 
 * @param[out] event 
 * @return true if the data is a valid RDM packet.
 * @return false if the data is not a valid RDM packet.
 */
bool dmx_parse_rdm(void *data, size_t size, dmx_event_t *event);

#ifdef __cplusplus
}
#endif
