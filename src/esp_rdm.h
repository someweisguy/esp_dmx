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
typedef union {
  struct __attribute__((__packed__)) {
    uint32_t device_id;        // The device ID of the RDM device.
    uint16_t manufacturer_id;  // The manufacturer ID of the RDM device.
  };
  uint64_t raw;  // The raw value of the UID as a unsigned, 64-bit integer.
} rdm_uid_t;

/**
 * @brief UID which indicates an RDM packet is being broadcast. Responders shall
 * not respond to RDM broadcast messages.
 */
static const rdm_uid_t RDM_BROADCAST_UID = {.raw = 0xffffffffffff};

/**
 * @brief Returns the 48 bit unique ID of this device.
 * 
 * @return The UID of this device.
 */
rdm_uid_t rdm_get_uid();

/**
 * @brief Set the device UID to a custom value. Setting the UID to 0 will reset 
 * the UID to its default value.
 * 
 * @param uid The custom value to which to set the device UID.
 */
void rdm_set_uid(rdm_uid_t uid);

/**
 * @brief // TODO
 * 
 * @param data 
 * @param size 
 * @param event 
 * @return void* a pointer to the RDM parameter data or NULL on failure.
 */
void *rdm_parse(void *data, size_t size, dmx_event_t *event);

#ifdef __cplusplus
}
#endif
