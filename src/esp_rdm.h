#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UID which indicates an RDM packet is being broadcast. Responders shall
 * not respond to RDM broadcast messages.
 */
static const uint64_t RDM_BROADCAST_UID = 0xffffffffffff;

/**
 * @brief Returns the 48 bit unique ID of this device.
 * 
 * @return The UID of this device.
 */
uint64_t rdm_get_uid();

/**
 * @brief Set the device UID to a custom value.
 * 
 * @param uid The custom value to which to set the device UID.
 */
void rdm_set_uid(uint64_t uid);

#ifdef __cplusplus
}
#endif
