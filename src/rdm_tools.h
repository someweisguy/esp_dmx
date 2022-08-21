#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dmx_types.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The recommended method for representing the UID in text by separating
 * the manufacturer ID and the device ID. For use with printf-like functions.
 */
#define UIDSTR "%04x:%08x"

/**
 * @brief Used to generate arguments for the UIDSTR macro for representing the
 * UID in text by separating the manufacturer ID and device ID. For use with
 * printf-like functions.
 */
#define UID2STR(uid) \
  ((rdm_uid_t *)(&uid))->manufacturer_id, ((rdm_uid_t *)(&uid))->device_id


/**
 * @brief Helper function that takes an RDM UID from a most-significant-byte
 * first buffer and copies it to least-significant-byte first endianness, which
 * is what ESP32 uses.
 *
 * @note This function is designed to be the quickest way to swap endianness of
 * a 48-bit number on the Xtensa compiler which is important because it will be
 * used in an interrupt handler. It must be inlined in order to prevent cache 
 * misses in IRAM interrupts.
 *
 * @param buf A pointer to an RDM buffer.
 * @return uint64_t The properly formatted RDM UID.
 */
uint64_t uidcpy(const void *buf);

/**
 * @brief Returns the 48 bit unique ID of this device.
 * 
 * @return The UID of this device.
 */
uint64_t rdm_get_uid();

/**
 * @brief Set the device UID to a custom value. Setting the UID to 0 will reset 
 * the UID to its default value.
 * 
 * @param uid The custom value to which to set the device UID.
 */
void rdm_set_uid(uint64_t uid);

/**
 * @brief // TODO
 * 
 * @param data 
 * @param size 
 * @param[out] event 
 * @return true if the data is a valid RDM packet.
 * @return false if the data is not a valid RDM packet.
 */
bool rdm_parse(void *data, size_t size, rdm_event_t *event);

bool rdm_send_discovery_response(dmx_port_t dmx_num, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif
