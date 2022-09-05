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
uint64_t buf_to_uid(const void *buf);

/**
 * @brief Helper function that converts an RDM UID stored as a 64-bit integer
 * and copies it into a 48-bit buffer. It also converts endianness to compensate
 * for the fact that the ESP32 stores values in least-significant-byte first 
 * endianness and RDM requires most-significant-byte first.
 * 
 * @param buf A pointer to the destination buffer.
 * @param uid The 64-bit representation of the UID.
 * @return void* A pointer to the destination buffer.
 */
void *uid_to_buf(void *buf, uint64_t uid);

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

// TODO: docs
size_t rdm_send_disc_response(dmx_port_t dmx_num);

size_t rdm_send_disc_un_mute(dmx_port_t dmx_num, uint64_t uid,
                             dmx_event_t *event, size_t *num_params,
                             rdm_disc_mute_param_t *params);

size_t rdm_send_disc_mute(dmx_port_t dmx_num, uint64_t uid, dmx_event_t *event,
                          size_t *num_params, rdm_disc_mute_param_t *params);

#ifdef __cplusplus
}
#endif
