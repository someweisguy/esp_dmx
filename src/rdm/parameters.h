/**
 * @file parameters.h
 * @author Mitch Weisbrod
 * @brief This file contains getters and setters for various RDM parameters. It
 * is included with `rdm/responder.h`.
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets a copy of the RDM identify device state of this device.
 *
 * @param dmx_num The DMX port number.
 * @param identify A pointer which stores a copy of the identify device state of
 * this device.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_get_identify_device(dmx_port_t dmx_num, uint8_t *identify);

/**
 * @brief Sets the RDM identify device state of this device.
 *
 * @param dmx_num The DMX port number.
 * @param identify The identify device state to which to set this device.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_set_identify_device(dmx_port_t dmx_num, const uint8_t identify);

/**
 * @brief Gets a copy of the DMX start address of this device.
 *
 * @param dmx_num The DMX port number.
 * @param dmx_start_address A pointer which stores a copy of the DMX start
 * address of this device.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_get_dmx_start_address(dmx_port_t dmx_num, uint16_t *dmx_start_address);

/**
 * @brief Sets the DMX start address of this device. The DMX start address of
 * this device may not be set when the DMX start address is set to
 * DMX_START_ADDRESS_NONE.
 *
 * @param dmx_num The DMX port number.
 * @param dmx_start_address The DMX start address to which to set this device.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address);

// TODO docs
bool rdm_get_current_personality(dmx_port_t dmx_num,
                                 rdm_dmx_personality_t *personality);

// TODO: docs
bool rdm_set_current_personality(dmx_port_t dmx_num, uint8_t personality_num);

/** 
 * @brief Gets the device label.
 * 
 * @param dmx_num The DMX port number.
 * @param[out] label A pointer to a buffer that the device_label will be copied into.
 *                   This will not contain a trailing '\0'
 * @param labelLen The size of @p label
 * @return The number of bytes copied
*/
size_t rdm_get_device_label(dmx_port_t dmx_num, char * label, size_t label_len);

#ifdef __cplusplus
}
#endif
