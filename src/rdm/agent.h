/**
 * @file agent.h
 * @author Mitch Weisbrod
 * @brief This file contains functions needed to perform necessary RDM
 * operations, such as reading, writing, sending, and registering responder
 * callbacks.
 */
#pragma once

#include "esp_dmx.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets the DMX start address of this device. To get the DMX start
 * address of a responder device, use `rdm_get_dmx_start_address()`.
 *
 * @param dmx_num The DMX port number.
 * @return The DMX start address of this device or 0 on failure.
 */
uint16_t rdm_get_dmx_start_address(dmx_port_t dmx_num);

/**
 * @brief Sets the DMX start address of this device. To set the DMX start
 * address of a responder device, use `rdm_set_dmx_start_address()`.
 *
 * @param dmx_num The DMX port number.
 * @param start_address The DMX start address to which to set this device. Must
 * be between 1 and 512 (inclusive).
 */
void rdm_set_dmx_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address);

#ifdef __cplusplus
}
#endif
