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
 * @brief Returns true if RDM discovery responses are be muted on this device.
 *
 * @param dmx_num The DMX port number.
 * @return true if RDM discovery is muted.
 * @return false if RDM discovery is not muted.
 */
bool rdm_driver_is_muted(dmx_port_t dmx_num);

/**
 * @brief Sets the device info for this device.
 *
 * @param dmx_num The DMX port number.
 * @param[in] device_info A pointer to a struct which stores the device info to
 * copy to this device.
 */
void rdm_driver_set_device_info(dmx_port_t dmx_num,
                                const rdm_device_info_t *device_info);

/**
 * @brief Gets the DMX start address of this device. To get the DMX start
 * address of a responder device, use `rdm_get_dmx_start_address()`.
 *
 * @param dmx_num The DMX port number.
 * @return The DMX start address of this device or 0 on failure.
 */
int rdm_driver_get_dmx_start_address(dmx_port_t dmx_num);

/**
 * @brief Sets the DMX start address of this device. To set the DMX start
 * address of a responder device, use `rdm_set_dmx_start_address()`.
 *
 * @param dmx_num The DMX port number.
 * @param start_address The DMX start address to which to set this device. Must
 * be between 1 and 512 (inclusive).
 */
void rdm_driver_set_dmx_start_address(dmx_port_t dmx_num, int start_address);

// TODO: docs
bool rdm_register_disc_unique_branch(dmx_port_t dmx_num);

// TODO: docs
bool rdm_register_disc_mute(dmx_port_t dmx_num);

// TODO: docs
bool rdm_register_disc_un_mute(dmx_port_t dmx_num);

// TODO: docs
bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info);

// TODO: docs
bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         const char *software_version_label);

// TODO: docs
bool rdm_register_identify_device(dmx_port_t dmx_num);

// TODO: docs
bool rdm_register_dmx_start_address(dmx_port_t dmx_num,
                                    uint16_t *dmx_start_address);

#ifdef __cplusplus
}
#endif
