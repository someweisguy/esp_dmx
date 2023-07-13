#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets a copy of the RDM device info of this device.
 * 
 * @param dmx_num The DMX port number.
 * @param[out] device_info A pointer which stores a copy of the device info of 
 * this device.
 * @return true on success. 
 * @return false on failure.
 */
bool rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info);

/**
 * @brief Gets a copy of the RDM software version label of this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] software_version_label A pointer which stores a copy of the
 * software version label of this device.
 * @param[inout] size A pointer to the size of the software_version_label
 * buffer. Is set to the size of the software_version_label on success.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_get_software_version_label(dmx_port_t dmx_num,
                                    char *software_version_label, size_t *size);

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

#ifdef __cplusplus
}
#endif
