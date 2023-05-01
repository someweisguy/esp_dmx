#pragma once

#include "esp_dmx.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Returns the 48 bit unique ID of this device.
 *
 * @param dmx_num The DMX port number.
 * @return The UID of the DMX port.
 */
rdm_uid_t rdm_get_uid(dmx_port_t dmx_num);

/**
 * @brief Set the device UID to a custom value. Setting the UID to 0 will reset
 * the UID to its default value.
 *
 * @param dmx_num The DMX port number.
 * @param uid The custom value to which to set the device UID. Must be less than
 * or equal to RDM_MAX_UID.
 */
void rdm_set_uid(dmx_port_t dmx_num, rdm_uid_t uid);

/**
 * @brief Returns true if RDM discovery responses are be muted on this device.
 *
 * @param dmx_num The DMX port number.
 * @return true if RDM discovery is muted.
 * @return false if RDM discovery is not muted.
 */
bool rdm_is_muted(dmx_port_t dmx_num);

// TODO: docs
bool rdm_set_device_info(dmx_port_t dmx_num,
                         const rdm_device_info_t *device_info);

#ifdef __cplusplus
}
#endif
