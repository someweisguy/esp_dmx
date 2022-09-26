#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dmx_types.h"
#include "rdm_types.h"

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
#define UID2STR(uid) ((uint16_t)(uid >> 32)), ((uint32_t)(uid))

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
 * @brief Helper function to parse RDM data into a user-friendly format.
 *
 * @param data A pointer to the RDM data buffer.
 * @param size The size of the RDM data buffer.
 * @param[out] event A structure containing information about the RDM packet.
 * @return true if the data is a valid RDM packet.
 * @return false if the data is not a valid RDM packet.
 */
bool rdm_parse(void *data, size_t size, rdm_event_t *event);

// TODO: docs
size_t rdm_send_disc_response(dmx_port_t dmx_num);

// TODO: docs

size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                   rdm_disc_unique_branch_t *params,
                                   rdm_response_t *response, rdm_uid_t *uid);

// TODO: docs
size_t rdm_send_disc_mute(dmx_port_t dmx_num, rdm_uid_t uid, bool mute,
                          rdm_response_t *response,
                          rdm_disc_mute_t *mute_params);

// TODO: docs
size_t rdm_discover_devices(dmx_port_t dmx_num, size_t size, rdm_uid_t *uids);


// TODO: docs
// size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_uid_t uid,
//                            uint16_t sub_device, rdm_device_info_t *device_info,
//                            bool *response_is_valid);

// TODO: get software version label
// size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_uid_t uid,
//                                       uint16_t sub_device,
//                                       rdm_software_version_label_t *param,
//                                       bool *response_is_valid);

// TODO: get/set identify device

// TODO: get/set dmx start address

// TODO: get supported parameters

// TODO: get parameter description

#ifdef __cplusplus
}
#endif
