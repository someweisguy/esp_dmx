#pragma once

#include <stdint.h>

#include "esp_dmx.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief A callback function type for use with rdm_discover_with_callback().
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID of the found device.
 * @param device_number The number of the found device.
 * @param[in] mute_params A pointer to the received mute parameters from the
 * device.
 * @param[inout] context A pointer to user-provided context.
 */
typedef void(rdm_discovery_cb_t)(dmx_port_t dmx_num, rdm_uid_t uid,
                                 size_t device_num,
                                 rdm_disc_mute_t *mute_params, void *context);

// TODO: docs
size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num, rdm_header_t *header,
                                   const rdm_disc_unique_branch_t *param,
                                   rdm_ack_t *ack);

// TODO: docs
size_t rdm_send_disc_mute(dmx_port_t dmx_num, rdm_header_t *header,
                          rdm_ack_t *ack, rdm_disc_mute_t *param);

// TODO: docs
size_t rdm_send_disc_un_mute(dmx_port_t dmx_num, rdm_header_t *header,
                             rdm_ack_t *ack, rdm_disc_mute_t *param);

/**
 * @brief Performs the RDM device discovery algorithm and executes a callback
 * function when a new device is discovered.
 *
 * @note This discovery algorithm is not recursive like the RDM technical
 * standard suggests, but iterative. It requires the allocation of 784 bytes of
 * data to store a list of discovery requests that must be made. By default,
 * this data is heap allocated but may be stack allocated by configuring
 * settings in the ESP-IDF sdkconfig.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback function which is called when a new device is found.
 * @param[inout] context Context which is passed to the callback function when a
 * new device is found.
 * @return The number of devices found.
 */
size_t rdm_discover_with_callback(dmx_port_t dmx_num, rdm_discovery_cb_t cb,
                                  void *context);

/**
 * @brief Performes the RDM device discovery algorithm with a default callback
 * function to store the UIDs of found devices in an array.
 *
 * @note This discovery algorithm is not recursive like the RDM technical
 * standard suggests, but iterative. It requires the allocation of 784 bytes of
 * data to store a list of discovery requests that must be made. By default,
 * this data is heap allocated but may be stack allocated by configuring
 * settings in the ESP-IDF sdkconfig.
 *
 * @param dmx_num The DMX port number.
 * @param[out] uids An array of UIDs used to store found device UIDs.
 * @param size The size of the provided UID array.
 * @return The number of devices found. // TODO: return int, not size_t
 */
size_t rdm_discover_devices_simple(dmx_port_t dmx_num, rdm_uid_t *uids,
                                   const size_t size);

#ifdef __cplusplus
}
#endif