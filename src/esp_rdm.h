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
typedef rdm_response_type_t (*rdm_response_cb_t)(dmx_port_t dmx_num,
                                                 const rdm_header_t *header,
                                                 rdm_mdb_t *mdb, void *context);


// TODO: docs
size_t rdm_send(dmx_port_t dmx_num, rdm_header_t *header,
                const rdm_encode_t *encode, rdm_decode_t *decode,
                rdm_ack_t *ack);

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
 * @return The number of devices found.
 */
size_t rdm_discover_devices_simple(dmx_port_t dmx_num, rdm_uid_t *uids,
                                   const size_t size);

/**
 * @brief Sends an RDM SUPPORTED_PARAMETERS request and reads the response, if
 * any.
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID to which to send the request.
 * @param sub_device The sub-device to which to address the request.
 * @param[out] response A pointer into which to store the RDM response summary.
 * @param[out] pids An array of PIDs that the responding device supports.
 * @param size The size of the provided PID array.
 * @return The number of PIDs supported by the responding device, which may be
 * less than the number of PIDs copied to the provided array.
 */
// size_t rdm_get_supported_parameters(dmx_port_t dmx_num, rdm_uid_t uid,
//                                     rdm_sub_device_t sub_device,
//                                     rdm_response_t *response, rdm_pid_t *pids,
//                                     size_t size);

/**
 * @brief Sends an RDM DEVICE_INFO request and reads the response, if any.
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID to which to address the request.
 * @param sub_device The sub-device to which to address the request.
 * @param[out] response A pointer into which to store the RDM response summary.
 * @param[out] device_info A pointer to a struct which stores the device info of
 * the responding device.
 * @return 1 if a successful response is received. Otherwise, 0.
 */
// size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_uid_t uid,
//                            rdm_sub_device_t sub_device,
//                            rdm_response_t *response,
//                            rdm_device_info_t *device_info);

/**
 * @brief Sends an RDM SOFTWARE_VERSION_LABEL request and reads the response, if
 * any.
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID to which to address the request.
 * @param sub_device The sub-device to which to address the request.
 * @param response A pointer into which to store the RDM response summary.
 * @param label A string which stores the software version label of the
 * responding device. Should be 33 characters max.
 * @param size The size of the provided string.
 * @return The size of the null-terminated string that was sent, which may be
 * smaller than the string that was copied.
 */
// size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_uid_t uid,
//                                       rdm_sub_device_t sub_device,
//                                       rdm_response_t *response, char *label,
//                                       size_t size);

/**
 * @brief Sends an RDM DMX_START_ADDRESS GET request and reads the response, if
 * any.
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID to which to address the request.
 * @param sub_device The sub-device to which to address the request.
 * @param response A pointer into which to store the RDM response summary.
 * @param start_address A pointer into which to store the response start
 * address.
 * @return 1 if a successful response is received. Otherwise, 0.
 */
// size_t rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
//                                  rdm_sub_device_t sub_device,
//                                  rdm_response_t *response, int *start_address);

/**
 * @brief Sends an RDM DMX_START_ADDRESS SET request.
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID to which to address the request.
 * @param sub_device The sub-device to which to address the request.
 * @param response A pointer into which to store the RDM response summary.
 * @param start_address The start address to which to set the device or devices.
 * @return true if the request was successful.
 * @return false if the request was not successful.
 */
// bool rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
//                                rdm_sub_device_t sub_device,
//                                rdm_response_t *response, int start_address);

/**
 * @brief Sends an RDM IDENTIFY_DEVICE GET request and reads the response, if
 * any.
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID to which to address the request.
 * @param sub_device The sub-device to which to address the request.
 * @param response A pointer into which to store the RDM response summary.
 * @param identify A pointer into which to store the response identify state.
 * @return 1 if a successful response is received. Otherwise, 0.
 */
// size_t rdm_get_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
//                                rdm_sub_device_t sub_device,
//                                rdm_response_t *response, bool *identify);

/**
 * @brief Sends an RDM IDENTIFY_DEVICE SET request.
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID to which to address the request.
 * @param sub_device The sub-device to which to address the request.
 * @param response A pointer into which to store the RDM response summary.
 * @param identify The identify state to which to set the device or devices.
 * @return true if the request was successful.
 * @return false if the request was not successful.
 */
// bool rdm_set_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
//                              rdm_sub_device_t sub_device,
//                              rdm_response_t *response, bool identify);

#ifdef __cplusplus
}
#endif
