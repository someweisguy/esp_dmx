/**
 * @file requests.h
 * @author Mitch Weisbrod
 * @brief This file contains functions needed to send requests to RDM
 * responders.
 */
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

// TODO: size_t rdm_send_generic_request()

/**
 * @brief Sends an RDM discovery request and reads the response, if any.
 *
 * @param dmx_num The DMX port number.
 * @param[out] header A pointer which contains header information from the
 * received RDM response, if any.
 * @param[in] param A pointer to the discovery UID bounds to send.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @return The number of bytes received in response to the request.
 */
bool rdm_send_disc_unique_branch(dmx_port_t dmx_num, rdm_header_t *header,
                                 const rdm_disc_unique_branch_t *param,
                                 rdm_ack_t *ack);

/**
 * @brief Sends an RDM discovery mute request and reads the response, if any.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which contains header information of the
 * request. Upon receiving a response, header information from the response is
 * copied into this pointer.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @param[out] param A pointer into which the discovery mute parameters from the
 * responder are stored.
 * @return The number of bytes received in response to the request.
 */
bool rdm_send_disc_mute(dmx_port_t dmx_num, rdm_header_t *header,
                        rdm_ack_t *ack, rdm_disc_mute_t *param);

/**
 * @brief Sends an RDM discovery un-mute request and reads the response, if any.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which contains header information of the
 * request. Upon receiving a response, header information from the response is
 * copied into this pointer.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @param[out] param A pointer into which the discovery mute parameters from the
 * responder are stored.
 * @return The number of bytes received in response to the request.
 */
bool rdm_send_disc_un_mute(dmx_port_t dmx_num, rdm_header_t *header,
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
int rdm_discover_with_callback(dmx_port_t dmx_num, rdm_discovery_cb_t cb,
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
int rdm_discover_devices_simple(dmx_port_t dmx_num, rdm_uid_t *uids,
                                const size_t size);

/**
 * @brief Sends an RDM get device info request and reads the response, if any.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which contains header information of the
 * request. Upon receiving a response, header information from the response is
 * copied into this pointer.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @param[out] param A pointer into which the device info parameter from the
 * responder is stored.
 * @return The number of bytes received in response to the request.
 */
bool rdm_get_device_info(dmx_port_t dmx_num, rdm_header_t *header,
                         rdm_ack_t *ack, rdm_device_info_t *param);

/**
 * @brief Sends an RDM get software version label request and reads the
 * response, if any.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which contains header information of the
 * request. Upon receiving a response, header information from the response is
 * copied into this pointer.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @param[out] sw_version_label A pointer into which the software version label
 * parameter from the responder is stored. The string should be 33 chars long,
 * maximum.
 * @param size The size of the sw_version_label string.
 * @return The number of bytes received in response to the request.
 */
bool rdm_get_software_version_label(dmx_port_t dmx_num, rdm_header_t *header,
                                    rdm_ack_t *ack, char *sw_version_label,
                                    size_t size);

/**
 * @brief Sends an RDM get identify device request and reads the response, if
 * any.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which contains header information of the
 * request. Upon receiving a response, header information from the response is
 * copied into this pointer.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @param[out] identify A pointer into which the identify device parameter from
 * the responder is stored.
 * @return The number of bytes received in response to the request.
 */
bool rdm_get_identify_device(dmx_port_t dmx_num, rdm_header_t *header,
                             rdm_ack_t *ack, uint8_t *identify);

/**
 * @brief Sends an RDM set identify device request and reads the response, if
 * any.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which contains header information of the
 * request. Upon receiving a response, header information from the response is
 * copied into this pointer.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @param identify The value to which to set the identify device parameter of
 * the target device(s).
 * @return The number of bytes received in response to the request.
 */
bool rdm_set_identify_device(dmx_port_t dmx_num, rdm_header_t *header,
                             uint8_t identify, rdm_ack_t *ack);

/**
 * @brief Sends an RDM get DMX start address request and reads the response, if
 * any.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which contains header information of the
 * request. Upon receiving a response, header information from the response is
 * copied into this pointer.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @param[out] dmx_start_address A pointer into which the DMX start address
 * parameter from the responder is stored.
 * @return The number of bytes received in response to the request.
 */
bool rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_header_t *header,
                               rdm_ack_t *ack, uint16_t *dmx_start_address);

/**
 * @brief Sends an RDM set DMX start address request and reads the response, if
 * any.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which contains header information of the
 * request. Upon receiving a response, header information from the response is
 * copied into this pointer.
 * @param[out] ack A pointer into which to store the RDM ACK summary.
 * @param dmx_start_address The value to which to set the DMX start address
 * parameter of the target device(s). Must be between 1 and 512 inclusive.
 * @return The number of bytes received in response to the request.
 */
bool rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_header_t *header,
                               uint16_t dmx_start_address, rdm_ack_t *ack);

#ifdef __cplusplus
}
#endif
