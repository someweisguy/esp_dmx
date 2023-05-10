#pragma once

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
bool rdm_read(dmx_port_t dmx_num, rdm_header_t *header, rdm_mdb_t *mdb);

// TODO: docs
size_t rdm_write(dmx_port_t dmx_num, const rdm_header_t *header,
                 const rdm_mdb_t *mdb);

/**
 * @brief Sends an RDM controller request and processes the response. It is
 * important for users to check the value of the ack argument to verify if a
 * valid RDM response was received.
 * - ack.err will evaluate to true if an error occurred during the sending or
 * receiving of raw DMX data. RDM data will not be processed if an error
 * occurred. If a response was expected but none was received, ack.err will
 * evaluate to ESP_ERR_TIMEOUT. If no response was expected, ack.err will be set
 * to ESP_OK.
 * - ack.type will evaluate to RDM_RESPONSE_TYPE_INVALID if an invalid response
 * is received but does not necessarily indicate a DMX error occurred. If no
 * response is received ack.type will be set to RDM_RESPONSE_TYPE_NONE whether
 * or not a response was expected. Otherwise, ack.type will be set to the ack
 * type received in the RDM response.
 * The final parameter of ack is a union of num, nack_reason, and timer. If the
 * received response type is RDM_RESPONSE_TYPE_ACK or
 * RDM_RESPONSE_TYPE_ACK_OVERFLOW, ack.num should be read. If the response type
 * is RDM_RESPONSE_TYPE_NACK_REASON, nack_reason should be read. If the response
 * type is RDM_RESPONSE_TYPE_TIMER, timer should be read.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an rdm_header_t with information on where
 * to address the RDM request. Upon receiving a response, this information is
 * overwritten with information about the received RDM response.
 * @param[in] encode A pointer to an rdm_encode_t which contains information
 * about how to encode the MDB of the RDM request.
 * @param[out] decode A pointer to an rdm_decode_t which contains information
 * about how to decode the MDB of the RDM response.
 * @param[out] ack A pointer to an rdm_ack_t which contains information about
 * the received RDM response.
 * @return The size of the received RDM packet or 0 if no packet was received.
 */
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
 * @return The number of devices found. // TODO: return int, not size_t
 */
size_t rdm_discover_devices_simple(dmx_port_t dmx_num, rdm_uid_t *uids,
                                   const size_t size);

#ifdef __cplusplus
}
#endif
