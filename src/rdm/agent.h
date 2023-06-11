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
 * @brief Returns the 48-bit unique ID of this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] uid A pointer to a rdm_uid_t type to store the received UID.
 * @return The UID of the DMX port.
 */
void rdm_driver_get_uid(dmx_port_t dmx_num, rdm_uid_t *uid);

/**
 * @brief Set the device UID to a custom value. Setting the UID to 0 will reset
 * the UID to its default value.
 *
 * @param dmx_num The DMX port number.
 * @param uid The custom value to which to set the device UID. Must be less than
 * or equal to RDM_MAX_UID.
 */
void rdm_driver_set_uid(dmx_port_t dmx_num, rdm_uid_t uid);

/**
 * @brief Returns true if RDM discovery responses are be muted on this device.
 *
 * @param dmx_num The DMX port number.
 * @return true if RDM discovery is muted.
 * @return false if RDM discovery is not muted.
 */
bool rdm_driver_is_muted(dmx_port_t dmx_num);

/**
 * @brief Gets the device info for this device. To get the device info of a
 * responder device, use `rdm_get_device_info()`.
 *
 * @param dmx_num The DMX port number.
 * @param[out] device_info A pointer to a struct which stores a copy of this
 * device's device info.
 * @return true if the device info was copied.
 * @return false if the the device info was not copied.
 */
bool rdm_driver_get_device_info(dmx_port_t dmx_num,
                                rdm_device_info_t *device_info);

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

/**
 * @brief Registers a callback which is called when a request is received for
 * this device for a specified PID. Callbacks may be overwritten, but they may
 * not be deleted.
 *
 * @param dmx_num The DMX port number.
 * @param pid The PID to which the callback function should be attached.
 * @param callback A pointer to a callback function which will be called when
 * receiving a request for the specified PID.
 * @param[inout] context A pointer to a user-specified context for use within
 * the callback function.
 * @return true when the callback has been successfully registered.
 * @return false on failure.
 * // TODO update docs
 */
bool rdm_register_callback(dmx_port_t dmx_num,
                           const rdm_pid_description_t *desc,
                           const rdm_encode_decode_t *get,
                           const rdm_encode_decode_t *set,
                           rdm_response_cb_t callback, void *context);

/**
 * @brief Reads and formats a received RDM message from the DMX driver buffer.
 *
 * @param dmx_num The DMX port number.
 * @param[out] header A pointer which stores RDM header information.
 * @param[out] mdb A pointer which stores RDM message data block information.
 * This is typically further decoded using functions defined in `rdm/mdb.h`.
 * @return The number of bytes in the RDM packet or zero if the packet is
 * invalid.
 */
size_t rdm_read(dmx_port_t dmx_num, rdm_header_t *header, rdm_mdb_t *mdb);

/**
 * @brief Writes and formats an RDM message into the DMX driver buffer.
 *
 * @param dmx_num The DMX port number.
 * @param[in] header A pointer which stores RDM header information.
 * @param[in] mdb A pointer which stores RDM message data block information.
 * This is typically already encoded using functions defined in `rdm/mdb.h`.
 * @return The number of bytes written to the DMX driver buffer or zero on
 * error.
 */
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
bool rdm_register_disc_unique_branch(dmx_port_t dmx_num, void *context);

// TODO: docs
bool rdm_register_disc_mute(dmx_port_t dmx_num, void *context);

// TODO: docs
bool rdm_register_disc_un_mute(dmx_port_t dmx_num, void *context);

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
