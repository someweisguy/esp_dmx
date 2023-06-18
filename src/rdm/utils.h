/**
 * @file utils.h
 * @author Mitch Weisbrod
 * @brief This file contains some functions that can be helpful when performing
 * advanced operations on RDM packets. It is used throughout this library, but
 * is not included by default in esp_dmx.h. It may be manually included by the
 * user when it is needed.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief A function type for RDM responder callbacks. This is the type of
 * function that is called when responding to RDM requests.
 */
typedef rdm_response_type_t (*rdm_response_cb_t)(dmx_port_t dmx_num,
                                                 const rdm_header_t *header,
                                                 void *pd, uint8_t *pdl_out,
                                                 void *param, size_t param_size,
                                                 void *context);

/**
 * @brief Copies RDM UID from a source buffer directly into a destination
 * buffer. This function swaps endianness, allowing for UIDs to be copied from
 * RDM packet buffers into ESP32 memory. Either the source or the destination
 * should point to an rdm_uid_t type.
 *
 * To avoid overflows, the size of the arrays pointed to by both the destination
 * and source parameters shall be at least six bytes and should not overlap.
 *
 * @param[out] destination A pointer to the destination buffer.
 * @param[in] source A pointer to the source buffer of the UID.
 * @return A pointer to the destination buffer.
 */
void *uidcpy(void *restrict destination, const void *restrict source);

/**
 * @brief Copies RDM UID from a source buffer into a destination buffer. Copying
 * takes place as if an intermediate buffer were used, allowing the destination
 * and source to overlap. This function swaps endianness, allowing for UIDs to
 * be copied from RDM packet buffers into ESP32 memory. Either the source or the
 * destination should point to an rdm_uid_t type.
 *
 * To avoid overflows, the size of the arrays pointed to by both the destination
 * and source parameters shall be at least six bytes.
 *
 * @param[out] destination A pointer to the destination buffer.
 * @param[in] source A pointer to the source buffer of the UID.
 * @return A pointer to the destination buffer.
 */
void *uidmove(void *destination, const void *source);

/**
 * @brief Returns the 48-bit unique ID of the desired DMX port.
 *
 * @param dmx_num The DMX port number.
 * @param[out] uid A pointer to a rdm_uid_t type to store the received UID.
 */
void uid_get(dmx_port_t dmx_num, rdm_uid_t *uid);

/**
 * @brief Returns true if the UIDs are equal to each other. Is equivalent to
 * a == b.
 *
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if the UIDs are equal.
 * @return false if the UIDs are not equal.
 */
static inline bool uid_is_eq(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id == b->man_id && a->dev_id == b->dev_id;
}

/**
 * @brief Returns true if the first UID is less than the second UID. Is
 * equivalent to a < b.
 *
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if a is less than b.
 * @return false if a is not less than b.
 */
static inline bool uid_is_lt(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id < b->man_id ||
         (a->man_id == b->man_id && a->dev_id < b->dev_id);
}

/**
 * @brief Returns true if the first UID is greater than the second UID. Is
 * equivalent to a > b.
 *
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if a is greater than b.
 * @return false if a is not greater than b.
 */
static inline bool uid_is_gt(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id > b->man_id ||
         (a->man_id == b->man_id && a->dev_id > b->dev_id);
}

/**
 * @brief Returns true if the first UID is less than or equal to the second
 * UID. Is equivalent to a <= b.
 *
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if a is less than or equal to b.
 * @return false if a is not less than or equal to b.
 */
static inline bool uid_is_le(const rdm_uid_t *a, const rdm_uid_t *b) {
  return !uid_is_gt(a, b);
}

/**
 * @brief Returns true if the first UID is greater than or equal to the second
 * UID. Is equivalent to a >= b.
 *
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if a is greater than or equal to b.
 * @return false if a is not greater than or equal to b.
 */
static inline bool uid_is_ge(const rdm_uid_t *a, const rdm_uid_t *b) {
  return !uid_is_lt(a, b);
}

/**
 * @brief Returns true if the specified UID is a broadcast address.
 *
 * @param uid A pointer to the unary UID operand.
 * @return true if the UID is a broadcast address.
 * @return false if the UID is not a broadcast address.
 */
static inline bool uid_is_broadcast(const rdm_uid_t *uid) {
  return uid->dev_id == 0xffffffff;
}

/**
 * @brief Returns true if the specified UID is null.
 *
 * @param uid A pointer to the unary UID operand.
 * @return true if the UID is null.
 * @return false if the UID is not null.
 */
static inline bool uid_is_null(const rdm_uid_t *uid) {
  return uid->man_id == 0 && uid->dev_id == 0;
}

/**
 * @brief Returns true if the first UID is targeted by the second UID. A
 * common usage of this function is `uid_is_target(&my_uid,
 * &destination_uid)`.
 *
 * @param uid A pointer to a UID which to check is targeted.
 * @param alias A pointer to a UID which may alias the first UID.
 * @return true if the UID is targeted by the alias UID.
 * @return false if the UID is not targeted by the alias UID.
 */
static inline bool uid_is_target(const rdm_uid_t *uid, const rdm_uid_t *alias) {
  return ((alias->man_id == 0xffff || alias->man_id == uid->man_id) &&
          alias->dev_id == 0xffffffff) ||
         uid_is_eq(uid, alias);
}

// TODO: docs
size_t pd_emplace(void *destination, const char *format, const void *source,
                  size_t num, bool encode_nulls);

size_t pd_emplace_word(void *destination, uint16_t word);

/**
 * @brief Reads and formats a received RDM message from the DMX driver buffer.
 *
 * @param dmx_num The DMX port number.
 * @param[out] header A pointer which stores RDM header information.
 * @param[out] mdb A pointer which stores RDM message data block information.
 * This is typically further decoded using functions defined in `rdm/mdb.h`.
 * @return The number of bytes in the RDM packet or zero if the packet is
 * invalid.
 * // TODO
 */
size_t rdm_read(dmx_port_t dmx_num, rdm_header_t *header, uint8_t pdl,
                void *pd);

/**
 * @brief Writes and formats an RDM message into the DMX driver buffer.
 *
 * @param dmx_num The DMX port number.
 * @param[in] header A pointer which stores RDM header information.
 * @param[in] mdb A pointer which stores RDM message data block information.
 * This is typically already encoded using functions defined in `rdm/mdb.h`.
 * @return The number of bytes written to the DMX driver buffer or zero on
 * error.
 * // TODO
 */
size_t rdm_write(dmx_port_t dmx_num, rdm_header_t *header, uint8_t pdl,
                 const void *pd);

/**
 * @brief Sends an RDM controller request and processes the response. It is
 * important for users to check the value of the ack argument to verify if a
 * valid RDM response was received.
 * - ack.err will evaluate to true if an error occurred during the sending or
 * receiving of raw DMX data. RDM data will not be processed if an error
 * occurred. If a response was expected but none was received, ack.err will
 * evaluate to ESP_ERR_TIMEOUT. If no response was expected, ack.err will be
 * set to ESP_OK.
 * - ack.type will evaluate to RDM_RESPONSE_TYPE_INVALID if an invalid
 * response is received but does not necessarily indicate a DMX error
 * occurred. If no response is received ack.type will be set to
 * RDM_RESPONSE_TYPE_NONE whether or not a response was expected. Otherwise,
 * ack.type will be set to the ack type received in the RDM response. The
 * final parameter of ack is a union of num, nack_reason, and timer. If the
 * received response type is RDM_RESPONSE_TYPE_ACK or
 * RDM_RESPONSE_TYPE_ACK_OVERFLOW, ack.num should be read. If the response
 * type is RDM_RESPONSE_TYPE_NACK_REASON, nack_reason should be read. If the
 * response type is RDM_RESPONSE_TYPE_TIMER, timer should be read.
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
// TODO: docs
bool rdm_request(dmx_port_t dmx_num, rdm_header_t *header, const uint8_t pdl_in,
                 const void *pd_in, uint8_t pdl_out, void *pd_out,
                 rdm_ack_t *ack);

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
// TODO: docs
bool rdm_register_response(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           rdm_pid_description_t *desc,
                           rdm_response_cb_t callback, void *param,
                           size_t param_len, void *context);

#ifdef __cplusplus
}
#endif