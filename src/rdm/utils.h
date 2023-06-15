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
 * @brief Returns true if the UIDs are equal to each other. Is equivalent to
 * a == b.
 * 
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if the UIDs are equal.
 * @return false if the UIDs are not equal.
 */
bool uid_is_eq(const rdm_uid_t *a, const rdm_uid_t *b);

/**
 * @brief Returns true if the first UID is less than the second UID. Is
 * equivalent to a < b.
 *
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if a is less than b.
 * @return false if a is not less than b.
 */
bool uid_is_lt(const rdm_uid_t *a, const rdm_uid_t *b);

/**
 * @brief Returns true if the first UID is greater than the second UID. Is
 * equivalent to a > b.
 * 
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if a is greater than b.
 * @return false if a is not greater than b.
 */
bool uid_is_gt(const rdm_uid_t *a, const rdm_uid_t *b);

/**
 * @brief Returns true if the first UID is less than or equal to the second
 * UID. Is equivalent to a <= b.
 * 
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if a is less than or equal to b.
 * @return false if a is not less than or equal to b.
 */
bool uid_is_le(const rdm_uid_t *a, const rdm_uid_t *b);

/**
 * @brief Returns true if the first UID is greater than or equal to the second
 * UID. Is equivalent to a >= b.
 * 
 * @param a A pointer to the first operand.
 * @param b A pointer to the second operand.
 * @return true if a is greater than or equal to b.
 * @return false if a is not greater than or equal to b.
 */
bool uid_is_ge(const rdm_uid_t *a, const rdm_uid_t *b);

/**
 * @brief Returns true if the specified UID is a broadcast address.
 * 
 * @param uid A pointer to the unary UID operand.
 * @return true if the UID is a broadcast address.
 * @return false if the UID is not a broadcast address.
 */
bool uid_is_broadcast(const rdm_uid_t *uid);

/**
 * @brief Returns true if the specified UID is null.
 * 
 * @param uid A pointer to the unary UID operand.
 * @return true if the UID is null.
 * @return false if the UID is not null.
 */
bool uid_is_null(const rdm_uid_t *uid);

/**
 * @brief Returns true if the first UID is targeted by the second UID. A common
 * usage of this function is `uid_is_target(&my_uid, &destination_uid)`.
 *
 * @param uid A pointer to a UID which to check is targeted.
 * @param alias A pointer to a UID which may alias the first UID.
 * @return true if the UID is targeted by the alias UID.
 * @return false if the UID is not targeted by the alias UID.
 */
bool uid_is_target(const rdm_uid_t *uid, const rdm_uid_t *alias);

// TODO: docs
size_t uid_encode(void *destination, const rdm_uid_t *uid, size_t preamble_len);

// TODO: docs
int uid_decode(rdm_uid_t *destination, const void *source, size_t size);

// TODO: docs
size_t pdcpy(void *destination, size_t dest_size, const char *format,
             const void *source, size_t src_size, const bool encode_nulls);

/**
 * @brief Get the preamble length of a DISC_UNIQUE_BRANCH response. A
 * DISC_UNIQUE_BRANCH response contains a preamble of between 0 and 7
 * (inclusive) preamble bytes and one delimiter byte.
 *
 * @param data A pointer to a buffer containing a DISC_UNIQUE_BRANCH response.
 * @return The number of preamble bytes in the buffer.
 */
size_t get_preamble_len(const void *data);

#ifdef __cplusplus
}
#endif