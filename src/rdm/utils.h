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

// TODO: docs
#define uid_is_equal(a, b) (a.man_id == b.man_id && a.dev_id == b.dev_id)

// TODO: docs
#define uid_is_gt(a, b) \
  (a.man_id > b.man_id || (a.man_id == b.man_id && a.dev_id > b.dev_id))

// TODO: docs
#define uid_is_lt(a, b) \
  (a.man_id < b.man_id || (a.man_id == b.man_id && a.dev_id < b.dev_id))

/**
 * @brief Returns true if the specified UID is a broadcast address. This
 * function only checks the device ID of the UID. It will return true when the 
 * device ID is broadcast including when the UID is invalid.
 * 
 * @param uid The UID to compare.
 * @return true if the UID is a broadcast address.
 * @return false if the UID is not a broadcast address.
 */
#define uid_is_broadcast(uid) (uid.dev_id == 0xffffffff)

// TODO: docs
#define uid_is_valid(uid) (uid.man_id <= 0xffff && uid.dev_id <= 0xfffffffe)

/**
 * @brief Returns true if the specified UID is addressed to the desired UID.
 * 
 * @param compare_uid The UID to check against a recipient UID.
 * @param recipient_uid The recipient UID.
 * @return true if the recipient UID is targeted by the comparison UID.
 * @return false if the recipient UID is not targeted by the comparison UID.
 */
bool uid_is_recipient(rdm_uid_t compare_uid, rdm_uid_t recipient_uid);

/**
 * @brief Copies RDM UID from a source buffer directly into a destination
 * buffer. Either the source or the destination should point to an rdm_uid_t
 * type.
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