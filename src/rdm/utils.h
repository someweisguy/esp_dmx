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

#define uid_is_equal(a, b) (a.man_id == b.man_id && a.dev_id == b.dev_id)

#define uid_is_gt(a, b) (a.man_id > b.man_id || (a.man_id == b.man_id && a.dev_id > b.dev_id))

#define uid_is_lt(a, b) (a.man_id < b.man_id || (a.man_id == b.man_id && a.dev_id < b.dev_id))

#define uid_is_broadcast(uid) (uid.dev_id == 0xffffffff)

#define uid_is_valid(uid) (uid.man_id <= 0xffff && uid.dev_id <= 0xfffffffe)

/**
 * @brief Helper function that takes an RDM UID from a most-significant-byte
 * first buffer and copies it to least-significant-byte first endianness, which
 * is what ESP32 uses.
 *
 * @note This function is designed to be the quickest way to swap endianness of
 * a 48-bit number on the Xtensa compiler which is important because it will be
 * used in an interrupt handler. It must be inlined in order to prevent cache
 * misses in IRAM interrupts.
 *
 * @param buf A pointer to an RDM buffer.
 * @return The properly formatted RDM UID.
 */
// static inline rdm_uid_t bswap48(const void *buf) {
//   rdm_uid_t val;
//   ((uint8_t *)&val)[0] = ((uint8_t *)buf)[5];
//   ((uint8_t *)&val)[1] = ((uint8_t *)buf)[4];
//   ((uint8_t *)&val)[2] = ((uint8_t *)buf)[3];
//   ((uint8_t *)&val)[3] = ((uint8_t *)buf)[2];
//   ((uint8_t *)&val)[4] = ((uint8_t *)buf)[1];
//   ((uint8_t *)&val)[5] = ((uint8_t *)buf)[0];
//   *(uint16_t *)&((&val)[3]) = 0;
//   return val;
// }

/**
 * @brief Returns true if the specified UID is a broadcast address. This
 * function only checks the device ID of the UID. It will return true when the 
 * device ID is broadcast including when the UID is invalid.
 * 
 * @param uid The UID to compare.
 * @return true if the UID is a broadcast address.
 * @return false if the UID is not a broadcast address.
 */
//bool uid_is_broadcast(rdm_uid_t uid);

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
 * @brief Helper function that converts an RDM UID stored as a 64-bit integer
 * and copies it into a 48-bit buffer. It also converts endianness because the
 * ESP32 stores values in least-significant-byte first endianness and RDM
 * requires most-significant-byte first.
 * 
 * // TODO
 *
 * @param[out] dest A pointer to the destination buffer.
 * @param[in] uid The 64-bit representation of the UID.
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