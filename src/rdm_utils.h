#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dmx_types.h"
#include "rdm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#if ESP_IDF_VERSION_MAJOR >= 5
/**
 * @brief The recommended method for representing the UID in text by separating
 * the manufacturer ID and the device ID. For use with printf-like functions.
 */
#define UIDSTR "%04x:%08lx"
#else
// TODO: can we remove this one?
/**
 * @brief The recommended method for representing the UID in text by separating
 * the manufacturer ID and the device ID. For use with printf-like functions.
 */
#define UIDSTR "%04x:%08x"
#endif

/**
 * @brief Used to generate arguments for the UIDSTR macro for representing the
 * UID in text by separating the manufacturer ID and device ID. For use with
 * printf-like functions.
 */
#define UID2STR(uid) ((uint16_t)(uid >> 32)), ((uint32_t)(uid))

/**
 * @brief Returns true if the specified UID is a broadcast address.
 * 
 * @param uid The UID to compare.
 * @return true if the UID is a broadcast address.
 * @return false if the UID is not a broadcast address.
 */
inline bool uid_is_broadcast(rdm_uid_t uid) {
  return (uint32_t)uid == 0xffffffff;
}

/**
 * @brief Returns true if the specified UID is addressed to the desired UID.
 * 
 * @param uid The UID to check against an addressee.
 * @param addressee The addressee UID.
 * @return true if the addressee UID is targeted by the specified UID.
 * @return false if the addressee is not targeted by the specified UID.
 */
// TODO: rename arguments for clarity
inline bool uid_is_recipient(rdm_uid_t uid, rdm_uid_t addressee) {
  uid &= 0xffffffffffff;
  addressee &= 0xffffffffffff;
  return addressee == uid ||
         ((uid >> 32 == 0xffff || uid >> 32 == addressee >> 32) &&
          (uint32_t)uid == 0xffffffff);
}

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
FORCE_INLINE_ATTR rdm_uid_t get_uid(const void *buf) {
  rdm_uid_t val;
  // TODO: *(uint16_t *)&((&val)[3]) = 0;
  ((uint8_t *)&val)[7] = 0;
  ((uint8_t *)&val)[6] = 0;
  ((uint8_t *)&val)[5] = ((uint8_t *)buf)[0];
  ((uint8_t *)&val)[4] = ((uint8_t *)buf)[1];
  ((uint8_t *)&val)[3] = ((uint8_t *)buf)[2];
  ((uint8_t *)&val)[2] = ((uint8_t *)buf)[3];
  ((uint8_t *)&val)[1] = ((uint8_t *)buf)[4];
  ((uint8_t *)&val)[0] = ((uint8_t *)buf)[5];
  return val;
}

/**
 * @brief Helper function that converts an RDM UID stored as a 64-bit integer
 * and copies it into a 48-bit buffer. It also converts endianness to compensate
 * for the fact that the ESP32 stores values in least-significant-byte first
 * endianness and RDM requires most-significant-byte first.
 *
 * @param buf A pointer to the destination buffer.
 * @param uid The 64-bit representation of the UID.
 * @return void* A pointer to the destination buffer.
 */
// TODO: doesn't need to be inlined
FORCE_INLINE_ATTR void *uidcpy(void *buf, rdm_uid_t uid) {
  ((uint8_t *)buf)[0] = ((uint8_t *)&uid)[5];
  ((uint8_t *)buf)[1] = ((uint8_t *)&uid)[4];
  ((uint8_t *)buf)[2] = ((uint8_t *)&uid)[3];
  ((uint8_t *)buf)[3] = ((uint8_t *)&uid)[2];
  ((uint8_t *)buf)[4] = ((uint8_t *)&uid)[1];
  ((uint8_t *)buf)[5] = ((uint8_t *)&uid)[0];
  return buf;
}

// TODO: docs
size_t get_preamble_len(const void *data);

#ifdef __cplusplus
}
#endif