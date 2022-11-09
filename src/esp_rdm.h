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
 * @brief The recommended method for representing the UID in text by separating
 * the manufacturer ID and the device ID. For use with printf-like functions.
 */
#define UIDSTR "%04x:%08x"

/**
 * @brief Used to generate arguments for the UIDSTR macro for representing the
 * UID in text by separating the manufacturer ID and device ID. For use with
 * printf-like functions.
 */
#define UID2STR(uid) ((uint16_t)(uid >> 32)), ((uint32_t)(uid))

// TODO: docs
typedef void (rdm_discovery_cb_t)(dmx_port_t, rdm_uid_t, size_t, void *);

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
FORCE_INLINE_ATTR rdm_uid_t buf_to_uid(const void *buf) {
  rdm_uid_t val;
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
FORCE_INLINE_ATTR void *uid_to_buf(void *buf, rdm_uid_t uid) {
  ((uint8_t *)buf)[0] = ((uint8_t *)&uid)[5];
  ((uint8_t *)buf)[1] = ((uint8_t *)&uid)[4];
  ((uint8_t *)buf)[2] = ((uint8_t *)&uid)[3];
  ((uint8_t *)buf)[3] = ((uint8_t *)&uid)[2];
  ((uint8_t *)buf)[4] = ((uint8_t *)&uid)[1];
  ((uint8_t *)buf)[5] = ((uint8_t *)&uid)[0];
  return buf;
}

/**
 * @brief Returns the 48 bit unique ID of this device.
 *
 * @return The UID of this device.
 */
rdm_uid_t rdm_get_uid(dmx_port_t dmx_num);

/**
 * @brief Set the device UID to a custom value. Setting the UID to 0 will reset
 * the UID to its default value.
 *
 * @param uid The custom value to which to set the device UID.
 */
void rdm_set_uid(dmx_port_t dmx_num, rdm_uid_t uid);

/**
 * @brief Returns true if RDM discovery responses are be muted on this device.
 * 
 * @return true if RDM discovery is muted.
 * @return false if RDM discovery is not muted.
 */
bool rdm_is_muted();

// TODO: docs
size_t rdm_send_disc_response(dmx_port_t dmx_num, rdm_uid_t uid);

// TODO: docs
size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                   rdm_disc_unique_branch_t *params,
                                   rdm_response_t *response, rdm_uid_t *uid);

// TODO: docs
size_t rdm_send_disc_mute(dmx_port_t dmx_num, rdm_uid_t uid, bool mute,
                          rdm_response_t *response,
                          rdm_disc_mute_t *mute_params);

// TODO: docs
size_t rdm_discover_with_callback(dmx_port_t dmx_num, rdm_discovery_cb_t cb,
                                  void *context);

// TODO: docs
size_t rdm_discover_devices(dmx_port_t dmx_num, rdm_uid_t *uids,
                            const size_t size);

// TODO: implement, docs
size_t rdm_get_supported_parameters(dmx_port_t dmx_num, rdm_uid_t uid,
                                    uint16_t sub_device,
                                    rdm_response_t *response, rdm_pid_t *pids,
                                    size_t size);

// TODO: docs
size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_uid_t uid,
                           uint16_t sub_device, rdm_response_t *response,
                           rdm_device_info_t *device_info);

// TODO: docs
size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_uid_t uid,
                                      uint16_t sub_device,
                                      rdm_response_t *response, char *label,
                                      size_t size);

// TODO: implement, docs
size_t rdm_get_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                               uint16_t sub_device, rdm_response_t *response,
                               bool *identify_state);

// TODO: implement, docs
size_t rdm_set_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                               uint16_t sub_device, rdm_response_t *response,
                               const bool identify_state);

// TODO: implement, docs
size_t rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                                 uint16_t sub_device, rdm_response_t *response,
                                 int *start_address);

// TODO: implement, docs
size_t rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                                 uint16_t sub_device, rdm_response_t *response,
                                 const int start_address);

#ifdef __cplusplus
}
#endif
