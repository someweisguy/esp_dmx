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

/**
 * @brief Returns true if the specified UID is a broadcast address.
 * 
 * @param uid The UID to compare.
 * @return true if the UID is a broadcast address.
 * @return false if the UID is not a broadcast address.
 */
inline bool rdm_uid_is_broadcast(rdm_uid_t uid) {
  return (uint32_t)uid == 0xffffffff;
}

/**
 * @brief Returns true if the specified UID is a broadcast address.
 * 
 * @param uid The UID to compare.  An array of 6 uint8_t's
 * @return true if the UID is a broadcast address.
 * @return false if the UID is not a broadcast address.
 */
// inline 
bool rdm_uid_array_is_broadcast(uint8_t *uid);

/**
 * @brief Returns true if the specified UID is addressed to the desired UID.
 * 
 * @param uid The UID to check against an addressee.
 * @param addressee The addressee UID.
 * @return true if the addressee UID is targeted by the specified UID.
 * @return false if the addressee is not targeted by the specified UID.
 */
inline bool rdm_uid_is_addressed_to(rdm_uid_t uid, rdm_uid_t addressee) {
  uid &= 0xffffffffffff;
  addressee &= 0xffffffffffff;
  return addressee == uid ||
         ((uid >> 32 == 0xffff || uid >> 32 == addressee >> 32) &&
          (uint32_t)uid == 0xffffffff);
}

/**
 * @brief Returns true if the specified UID is addressed to the desired UID.
 * 
 * @param uid The UID to check against an addressee.  An array of 6 uint8_t's
 * @param addressee The addressee UID.  An array of 6 uint8_t's
 * @return true if the addressee UID is targeted by the specified UID.
 * @return false if the addressee is not targeted by the specified UID.
 */
// inline 
bool rdm_uid_array_is_addressed_to(uint8_t *uid, uint8_t *addressee);

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
 * @param dmx_num The DMX port number.
 * @return The UID of the DMX port.
 */
rdm_uid_t rdm_get_uid(dmx_port_t dmx_num);

/**
 * @brief Returns the 48 bit unique ID of this device.
 *
 * @param dmx_num The DMX port number.
 * @param 
 * @return The UID of the DMX port.
 */
rdm_uid_t rdm_get_uid_for_mfr(dmx_port_t dmx_num, uint16_t mfrID);

/**
 * @brief Set the device UID to a custom value. Setting the UID to 0 will reset
 * the UID to its default value.
 *
 * @param dmx_num The DMX port number.
 * @param uid The custom value to which to set the device UID. Must be less than
 * or equal to RDM_MAX_UID.
 */
void rdm_set_uid(dmx_port_t dmx_num, rdm_uid_t uid);

/**
 * @brief Returns true if RDM discovery responses are be muted on this device.
 *
 * @param dmx_num The DMX port number.
 * @return true if RDM discovery is muted.
 * @return false if RDM discovery is not muted.
 */
bool rdm_is_muted(dmx_port_t dmx_num);

/**
 * @brief Returns true if RDM discovery responses are be muted on this device.
 *
 * @param dmx_num The DMX port number.
 * @param mute The boolean value to set the discovery_is_muted flag to.
 * @return true if RDM discovery was muted.
 * @return false if RDM discovery was not muted.
 */
bool set_rdm_muted(dmx_port_t dmx_num, bool mute);

/**
 * @brief Sends a "normal" RDM response on the desired DMX port.
 *
 * @param dmx_num The DMX port number.
 * @param pd_len The length of the Message Data Block.
 * @param uid The UID to encode into the packet.
 * @param packet The response packet sent out.
 * @return The number of bytes sent.
 */
size_t rdm_send_response(dmx_port_t dmx_num, rdm_data_t *data, const void *payload, size_t pd_len,
                         rdm_response_type_t response, uint8_t *packet);

/**
 * @brief Sends an RDM discovery response on the desired DMX port.
 *
 * @param dmx_num The DMX port number.
 * @param preamble_len The length of the packet preamble (max: 7).
 * @param uid The UID to encode into the packet.
 * @param packet The response packet sent out.
 * @return The number of bytes sent.
 */
size_t rdm_send_disc_response(dmx_port_t dmx_num, size_t preamble_len,
                              rdm_uid_t uid, uint8_t *packet);

/**
 * @brief Sends an RDM discovery request and reads the response, if any.
 *
 * @param dmx_num The DMX port number.
 * @param[in] params A pointer to the discovery UID bounds to send.
 * @param[out] response A pointer to into which to store RDM response summary.
 * @return The received UID or 0 if no response received. This function returns
 * a UID value even if a data collision occurred.
 */
rdm_uid_t rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                      rdm_disc_unique_branch_t *params,
                                      rdm_response_t *response);

/**
 * @brief Sends an RDM discovery mute request and reads the response, if any.
 *
 * @param dmx_num The DMX port number.
 * @param uid The UID to which to send the request.
 * @param mute True to send a mute request, false to send an un-mute request.
 * @param[out] response A pointer into which to store the RDM response summary.
 * @param[out] params A pointer to the discovery mute params to receive.
 * @return true if a response indicating request success was received.
 * @return false if no response was received or response was invalid.
 */
bool rdm_send_disc_mute(dmx_port_t dmx_num, rdm_uid_t uid, bool mute,
                        rdm_response_t *response, rdm_disc_mute_t *params);

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
size_t rdm_get_supported_parameters(dmx_port_t dmx_num, rdm_uid_t uid,
                                    rdm_sub_device_t sub_device,
                                    rdm_response_t *response, rdm_pid_t *pids,
                                    size_t size);

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
size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_uid_t uid,
                           rdm_sub_device_t sub_device,
                           rdm_response_t *response,
                           rdm_device_info_t *device_info);

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
size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_uid_t uid,
                                      rdm_sub_device_t sub_device,
                                      rdm_response_t *response, char *label,
                                      size_t size);

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
size_t rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                                 rdm_sub_device_t sub_device,
                                 rdm_response_t *response, int *start_address);

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
bool rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                               rdm_sub_device_t sub_device,
                               rdm_response_t *response, int start_address);

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
size_t rdm_get_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                               rdm_sub_device_t sub_device,
                               rdm_response_t *response, bool *identify);

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
bool rdm_set_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                             rdm_sub_device_t sub_device,
                             rdm_response_t *response, bool identify);

#ifdef __cplusplus
}
#endif
