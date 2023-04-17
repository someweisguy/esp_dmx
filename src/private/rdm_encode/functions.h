#pragma once

#include <stdint.h>
#include <string.h>

#include "endian.h"
#include "esp_dmx.h"
#include "private/rdm_encode/types.h"
#include "rdm_types.h"

#define RDM_PACKET_MAX_PARAMS(type) (255 / sizeof(type))

#define RDM_PDL_MAX_PARAMS(pdl, type) (pdl / sizeof(type))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Checks if a data buffer appears to contain valid RDM data. This
 * function considers a packet valid if it is at least the right size and if the
 * packet begins with the correct start code. This function does not verify
 * packet format nor validate checksums.
 *
 * @param data The buffer which stores a potentially valid RDM packet.
 * @param size The size of the buffer.
 * @return true if the data is a valid RDM packet.
 * @return false if the packet is invalid.
 */
bool rdm_is_valid(const void *data, size_t size);

/**
 * @brief Decodes an RDM packet from the desired data buffer. The function 
 * `rdm_is_valid()` should be called on the data buffer first to ensure data
 * is safely decoded.
 * 
 * @param[in] data The buffer which stores a valid, encoded RDM packet.
 * @param[out] header A pointer to an RDM header in which data will be copied.
 * @param[out] mdb A pointer to an RDM message data block in which data will be
 * copied.
 * @return true if the data is a valid RDM packet.
 * @return false if the packet is invalid.
 */
bool rdm_decode_packet(const void *data, rdm_header_t *header, rdm_mdb_t *mdb);

/**
 * @brief Encodes an RDM packet into the desired data buffer.
 * 
 * @param[out] data The buffer in which to encode the packet.
 * @param[in] header A pointer to an RDM header used to encode data.
 * @param[in] mdb A pointer to an RDM message data block to encode data.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_packet(void *data, rdm_header_t *header, rdm_mdb_t *mdb);

/**
 * @brief Encodes RDM discovery mute parameters into the desired buffer.
 *
 * @param[out] data The buffer in which to encode the data.
 * @param[in] param A pointer to a discovery mute parameter to encode.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_mute(void *data, const rdm_disc_mute_t *param);

/**
 * @brief Decodes RDM discovery mute parameters.
 *
 * @param[in] data The buffer in which the data to decode is stored.
 * @param[out] param A pointer to a discovery mute parameter to store decoded
 * data.
 * @param pdl The length of the parameter data.
 * @return The number of parameters decoded (always 1).
 */
int rdm_decode_mute(const void *data, rdm_disc_mute_t *param, int size,
                    size_t pdl);

/**
 * @brief Encodes RDM UIDs into the desired buffer.
 *
 * @param[out] data The buffer in which to encode the data.
 * @param[in] uids A pointer to an array of UIDs to encode.
 * @param size The size of the array of UIDs.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_uids(void *data, const void *uids, int size);

/**
 * @brief Decodes RDM UIDs into the desired array.
 *
 * @param[in] data The buffer in which the data to decode is stored.
 * @param[out] uids A pointer to an array of UIDs to store decoded data.
 * @param size The size of the array of UIDs.
 * @return The number of UIDs decoded.
 */
// int rdm_decode_uids(const void *data, void *uids, int size);

/**
 * @brief Encode an array of 16-bit numbers into the desired array.
 *
 * @param[out] pd The buffer in which to encode the data.
 * @param[in] data A pointer to an array of values to encode.
 * @param size The size of the array of values.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_16bit(void *pd, const void *data, int size);

/**
 * @brief Decode an array of 16-bit numbers into the desired array.
 *
 * @param[in] pd A pointer to the parameter data to decode.
 * @param[out] data A pointer to an array in which to store the decoded data.
 * @param size The size of the array to store decoded data.
 * @return The number of of values available to decode.
 */
int rdm_decode_16bit(const void *pd, void *data, int size);

/**
 * @brief Encode an array of 8-bit numbers into the desired array.
 *
 * @param[out] pd The buffer in which to encode the data.
 * @param[in] data A pointer to an array of values to encode.
 * @param size The size of the array of values.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_8bit(void *pd, const void *data, int size);

/**
 * @brief Decode an array of 8-bit numbers into the desired array.
 *
 * @param[in] pd A buffer in which the data to decode is stored.
 * @param[out] data A pointer to an array in which to store the decoded data.
 * @param size The size of the array to store decoded data.
 * @return The number of of values available to be decoded.
 */
int rdm_decode_8bit(const void *pd, void *data, int size);

/**
 * @brief Encode a string into the desired array.
 * 
 * @param[out] pd A buffer into which to encode the data.
 * @param[in] data A pointer to an array of values to encode.
 * @param size The size of the string.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_string(void *pd, const void *data, int size);

/**
 * @brief Decode a string from a buffer.
 *
 * @param pd A buffer in which the to decode is stored.
 * @param data A pointer into which to store the decoded data.
 * @param size The size of the buffer to store the decoded data.
 * @return The number of characters that was decoded.
 */
int rdm_decode_string(const void *pd, void *data, int size);

/**
 * @brief Encodes RDM device info into the desired buffer.
 *
 * @param[out] pd The buffer in which to encode the data.
 * @param[in] data A pointer to a discovery mute parameter to encode.
 * @param size The size of the array to store decoded data.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_device_info(void *pd, const void *data, int size);

/**
 * @brief Decodes RDM device info.
 *
 * @param[in] pd The buffer in which the data to decode is stored.
 * @param[out] data A pointer to a device info parameter to store decoded data.
 * @param size The size of the array to store decoded data.
 * @return The number of parameters decoded (always 1).
 */
int rdm_decode_device_info(const void *pd, void *data, int size);

// TODO: docs
size_t rdm_get_message_len(const void *data);

// TODO: docs
size_t rdm_get_preamble_len(const void *data);

// TODO: docs
bool rdm_checksum_is_valid(const void *data);

// TODO: docs
bool rdm_is_request(const void *data);

// TODO: docs
size_t rdm_encode_nack_reason(rdm_mdb_t *mdb, rdm_nr_t nack_reason);

// TODO: docs
int rdm_decode_uids(const rdm_mdb_t *mdb, void *data, int num);

#ifdef __cplusplus
}
#endif
