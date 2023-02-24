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
 * @brief Encodes a normal response in the desired data buffer.
 *
 * @param[out] data The buffer in which to encode the response.
 * @param mdb_len The length of the response MDB.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_response(uint8_t *data, size_t mdb_len);

/**
 * @brief Encodes a DISC_UNIQUE_BRANCH response in the desired data buffer.
 *
 * @param[out] data The buffer in which to encode the response.
 * @param preamble_len The length of the response preamble (max: 7).
 * @param uid The RDM UID to encode into the response.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_disc_response(uint8_t *data, size_t preamble_len,
                                const rdm_uid_t uid);

/**
 * @brief Decodes a DISC_UNIQUE_BRANCH response.
 *
 * @param[in] data The buffer in which the data to decode is stored.
 * @param[out] uid The decoded UID in the response.
 * @return true if the data checksum was valid.
 * @return false if the data checksum was invalid.
 */
bool rdm_decode_disc_response(const uint8_t *data, rdm_uid_t *uid);

/**
 * @brief Encodes an RDM header and checksum into the desired buffer. When
 * encoding data, parameter data must be encoded before calling this function.
 * Otherwise, the encoded checksum will be invalid.
 *
 * @param[out] data The buffer in which to encode the header.
 * @param[in] header A pointer to an RDM header used to encode data.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_header(void *data, const rdm_header_t *header);

/**
 * @brief Decodes an RDM header.
 *
 * @param[in] data The buffer in which the data to decode is stored.
 * @param[out] header A pointer to an RDM header used to store decoded data.
 * @return true if the data was a valid RDM packet.
 * @return false if the data was invalid.
 */
bool rdm_decode_header(const void *data, rdm_header_t *header);

/**
 * @brief Encodes RDM UIDs into the desired buffer.
 *
 * @param[out] data The buffer in which to encode the data.
 * @param[in] uids A pointer to an array of UIDs to encode.
 * @param size The size of the array of UIDs.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_uids(void *data, const rdm_uid_t *uids, size_t size);

/**
 * @brief Decodes RDM UIDs into the desired array.
 *
 * @param[in] data The buffer in which the data to decode is stored.
 * @param[out] uids A pointer to an array of UIDs to store decoded data.
 * @param size The size of the array of UIDs.
 * @param pdl The length of the parameter data.
 * @return The number of UIDs decoded.
 */
size_t rdm_decode_uids(const void *data, rdm_uid_t *const uids, size_t size,
                       size_t pdl);

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
size_t rdm_decode_mute(const void *data, rdm_disc_mute_t *param, size_t size,
                       size_t pdl);

/**
 * @brief Encode an array of 16-bit numbers into the desired array.
 *
 * @param[out] pd The buffer in which to encode the data.
 * @param[in] data A pointer to an array of values to encode.
 * @param size The size of the array of values.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_16bit(void *pd, const void *data, size_t size);

/**
 * @brief Decode an array of 16-bit numbers into the desired array.
 *
 * @param[in] pd A pointer to the parameter data to decode.
 * @param[out] data A pointer to an array in which to store the decoded data.
 * @param size The size of the array to store decoded data.
 * @param pdl The length of the parameter data.
 * @return The number of of values available to decode.
 */
size_t rdm_decode_16bit(const void *pd, void *data, size_t size, size_t pdl);

/**
 * @brief Encode an array of 8-bit numbers into the desired array.
 *
 * @param[out] pd The buffer in which to encode the data.
 * @param[in] data A pointer to an array of values to encode.
 * @param size The size of the array of values.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_8bit(void *pd, const void *data, size_t size);

/**
 * @brief Decode an array of 8-bit numbers into the desired array.
 *
 * @param[in] pd A buffer in which the data to decode is stored.
 * @param[out] data A pointer to an array in which to store the decoded data.
 * @param size The size of the array to store decoded data.
 * @param pdl The length of the parameter data.
 * @return The number of of values available to be decoded.
 */
size_t rdm_decode_8bit(const void *pd, void *data, size_t size,
                       size_t pdl);

/**
 * @brief Encode a string into the desired array.
 * 
 * @param[out] pd A buffer into which to encode the data.
 * @param[in] data A pointer to an array of values to encode.
 * @param size The size of the string.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_string(void *pd, const void *data, size_t size);

/**
 * @brief Decode a string from a buffer.
 *
 * @param pd A buffer in which the to decode is stored.
 * @param data A pointer into which to store the decoded data.
 * @param size The size of the buffer to store the decoded data.
 * @param pdl The length of the parameter data.
 * @return The number of characters that was decoded.
 */
size_t rdm_decode_string(const void *pd, void *data, size_t size, size_t pdl);

/**
 * @brief Encodes RDM device info into the desired buffer.
 *
 * @param[out] pd The buffer in which to encode the data.
 * @param[in] data A pointer to a discovery mute parameter to encode.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_device_info(void *pd, const void *data);

/**
 * @brief Decodes RDM device info.
 *
 * @param[in] pd The buffer in which the data to decode is stored.
 * @param[out] data A pointer to a device info parameter to store decoded data.
 * @param size The size of the array to store decoded data.
 * @param pdl The length of the parameter data.
 * @return The number of parameters decoded (always 1).
 */
size_t rdm_decode_device_info(const void *pd, void *data, size_t size,
                              size_t pdl);

#ifdef __cplusplus
}
#endif
