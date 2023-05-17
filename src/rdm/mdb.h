/**
 * @file mdb.h
 * @author Mitch Weisbrod
 * @brief This file contains functions used for encoded and decoding symbols in
 * an RDM packet's message data block (MDB).
 */
#pragma once

#include <stdint.h>
#include <string.h>

#include "endian.h"
#include "esp_dmx.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decodes 8-bit values from an RDM message data block.
 *
 * @param[in] mdb A pointer to a message data block.
 * @param[out] data A pointer to an array in which to store the decoded values.
 * @param num The number of elements in the array in which to store the decoded
 * values.
 * @return The number of parameters that were decoded.
 */
int rdm_decode_8bit(const rdm_mdb_t *mdb, void *data, int num);

/**
 * @brief Encodes 8-bit values into an RDM message data block.
 *
 * @param[out] mdb A pointer to a message data block.
 * @param[in] data A pointer to an array which stores the values to encode.
 * @param num The number of elements in the array to encode.
 * @return The number of bytes that were encoded.
 */
size_t rdm_encode_8bit(rdm_mdb_t *mdb, const void *data, int num);

/**
 * @brief Decodes 16-bit values from an RDM message data block.
 *
 * @param[in] mdb A pointer to a message data block.
 * @param[out] data A pointer to an array in which to store the decoded values.
 * @param num The number of elements in the array in which to store the decoded
 * values.
 * @return The number of parameters that were decoded.
 */
int rdm_decode_16bit(const rdm_mdb_t *mdb, void *data, int num);

/**
 * @brief Encodes 16-bit values into an RDM message data block.
 *
 * @param[out] mdb A pointer to a message data block.
 * @param[in] data A pointer to an array which stores the values to encode.
 * @param num The number of elements in the array to encode.
 * @return The number of bytes that were encoded.
 */
size_t rdm_encode_16bit(rdm_mdb_t *mdb, const void *data, int num);

// TODO: rdm_decode_32bit()

// TODO: rdm_encode_32bit()

/**
 * @brief Decodes UIDs from an RDM message data block.
 *
 * @param[in] mdb A pointer to a message data block.
 * @param[out] data A pointer to an array in which to store the decoded values.
 * @param num The number of elements in the array in which to store the decoded
 * values.
 * @return The number of parameters that were decoded.
 */
int rdm_decode_uids(const rdm_mdb_t *mdb, void *data, int num);

/**
 * @brief Encodes UIDs into an RDM message data block.
 *
 * @param[out] mdb A pointer to a message data block.
 * @param[in] data A pointer to an array which stores the values to encode.
 * @param num The number of elements in the array to encode.
 * @return The number of bytes that were encoded.
 */
size_t rdm_encode_uids(rdm_mdb_t *mdb, const void *data, int num);

// TODO: rdm_decode_64bit()

// TODO: rdm_encode_64bit()

/**
 * @brief Decodes a string from an RDM message data block. Strings may be up to
 * 32 characters long.
 *
 * @param[in] mdb A pointer to a message data block.
 * @param[out] data A pointer to an array in which to store the decoded string.
 * @param num The number of elements in the array in which to store the decoded
 * string.
 * @return The number of bytes that were decoded, including a null terminator.
 */
int rdm_decode_string(const rdm_mdb_t *mdb, void *data, int num);

/**
 * @brief Encodes a string into an RDM message data block. Strings may be up to
 * 32 characters long.
 *
 * @param[out] mdb A pointer to a message data block.
 * @param[in] data A pointer to an array which stores the encoded string.
 * @param num The number of elements in the array which stores the string to
 * encode.
 * @return The number of bytes that were decoded, not including a null
 * terminator.
 */
size_t rdm_encode_string(rdm_mdb_t *mdb, const void *data, int num);

/**
 * @brief Encodes a 16-bit NACK reason into an RDM message data block.
 *
 * @param[out] mdb A pointer to a message data block.
 * @param nack_reason A NACK reason to encode.
 * @return The number of bytes that were encoded.
 */
size_t rdm_encode_nack_reason(rdm_mdb_t *mdb, rdm_nr_t nack_reason);

/**
 * @brief Decodes an RDM mute parameter from a message data block.
 *
 * @param[in] mdb A pointer to a message data block.
 * @param[out] data A pointer to a buffer which stores the decoded mute
 * parameter, typically an rdm_disc_mute_t.
 * @param num The number of elements in the buffer in which to store the decoded
 * mute parameter.
 * @return The number of parameters that were decoded.
 */
int rdm_decode_mute(const rdm_mdb_t *mdb, void *data, int num);

/**
 * @brief Encodes an RDM mute parameter into a message data block.
 *
 * @param[out] mdb A pointer to a message data block.
 * @param[in] data A pointer to a buffer which stores the mute parameter to
 * encode, typically an rdm_disc_mute_t.
 * @param num The number of elements in the buffer in which the mute parameter
 * to encode is stored.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_mute(rdm_mdb_t *mdb, const void *data, int num);

/**
 * @brief Decodes an RDM device info parameter from a message data block.
 *
 * @param[in] mdb A pointer to a message data block.
 * @param[out] data A pointer to a buffer which stores the decoded device info
 * parameter, typically an rdm_device_info_t.
 * @param num The number of elements in the buffer in which to store the decoded
 * mute parameter.
 * @return The number of parameters that were decoded.
 */
int rdm_decode_device_info(const rdm_mdb_t *mdb, void *data, int num);

/**
 * @brief Encodes an RDM device info parameter into a message data block.
 *
 * @param[out] mdb A pointer to a message data block.
 * @param[in] data A pointer to a buffer which stores the device info parameter
 * to encode, typically an rdm_device_info_t.
 * @param num The number of elements in the buffer in which the device
 * info parameter to encode is stored.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_device_info(rdm_mdb_t *mdb, const void *data, int num);

#ifdef __cplusplus
}
#endif
