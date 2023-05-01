#pragma once

#include <stdint.h>
#include <string.h>

#include "endian.h"
#include "esp_dmx.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
bool rdm_read(dmx_port_t dmx_num, rdm_header_t *header, rdm_mdb_t *mdb);

// TODO: docs
size_t rdm_write(dmx_port_t dmx_num, const rdm_header_t *header,
                 const rdm_mdb_t *mdb);

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
 * @brief Decode a string from a buffer.
 *
 * @param pd A buffer in which the to decode is stored.
 * @param data A pointer into which to store the decoded data.
 * @param size The size of the buffer to store the decoded data.
 * @return The number of characters that was decoded.
 */
int rdm_decode_string(const void *pd, void *data, int size);

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
size_t rdm_encode_nack_reason(rdm_mdb_t *mdb, rdm_nr_t nack_reason);

// TODO: docs
int rdm_decode_uids(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_mute(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
size_t rdm_encode_device_info(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
size_t rdm_encode_string(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
size_t rdm_encode_8bit(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
int rdm_decode_8bit(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
int rdm_decode_16bit(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
int rdm_decode_mute(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_uids(rdm_mdb_t *mdb, const void *data, int num);

#ifdef __cplusplus
}
#endif
