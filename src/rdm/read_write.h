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
int rdm_decode_8bit(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_8bit(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
int rdm_decode_16bit(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_16bit(rdm_mdb_t *mdb, const void *data, int num);

// TODO: rdm_decode_32bit()

// TODO: rdm_encode_32bit()

// TODO: docs
int rdm_decode_uids(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_uids(rdm_mdb_t *mdb, const void *data, int num);

// TODO: rdm_decode_64bit()

// TODO: rdm_encode_64bit()

// TODO: docs
int rdm_decode_string(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_string(rdm_mdb_t *mdb, const void *data, int num);

// TODO: rdm_decode_nack_reason()

// TODO: docs
size_t rdm_encode_nack_reason(rdm_mdb_t *mdb, rdm_nr_t nack_reason);

// TODO: docs
int rdm_decode_mute(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_mute(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
int rdm_decode_device_info(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_device_info(rdm_mdb_t *mdb, const void *data, int num);

#ifdef __cplusplus
}
#endif
