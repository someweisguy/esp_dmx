#include "mdb.h"

#include <stdint.h>

#include "dmx/driver.h"
#include "dmx/hal.h"
#include "esp_dmx.h"
#include "rdm/utils.h"

/**
 * @brief A packed struct which can be used to help process RDM discovery mute
 * parameters. RDM sends data in most-significant byte first, so endianness must
 * be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_disc_mute_data_t {
  union {
    struct {
      uint8_t managed_proxy : 1;
      uint8_t sub_device : 1;
      uint8_t boot_loader : 1;
      uint8_t proxied_device : 1;
    };
    uint16_t control_field;
  };
  uint8_t binding_uid[6];
} rdm_disc_mute_data_t;

/**
 * @brief A packed struct which can be used to help process RDM device info
 * parameters. RDM sends data in most-significant byte first, so endianness must
 * be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_device_info_data_t {
  uint8_t major_rdm_version;
  uint8_t minor_rdm_version;
  uint16_t model_id;
  uint8_t coarse_product_category;
  uint8_t fine_product_category;
  uint32_t software_version_id;
  uint16_t footprint;
  uint8_t current_personality;
  uint8_t personality_count;
  uint16_t start_address;
  uint16_t sub_device_count;
  uint8_t sensor_count;
} rdm_device_info_data_t;

int rdm_decode_8bit(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const uint8_t *pd = (void *)mdb->pd;
    uint8_t *param = data;
    for (int i = 0; i < num && decoded * sizeof(uint8_t) < mdb->pdl; ++i) {
      param[i] = pd[i];
      ++decoded;
    }
  }
  return decoded;
}

size_t rdm_encode_8bit(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data) {
    uint8_t *pd = mdb->pd;
    const uint8_t *param = data;
    for (int i = 0; i < num && encoded < sizeof(mdb->pd); ++i) {
      pd[i] = param[i];
      encoded += sizeof(uint8_t);
    }
  }
  mdb->pdl = encoded;
  return encoded;
}

int rdm_decode_16bit(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const uint16_t *pd = (void *)mdb->pd;
    uint16_t *param = data;
    for (int i = 0; i < num && decoded * sizeof(uint16_t) < mdb->pdl; ++i) {
      param[i] = bswap16(pd[i]);
    }
  }
  return decoded;
}

size_t rdm_encode_16bit(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data) {
    uint16_t *const pd = (void *)mdb->pd;
    const uint16_t *const param = data;
    for (int i = 0; i < num && encoded < sizeof(mdb->pd); ++i) {
      pd[i] = bswap16(param[i]);
      encoded += sizeof(uint16_t);
    }
  }
  mdb->pdl = encoded;
  return encoded;
}

// TODO: rdm_decode_32bit()

// TODO: rdm_encode_32bit()

int rdm_decode_uids(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const struct __attribute__((__packed__)) {
      uint16_t manufacturer;
      uint64_t device;
    } *pd = (void *)mdb->pd;
    rdm_uid_t *param = data;
    for (int i = 0; i < num && decoded * 6 < mdb->pdl; ++i) {
      param[i] = bswap48(&pd[i]);
      ++decoded;
    }
  }
  return decoded;
}

size_t rdm_encode_uids(rdm_mdb_t *mdb, const void *data, int num) {
  // FIXME: this code doesn't work
  // size_t encoded = 0;
  // if (mdb && data) {
  //   struct __attribute__((__packed__)) {
  //     uint16_t manufacturer;
  //     uint64_t device;
  //   } *pd = (void *)mdb->pd;
  //   const rdm_uid_t *param = data;
  //   for (int i = 0; i < num && encoded < sizeof(mdb->pd); ++i) {
  //     uidcpy(&(pd[i]), &(param[i]));
  //     encoded += 6;  // Size of UID in bytes
  //   }
  // }
  // mdb->pdl = encoded;
  // return encoded;
  size_t encoded = 0;
  if (mdb && data && num) {
    for (int i = 0; i < num; ++i, encoded += 6) {
      uidcpy(mdb->pd + encoded, &(((rdm_uid_t *)data)[i]));
    }
  }
  mdb->pdl = encoded;
  return encoded;
}

// TODO: rdm_decode_64bit()

// TODO: rdm_encode_64bit()

int rdm_decode_string(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const char *src = (void *)mdb->pd;
    char *dest = data;
    while (decoded < num && decoded < sizeof(mdb->pd)) {
      if (*src) {
        *dest = *src;
        ++decoded;
        ++dest;
        ++src;
      } else {
        *dest = 0;  // Encode null terminator
        ++decoded;
        break;
      }
    }
  }
  return decoded;
}

size_t rdm_encode_string(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data) {
    char *dest = (void *)mdb->pd;
    const char *src = data;
    while (encoded < num && encoded < 32) {
      if (*src) {
        *dest = *src;
        ++encoded;
        ++dest;
        ++src;
      } else {
        break;  // Don't encode null terminators
      }
    }
  }
  mdb->pdl = encoded;
  return encoded;
}

// TODO: rdm_decode_nack_reason()

size_t rdm_encode_nack_reason(rdm_mdb_t *mdb, rdm_nr_t nack_reason) {
  return rdm_encode_16bit(mdb, &nack_reason, 1);
}

int rdm_decode_mute(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const struct rdm_disc_mute_data_t *const pd = (void *)mdb->pd;
    rdm_disc_mute_t *param = data;
    param->managed_proxy = pd->managed_proxy;
    param->sub_device = pd->sub_device;
    param->boot_loader = pd->boot_loader;
    param->proxied_device = pd->proxied_device;
    param->binding_uid = mdb->pdl > 2 ? bswap48(pd->binding_uid) : 0;
    decoded = 1;
  }
  return decoded;
}

size_t rdm_encode_mute(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data && num) {
    struct rdm_disc_mute_data_t *pd = (void *)mdb->pd;
    const rdm_disc_mute_t *param = data;
    pd->control_field = 0;  // Zero out bits 4 through 15
    pd->managed_proxy = param->managed_proxy;
    pd->sub_device = param->sub_device;
    pd->boot_loader = param->boot_loader;
    pd->proxied_device = param->proxied_device;
    if (param->binding_uid != 0) {
      uidcpy(pd->binding_uid, &(param->binding_uid));
      encoded += 6;
    }
    encoded += 2;
  }
  mdb->pdl = encoded;
  return encoded;
}

int rdm_decode_device_info(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb != NULL && data != NULL) {
    const rdm_device_info_data_t *pd = (void *)mdb->pd;
    rdm_device_info_t *const restrict param = data;
    param->model_id = bswap16(pd->model_id);
    param->coarse_product_category = pd->coarse_product_category;
    param->fine_product_category = pd->fine_product_category;
    param->software_version_id = bswap32(pd->software_version_id);
    param->footprint = bswap16(pd->footprint);
    param->current_personality = pd->current_personality;
    param->personality_count = pd->personality_count;
    param->start_address =
        pd->start_address != 0xffff ? bswap16(pd->start_address) : -1;
    param->sub_device_count = bswap16(pd->sub_device_count);
    param->sensor_count = pd->sensor_count;
    decoded = 1;
  }
  return decoded;
}

size_t rdm_encode_device_info(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data) {
    rdm_device_info_data_t *const pd = (void *)mdb->pd;
    const rdm_device_info_t *param = data;
    pd->major_rdm_version = 1;
    pd->minor_rdm_version = 0;
    pd->model_id = bswap16(param->model_id);
    pd->coarse_product_category = param->coarse_product_category;
    pd->fine_product_category = param->fine_product_category;
    pd->software_version_id = bswap32(param->software_version_id);
    pd->footprint = bswap16(param->footprint);
    pd->current_personality = param->current_personality;
    pd->start_address =
        param->start_address != -1 ? bswap16(param->start_address) : 0xffff;
    pd->sub_device_count = bswap16(param->sub_device_count);
    pd->sensor_count = param->sensor_count;
    encoded = sizeof(rdm_device_info_data_t);
  }
  mdb->pdl = encoded;
  return encoded;
}
