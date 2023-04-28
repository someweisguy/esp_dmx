#include "functions.h"
#include "rdm_utils.h"

size_t rdm_get_preamble_len(const void *data) {
  size_t preamble_len = 0;
  for (const uint8_t *d = data; preamble_len <= 7; ++preamble_len) {
    if (d[preamble_len] == RDM_DELIMITER) break;
  }
  return preamble_len;
}

// size_t rdm_encode_mute(void *data, const rdm_disc_mute_t *param) {
//   size_t pdl = 2;
//   struct rdm_disc_mute_data_t *const ptr = data;
//   bzero(data, 2);  // FIXME: make the bit field more efficient?
//   ptr->managed_proxy = param->managed_proxy;
//   ptr->sub_device = param->sub_device;
//   ptr->boot_loader = param->boot_loader;
//   ptr->proxied_device = param->proxied_device;
//   if (param->binding_uid) {
//     uid_to_buf(ptr->binding_uid, param->binding_uid);
//     pdl += 6;
//   }
//   return pdl;
// }

// int rdm_decode_mute(const void *pd, rdm_disc_mute_t *param, int size,
//                     size_t pdl) {
//   int decoded = 0;
//   if (param != NULL) {
//     const struct rdm_disc_mute_data_t *const ptr = pd;
//     param->managed_proxy = ptr->managed_proxy;
//     param->sub_device = ptr->sub_device;
//     param->boot_loader = ptr->boot_loader;
//     param->proxied_device = ptr->proxied_device;
//     param->binding_uid = pdl > 2 ? buf_to_uid(ptr->binding_uid) : 0;
//     decoded = 1;
//   }
//   return decoded;
// }

// size_t rdm_encode_uids(void *data, const void *uids, int size) {
//   size_t pdl = 0;
//   for (int i = 0; i < size; ++i, pdl += 6) {
//     uid_to_buf(data + pdl, ((rdm_uid_t *)uids)[i]);
//   }
//   return pdl;
// }

// int rdm_decode_uids(const void *data, void *uids, int size) {
//   int decoded = 0;
//   for (int i = 0; decoded < size; ++decoded, i += 6) {
//     ((rdm_uid_t *)uids)[decoded] = buf_to_uid(data + i);
//   }
//   return decoded;
// }

size_t rdm_encode_16bit(void *pd, const void *data, int size) {
  size_t pdl = 0;
  if (data != NULL) {
    uint16_t *const restrict ptr = pd;
    const uint32_t *const restrict params = data;
    for (int i = 0; i < size; ++i) {
      ptr[i] = bswap16(params[i]);
    }
    pdl = size * sizeof(uint16_t);
  }
  return pdl;
}

// int rdm_decode_16bit(const void *pd, void *data, int size) {
//   int decoded = 0;
//   if (data != NULL) {
//     const uint16_t *const restrict ptr = pd;
//     uint32_t *restrict params = data;
//     for (int i = 0; i < size; ++i) {
//       params[i] = bswap16(ptr[i]);
//     }
//     decoded = size * sizeof(uint16_t);
//   }
//   return decoded;
// }

int rdm_decode_string(const void *pd, void *data, int size) {
  int decoded = 0;
  if (data != NULL) {
    char *restrict string = data;
    memcpy(string, pd, size);
    string[size] = 0;
    decoded = size + 1;
  }
  return decoded;
}

int rdm_decode_device_info(const void *pd, void *data, int size) {
  int decoded = 0;
  if (data != NULL) {
    const rdm_device_info_data_t *restrict ptr = pd;
    rdm_device_info_t *const restrict device_info = data;
    device_info->model_id = bswap16(ptr->model_id);
    device_info->coarse_product_category = ptr->coarse_product_category;
    device_info->fine_product_category = ptr->fine_product_category;
    device_info->software_version_id = bswap32(ptr->software_version_id);
    device_info->footprint = bswap16(ptr->footprint);
    device_info->current_personality = ptr->current_personality;
    device_info->personality_count = ptr->personality_count;
    device_info->start_address =
        ptr->start_address != 0xffff ? bswap16(ptr->start_address) : -1;
    device_info->sub_device_count = bswap16(ptr->sub_device_count);
    device_info->sensor_count = ptr->sensor_count;
    decoded = 1;
  }
  return decoded;
}

size_t rdm_get_message_len(const void *data) {
  return ((rdm_data_t *)data)->message_len;
}

bool rdm_is_request(const void *data) {
  return (((rdm_data_t *)data)->cc & 0x1) == 0;
}

size_t rdm_encode_nack_reason(rdm_mdb_t *mdb, rdm_nr_t nack_reason) {
  const size_t encoded = rdm_encode_16bit(mdb->pd, &nack_reason, 1);
  mdb->pdl = encoded;
  return encoded;
}

int rdm_decode_uids(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb != NULL && mdb->pd != NULL && data != NULL) {
    for (int i = 0; decoded < num && i < mdb->pdl; i += 6) {
        ((rdm_uid_t *)data)[decoded] = buf_to_uid(mdb->pd + i);
        ++decoded;
    }
  }
  return decoded;
}

size_t rdm_encode_mute(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data && num) {
    struct rdm_disc_mute_data_t *pd = (void *)mdb->pd;
    const rdm_disc_mute_t *param = data;
    bzero(mdb->pd, 2);  // FIXME: make the bit field more efficient?
    pd->managed_proxy = param->managed_proxy;
    pd->sub_device = param->sub_device;
    pd->boot_loader = param->boot_loader;
    pd->proxied_device = param->proxied_device;
    if (param->binding_uid != 0) {
      uid_to_buf(pd->binding_uid, param->binding_uid);
      encoded += 6;
    }
    encoded += 2;
  }
  mdb->pdl = encoded;
  return encoded;
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

size_t rdm_encode_8bit(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data) {
    uint8_t *pd = mdb->pd;
    const uint8_t *param = data;
    for (int i = 0; i < num; ++i) {
      // FIXME: ensure that the number of encoded bytes never exceeds 231
      pd[i] = param[i];
    }
    encoded = num;
  }
  mdb->pdl = encoded;
  return encoded;
}

int rdm_decode_8bit(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const uint8_t *pd = mdb->pd;
    uint8_t *param = data;
    for (int i = 0; i < num; ++i) {
      param[i] = pd[i];
    }
    decoded = num;
  }
  return decoded;
}

int rdm_decode_16bit(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const uint16_t *pd = (void *)mdb->pd;
    uint16_t *params = data;
    if (num > 231 / sizeof(uint16_t)) {
      num = 231 / sizeof(uint16_t);
    }
    for (int i = 0, j = 0; i < num && j < mdb->pdl;
         ++i, j += sizeof(uint16_t)) {
      params[i] = bswap16(pd[i]);
    }
  }
  return decoded;
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
    param->binding_uid = mdb->pdl > 2 ? buf_to_uid(pd->binding_uid) : 0;
    decoded = 1;
  }
  return decoded;
}

size_t rdm_encode_uids(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data && num) {
    for (int i = 0; i < num; ++i, encoded += 6) {
      uid_to_buf(mdb->pd + encoded, ((rdm_uid_t *)data)[i]);
    }
  }
  mdb->pdl = encoded;
  return encoded;
}