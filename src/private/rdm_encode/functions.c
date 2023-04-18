#include "functions.h"

bool rdm_is_valid(const void *data, size_t size) {
  return (size >= 24 && *(uint16_t *)data == (RDM_SC | (RDM_SUB_SC << 8))) ||
         (size >= 17 && (*(uint8_t *)data == RDM_PREAMBLE ||
                         *(uint8_t *)data == RDM_DELIMITER));
}

bool rdm_decode_packet(const void *data, rdm_header_t *header, rdm_mdb_t *mdb) {
  if (data == NULL || header == NULL || mdb == NULL) {
    return false;
  }

  // Verify that the checksum is correct
  bool is_valid = rdm_checksum_is_valid(data);
  if (!is_valid) {
    return is_valid;
  }

  // Decode the packet
  const uint8_t sc = *(uint8_t *)data;
  if (sc == RDM_SC) {
    const rdm_data_t *const rdm = data;

    // Assign or ignore the parameter data
    const size_t pdl = rdm->pdl;
    if (pdl > 231) {
      return false;  // PDL must be <= 231
    } else if (pdl > 0) {
      mdb->pd = (void *)(&(rdm->pd));
    } else {
      mdb->pd = NULL;
    }
    mdb->pdl = pdl;

    // Copy the remaining header data
    header->dest_uid = buf_to_uid(rdm->destination_uid);
    header->src_uid = buf_to_uid(rdm->source_uid);
    header->tn = rdm->tn;
    header->port_id = rdm->port_id;  // Also copies response_type
    header->message_count = rdm->message_count;
    header->sub_device = bswap16(rdm->sub_device);
    header->cc = rdm->cc;
    header->pid = bswap16(rdm->pid);

  } else {
    // Decode the EUID
    uint8_t buf[6];
    const uint8_t *d = data;
    const size_t preamble_len = rdm_get_preamble_len(data);
    d = &d[preamble_len + 1];
    for (int i = 0, j = 0; i < 6; ++i, j += 2) {
      buf[i] = (d[j] & 0x55) | (d[j + 1] & 0xaa); // TODO: & each byte
    }
    header->src_uid = buf_to_uid(buf);

    // Fill out the remaining header and MDB data
    header->dest_uid = 0;
    header->tn = -1;
    header->response_type = RDM_RESPONSE_TYPE_ACK;
    header->port_id = -1;
    header->message_count = -1;
    header->sub_device = -1;
    header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
    header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    mdb->pdl = 0;
    mdb->preamble_len = preamble_len;
  }

  return is_valid;
}

size_t rdm_encode_packet(void *data, rdm_header_t *header, rdm_mdb_t *mdb) {
  if (data == NULL || header == NULL || mdb == NULL) {
    return 0;
  }

  size_t encoded;
  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
      header->cc == RDM_CC_DISC_COMMAND_RESPONSE) {
    // Encode the preamble
    if (mdb->preamble_len > 7) {
      mdb->preamble_len = 7;
    }
    uint8_t *d = data;
    for (int i = 0; i < mdb->preamble_len; ++i) {
      d[i] = RDM_PREAMBLE;
    }
    d[mdb->preamble_len] = RDM_DELIMITER;

    // Encode the EUID and calculate the checksum
    uint16_t checksum = 0;
    d = &(d[mdb->preamble_len + 1]);
    for (int i = 0, j = 5; i < 12; i += 2, --j) {
      d[i] = ((uint8_t *)&(header->src_uid))[j] | 0xaa;
      d[i + 1] = ((uint8_t *)&(header->src_uid))[j] | 0x55;
      checksum += ((uint8_t *)&(header->src_uid))[j] + 0xaa + 0x55;
    }

    // Encode the checksum
    d[12] = (checksum >> 8) | 0xaa;
    d[13] = (checksum >> 8) | 0x55;
    d[14] = (checksum & 0xff) | 0xaa;
    d[15] = (checksum & 0xff) | 0x55;

    encoded = mdb->preamble_len + 1 + 16;
  } else {
    rdm_data_t *const rdm = data;

    // Encode the parameter data
    if (mdb->pdl > 231) {
      return 0;  // PDL must be <= 231
    } else if (mdb->pdl > 0) {
      if (mdb->pd == NULL) {
        return 0;  // Invalid MDB
      }
      memmove(&rdm->pd, mdb->pd, mdb->pdl);
    }
    rdm->pdl = mdb->pdl;

    // Encode the packet header
    const size_t message_len = 24 + mdb->pdl;
    rdm->sc = RDM_SC;
    rdm->sub_sc = RDM_SUB_SC;
    rdm->message_len = message_len;
    uid_to_buf(rdm->destination_uid, header->dest_uid);
    uid_to_buf(rdm->source_uid, header->src_uid);
    rdm->tn = header->tn;
    rdm->port_id = header->port_id;  // Also copies response_type
    rdm->message_count = header->message_count;
    rdm->sub_device = bswap16(header->sub_device);
    rdm->cc = header->cc;
    rdm->pid = bswap16(header->pid);

    // Encode the checksum
    uint16_t checksum = 0;
    const uint8_t *d = data;
    for (int i = 0; i < message_len; ++i) {
      checksum += d[i];
    }
    *(uint16_t *)&d[message_len] = bswap16(checksum);

    encoded = message_len + 2;
  }

  return encoded;
}

size_t rdm_get_preamble_len(const void *data) {
  size_t preamble_len = 0;
  for (const uint8_t *d = data; preamble_len <= 7; ++preamble_len) {
    if (d[preamble_len] == RDM_DELIMITER) break;
  }
  return preamble_len;
}

bool rdm_checksum_is_valid(const void *data) {
  uint16_t sum = 0;
  uint16_t checksum;

  const uint8_t *d = data;
  const uint8_t sc = d[0];

  // Get the packet checksum
  if (sc == RDM_SC) {
    // Calculate sum and decode checksum normally
    const size_t message_len = rdm_get_message_len(data);
    for (int i = 0; i < message_len; ++i) {
      sum += d[i];
    }
    checksum = bswap16(*(uint16_t *)(&d[message_len]));
  } else {
    // Decode checksum from encoded DISC_UNIQUE_BRANCH response
    // FIXME: return false if preamble_len is >7
    d = &d[rdm_get_preamble_len(data) + 1];
    for (int i = 0; i < 12; ++i) {
      sum += d[i];
    }
    checksum = (d[14] & 0x55) | (d[15] & 0xaa);
    checksum |= ((d[12] & 0x55) | (d[13] & 0xaa)) << 8;
  }

  return (sum == checksum);
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

int rdm_decode_mute(const void *pd, rdm_disc_mute_t *param, int size,
                    size_t pdl) {
  int decoded = 0;
  if (param != NULL) {
    const struct rdm_disc_mute_data_t *const ptr = pd;
    param->managed_proxy = ptr->managed_proxy;
    param->sub_device = ptr->sub_device;
    param->boot_loader = ptr->boot_loader;
    param->proxied_device = ptr->proxied_device;
    param->binding_uid = pdl > 2 ? buf_to_uid(ptr->binding_uid) : 0;
    decoded = 1;
  }
  return decoded;
}

size_t rdm_encode_uids(void *data, const void *uids, int size) {
  size_t pdl = 0;
  for (int i = 0; i < size; ++i, pdl += 6) {
    uid_to_buf(data + pdl, ((rdm_uid_t *)uids)[i]);
  }
  return pdl;
}

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

int rdm_decode_16bit(const void *pd, void *data, int size) {
  int decoded = 0;
  if (data != NULL) {
    const uint16_t *const restrict ptr = pd;
    uint32_t *restrict params = data;
    for (int i = 0; i < size; ++i) {
      params[i] = bswap16(ptr[i]);
    }
    decoded = size * sizeof(uint16_t);
  }
  return decoded;
}

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
  if (mdb != NULL && mdb->pd != NULL && data != NULL && num == 1) {
    struct rdm_disc_mute_data_t *pd = mdb->pd;
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
  if (mdb && mdb->pd && data) {
    rdm_device_info_data_t *const pd = mdb->pd;
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
  if (mdb && mdb->pd && data) {
    char *dest = mdb->pd;
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
  if (mdb && mdb->pdl && data) {
    uint8_t *pd = mdb->pd;
    const uint8_t *param = data;
    for (int i = 0; i < num; ++i) {
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

/* TODO
size_t rdm_encode_whatever(rdm_mdb_t *mdb, const void *data, int num);
int rdm_decode_whatever(const rdm_mdb_t *mdb, void *data, int num);
*/
