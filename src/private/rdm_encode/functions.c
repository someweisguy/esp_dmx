#include "functions.h"

size_t rdm_encode_disc_response(void *data, size_t preamble_len,
                                const rdm_uid_t uid) {
  // Encode the RDM preamble and delimiter
  if (preamble_len > 7) {
    preamble_len = 7;  // Max preamble_len is 7
  }
  for (int i = 0; i < preamble_len; ++i) {
    ((uint8_t *)data)[i] = RDM_PREAMBLE;
  }
  ((uint8_t *)data)[preamble_len] = RDM_DELIMITER;

  // Encode the UID and calculate the checksum
  uint16_t checksum = 0;
  for (int i = 8, j = 5; i < 20; i += 2, --j) {
    ((uint8_t *)data)[i] = ((uint8_t *)&uid)[j] | 0xaa;
    ((uint8_t *)data)[i + 1] = ((uint8_t *)&uid)[j] | 0x55;
    checksum += ((uint8_t *)&uid)[j] + (0xaa + 0x55);
  }

  // Encode the checksum
  ((uint8_t *)data)[20] = (checksum >> 8) | 0xaa;
  ((uint8_t *)data)[21] = (checksum >> 8) | 0x55;
  ((uint8_t *)data)[22] = checksum | 0xaa;
  ((uint8_t *)data)[23] = checksum | 0x55;

  return preamble_len + 17;
}

bool rdm_decode_disc_response(const uint8_t *data, rdm_uid_t *uid) {
  // Find the length of the discovery response preamble (0-7 bytes)
  int preamble_len = 0;
  for (; preamble_len < 7; ++preamble_len) {
    if (data[preamble_len] == RDM_DELIMITER) {
      break;
    }
  }

  // Decode the 6-byte UID and get the packet sum
  uint16_t sum = 0;
  data = &((uint8_t *)data)[preamble_len + 1];
  for (int i = 5, j = 0; i >= 0; --i, j += 2) {
    ((uint8_t *)uid)[i] = data[j] & 0x55;
    ((uint8_t *)uid)[i] |= data[j + 1] & 0xaa;
    sum += ((uint8_t *)uid)[i] + 0xff;
  }

  // Decode the checksum received in the response
  uint16_t checksum;
  for (int i = 1, j = 12; i >= 0; --i, j += 2) {
    ((uint8_t *)&checksum)[i] = data[j] & 0x55;
    ((uint8_t *)&checksum)[i] |= data[j + 1] & 0xaa;
  }

  return (sum == checksum);
}

size_t rdm_encode_header(void *data, const rdm_header_t *header) {
  rdm_data_t *const rdm = data;
  rdm->sc = RDM_SC;
  rdm->sub_sc = RDM_SUB_SC;
  rdm->message_len = header->pdl + RDM_BASE_PACKET_SIZE - 2;
  uid_to_buf(rdm->destination_uid, header->destination_uid);
  uid_to_buf(rdm->source_uid, header->source_uid);
  rdm->tn = header->tn;
  rdm->port_id = header->port_id;
  rdm->message_count = header->message_count;
  rdm->sub_device = bswap16(header->sub_device);
  rdm->cc = header->cc;
  rdm->pid = bswap16(header->pid);
  rdm->pdl = header->pdl;

  // Calculate checksum
  uint16_t checksum = 0;
  for (int i = 0; i < rdm->message_len; ++i) {
    checksum += ((uint8_t *)data)[i];
  }
  *(uint16_t *)(data + rdm->message_len) = bswap16(checksum);

  return RDM_BASE_PACKET_SIZE;
}

bool rdm_decode_header(const void *data, rdm_header_t *header) {
  const rdm_data_t *const rdm = data;
  const bool has_header = (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC);
  if (has_header) {
    header->destination_uid = buf_to_uid(rdm->destination_uid);
    header->source_uid = buf_to_uid(rdm->source_uid);
    header->tn = rdm->tn;
    header->port_id = rdm->port_id;
    header->message_count = rdm->message_count;
    header->sub_device = bswap16(rdm->sub_device);
    header->cc = rdm->cc;
    header->pid = bswap16(rdm->pid);
    header->pdl = rdm->pdl;

    // Calculate checksum
    uint16_t sum = 0;
    const uint16_t checksum = bswap16(*(uint16_t *)(data + rdm->message_len));
    for (int i = 0; i < rdm->message_len; ++i) {
      sum += *(uint8_t *)(data + i);
    }
    header->checksum_is_valid = (sum == checksum);
  }
  return has_header;
}

size_t rdm_encode_mute(void *data, const rdm_disc_mute_t *param) {
  size_t pdl = 2;
  struct rdm_disc_mute_data_t *const ptr = data;
  bzero(data, 2);  // FIXME: make the bit field more efficient?
  ptr->managed_proxy = param->managed_proxy;
  ptr->sub_device = param->sub_device;
  ptr->boot_loader = param->boot_loader;
  ptr->proxied_device = param->proxied_device;
  if (param->binding_uid) {
    uid_to_buf(ptr->binding_uid, param->binding_uid);
    pdl += 6;
  }
  return pdl;
}

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

int rdm_decode_uids(const void *data, void *uids, int size) {
  int decoded = 0;
  for (int i = 0; decoded < size; ++decoded, i += 6) {
    ((rdm_uid_t *)uids)[decoded] = buf_to_uid(data + i);
  }
  return decoded;
}

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

size_t rdm_encode_8bit(void *pd, const void *data, int size) {
  size_t pdl = 0;
  if (data != NULL) {
    uint8_t *restrict ptr = pd;
    const uint32_t *restrict params = data;
    for (int i = 0; i < size; ++i) {
      ptr[i] = params[i];
    }
    pdl = size;
  }
  return pdl;
}

int rdm_decode_8bit(const void *pd, void *data, int size) {
  int decoded = 0;
  if (data != NULL) {
    const uint8_t *restrict ptr = pd;
    uint32_t *restrict params = data;
    for (int i = 0; i < size; ++i) {
      params[i] = ptr[i];
    }
    decoded = size;
  }
  return decoded;
}

size_t rdm_encode_string(void *pd, const void *data, int size) {
  size_t pdl = 0;
  if (data != NULL) {
    char *restrict destination = pd;
    const char *restrict source = data;
    while (pdl < size) {
      if (*source) {
        *destination = *source;
        ++destination;
        ++source;
        ++pdl;
      } else {
        break;  // Don't encode null terminators
      }
    }
  }
  return pdl;
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

size_t rdm_encode_device_info(void *pd, const void *data, int size) {
  size_t pdl = 0;
  if (data != NULL) {
    rdm_device_info_data_t *const restrict ptr = pd;
    const rdm_device_info_t *const restrict device_info = data;
    ptr->major_rdm_version = 1;
    ptr->minor_rdm_version = 0;
    ptr->model_id = bswap16(device_info->model_id);
    ptr->coarse_product_category = device_info->coarse_product_category;
    ptr->fine_product_category = device_info->fine_product_category;
    ptr->software_version_id = bswap32(device_info->software_version_id);
    ptr->footprint = bswap16(device_info->footprint);
    ptr->current_personality = device_info->current_personality;
    ptr->personality_count = device_info->personality_count;
    ptr->start_address = device_info->start_address != -1
                            ? bswap16(device_info->start_address)
                            : 0xffff;
    ptr->sub_device_count = bswap16(device_info->sub_device_count);
    ptr->sensor_count = device_info->sensor_count;
    pdl = sizeof(rdm_device_info_data_t);
  }
  return pdl;
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

bool rdm_is_valid(const void *data, size_t size) {
  return (size >= 24 && *(uint16_t *)data == (RDM_SC | (RDM_SUB_SC << 8))) ||
         (size >= 17 && (*(uint8_t *)data == RDM_PREAMBLE ||
                         *(uint8_t *)data == RDM_DELIMITER));
}

size_t rdm_get_message_len(const void *data) {
  return ((rdm_data_t *)data)->message_len;
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
    d = &d[rdm_get_preamble_len(data) + 1];
    for (int i = 0; i < 12; ++i) {
      sum += d[i];
    }
    checksum = (d[14] & 0x55) | (d[15] & 0xaa);
    checksum |= ((d[12] & 0x55) | (d[13] & 0xaa)) << 8;
  }

  return (sum == checksum);
}

bool rdm_is_request(const void *data) {
  return (((rdm_data_t *)data)->cc & 0x1) == 0;
}

bool rdm_decode_packet(const void *data, size_t size, rdm_header2_t *header,
                       rdm_mdb_t *mdb, void *pd) {
  // Check if the packet appears to be valid RDM
  bool is_valid = rdm_is_valid(data, size);
  if (!is_valid) {
    return is_valid;
  }

  // Verify that the checksum is correct
  is_valid = rdm_checksum_is_valid(data);
  if (!is_valid) {
    return is_valid;
  }

  // Decode the packet
  const uint8_t sc = *(uint8_t *)data;
  if (sc == RDM_SC) {
    const rdm_data_t *const rdm = data;

    // Copy or ignore the parameter data
    const size_t pdl = rdm->pdl;
    if (pdl > 231) {
      return false;  // PDL must be <231
    } else if (pdl > 0) {
      memcpy(pd, &rdm->pd, pdl);
      mdb->pd = pd;
    } else {
      mdb->pd = NULL;
    }
    mdb->pdl = pdl;

    // Check if packet is a request or response
    if (rdm_is_request(data)) {
      header->port_id = rdm->port_id;
      mdb->response_type = -1;
    } else {
      mdb->response_type = rdm->response_type;
      header->port_id = -1;
    }

    // Copy the remaining header data
    header->dest_uid = buf_to_uid(rdm->destination_uid);
    header->src_uid = buf_to_uid(rdm->source_uid);
    header->tn = rdm->tn;
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
      buf[i] = (d[j] & 0x55) | (d[j + 1] & 0xaa);
    }
    header->src_uid = buf_to_uid(buf);

    // Fill out the remaining header and MDB data
    header->dest_uid = 0;
    header->tn = -1;
    header->port_id = -1;
    header->message_count = -1;
    header->sub_device = -1;
    header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
    header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    mdb->response_type = RDM_RESPONSE_TYPE_ACK;
    mdb->pdl = 0;
    mdb->preamble_len = preamble_len;
  }

  return is_valid;
}
