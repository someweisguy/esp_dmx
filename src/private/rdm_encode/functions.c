#include "functions.h"

size_t rdm_encode_response(uint8_t *data, size_t mdb_len) {
  // Encode the ? and calculate the checksum
  uint16_t checksum = 0;
  
  // Encode the checksum
  for (int i = 0; i < mdb_len; ++i) {
    checksum += ((uint8_t *)data)[i];
  }
  *(uint16_t *)(data + mdb_len) = bswap16(checksum);
 
  return mdb_len + 2;
}

size_t rdm_encode_disc_response(uint8_t *data, size_t preamble_len,
                                const rdm_uid_t uid) {
  // Encode the RDM preamble and delimiter
  if (preamble_len > 7) {
    preamble_len = 7;  // Max preamble_len is 7
  }
  for (int i = 0; i < preamble_len; ++i) {
    data[i] = RDM_PREAMBLE;
  }
  data[7] = RDM_DELIMITER;

  // Encode the UID and calculate the checksum
  uint16_t checksum = 0;
  for (int i = 8, j = 5; i < 20; i += 2, --j) {
    data[i] = ((uint8_t *)&uid)[j] | 0xaa;
    data[i + 1] = ((uint8_t *)&uid)[j] | 0x55;
    checksum += ((uint8_t *)&uid)[j] + (0xaa + 0x55);
  }

  // Encode the checksum
  data[20] = (checksum >> 8) | 0xaa;
  data[21] = (checksum >> 8) | 0x55;
  data[22] = checksum | 0xaa;
  data[23] = checksum | 0x55;

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

  return (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC);
}

size_t rdm_encode_uids(void *data, const rdm_uid_t *uids, size_t size) {
  size_t pdl = 0;
  for (int i = 0; i < size; ++i, pdl += 6) {
    uid_to_buf(data + pdl, uids[i]);
  }
  return pdl;
}

size_t rdm_decode_uids(const void *data, rdm_uid_t *const uids, size_t size,
                       size_t pdl) {
  size_t num_params = 0;
  for (int i = 0; num_params < size; ++num_params, i += 6) {
    uids[num_params] = buf_to_uid(data + i);
  }
  return num_params;
}

size_t rdm_encode_mute(void *data, const rdm_disc_mute_t *param) {
  size_t pdl = 2;
  struct rdm_disc_mute_data_t *const ptr = data;
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

size_t rdm_decode_mute(const void *data, rdm_disc_mute_t *param, size_t size,
                       size_t pdl) {
  const struct rdm_disc_mute_data_t *const ptr = data;
  param->managed_proxy = ptr->managed_proxy;
  param->sub_device = ptr->sub_device;
  param->boot_loader = ptr->boot_loader;
  param->proxied_device = ptr->proxied_device;
  param->binding_uid = pdl > 2 ? buf_to_uid(ptr->binding_uid) : 0;
  return 1;
}

size_t rdm_encode_16bit(void *pd, const void *data, size_t size) {
  if (size > RDM_PACKET_MAX_PARAMS(uint16_t)) {
    // Don't encode more data than can fit in a single packet
    size = RDM_PACKET_MAX_PARAMS(uint16_t);
  }
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

size_t rdm_decode_16bit(const void *pd, void *data, size_t size, size_t pdl) {
  if (size > RDM_PDL_MAX_PARAMS(pdl, uint16_t)) {
    // Don't decode more data than can fit in a single packet
    size = RDM_PDL_MAX_PARAMS(pdl, uint16_t);
  }
  if (data != NULL) {
    const uint16_t *const restrict ptr = pd;
    uint32_t *restrict params = data;
    for (int i = 0; i < size; ++i) {
      params[i] = bswap16(ptr[i]);
    }
  }
  return pdl / sizeof(uint16_t);
}

size_t rdm_encode_8bit(void *pd, const void *data, size_t size) {
  if (size > RDM_PACKET_MAX_PARAMS(uint8_t)) {
    // Don't encode more data than can fit in a single packet
    size = RDM_PACKET_MAX_PARAMS(uint8_t);
  }
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

size_t rdm_decode_8bit(const void *pd, void *data, size_t size,
                       size_t pdl) {
  if (size > RDM_PDL_MAX_PARAMS(pdl, uint8_t)) {
    size = RDM_PDL_MAX_PARAMS(pdl, uint8_t);
  }
  if (data != NULL) {
    const uint8_t *restrict ptr = pd;
    uint32_t *restrict params = data;
    for (int i = 0; i < size; ++i) {
      params[i] = ptr[i];
    }
  }
  return pdl;
}

size_t rdm_encode_string(void *pd, const void *data, size_t size) {
  if (size > RDM_PACKET_MAX_PARAMS(char)) {
    size = RDM_PACKET_MAX_PARAMS(char);
  }
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

size_t rdm_decode_string(const void *pd, void *data, size_t size, size_t pdl) {
  if (size > RDM_PDL_MAX_PARAMS(pdl, char)) {
    size = RDM_PDL_MAX_PARAMS(pdl, char);
  }
  if (data != NULL) {
    char *restrict string = data;
    memcpy(string, pd, size);
    string[size] = 0;
  }
  return pdl;
}

size_t rdm_encode_device_info(void *pd, const void *data) {
  if (data != NULL) {
    rdm_device_info_data_t *const restrict ptr = pd;
    const rdm_device_info_t *const restrict device_info = data;
    ptr->major_rdm_version = device_info->major_rdm_version;
    ptr->minor_rdm_version = device_info->minor_rdm_version;
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
  }
  return sizeof(rdm_device_info_data_t);
}

size_t rdm_decode_device_info(const void *pd, void *data, size_t size,
                              size_t pdl) {
  if (data != NULL) {
    const rdm_device_info_data_t *restrict ptr = pd;
    rdm_device_info_t *const restrict device_info = data;
    device_info->major_rdm_version = ptr->major_rdm_version;
    device_info->minor_rdm_version = ptr->minor_rdm_version;
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
  }
  return 1;
}