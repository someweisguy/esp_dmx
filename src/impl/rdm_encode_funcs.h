#pragma once

#include <stdint.h>

#include "esp_dmx.h"
#include "impl/rdm_encode_types.h"
#include "rdm_constants.h"
#include "rdm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool rdm_decode_header(const void *source, size_t size, rdm_header_t *header) {
  bool is_rdm = false;
  rdm_data_t *rdm = (rdm_data_t *)source;

  if ((rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER) && size > 17) {
    // Decode a DISC_UNIQUE_BRANCH response

    // Find the length of the discovery response preamble (0-7 bytes)
    int preamble_len = 0;
    const uint8_t *data = source;
    for (; preamble_len < 7; ++preamble_len) {
      if (data[preamble_len] == RDM_DELIMITER) {
        break;
      }
    }
    if (data[preamble_len] != RDM_DELIMITER || size < preamble_len + 17) {
      return is_rdm;  // Not a valid discovery response
    }

    // Decode the 6-byte UID and get the packet sum
    rdm_uid_t uid = 0;
    uint16_t sum = 0;
    data = &((uint8_t *)source)[preamble_len + 1];
    for (int i = 5, j = 0; i >= 0; --i, j += 2) {
      ((uint8_t *)&uid)[i] = data[j] & 0x55;
      ((uint8_t *)&uid)[i] |= data[j + 1] & 0xaa;
      sum += ((uint8_t *)&uid)[i] + 0xff;
    }

    // Decode the checksum received in the response
    uint16_t checksum;
    for (int i = 1, j = 12; i >= 0; --i, j += 2) {
      ((uint8_t *)&checksum)[i] = data[j] & 0x55;
      ((uint8_t *)&checksum)[i] |= data[j + 1] & 0xaa;
    }

    // Return RDM data to the caller
    header->destination_uid = RDM_BROADCAST_UID;
    header->source_uid = uid;
    header->tn = 0;
    header->response_type = RDM_RESPONSE_TYPE_ACK;
    header->message_count = 0;
    header->sub_device = 0;
    header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
    header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    header->pdl = 0;
    header->checksum_is_valid = (sum == checksum);
    is_rdm = true;

  } else if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC && size >= 26) {
    // Decode a standard RDM message

    // Calculate the checksum
    uint16_t sum = 0;
    const uint16_t checksum = bswap16(*(uint16_t *)(source + rdm->message_len));
    for (int i = 0; i < rdm->message_len; ++i) {
      sum += *(uint8_t *)(source + i);
    }

    // Return RDM data to the caller
    header->destination_uid = buf_to_uid(rdm->destination_uid);
    header->source_uid = buf_to_uid(rdm->source_uid);
    header->tn = rdm->tn;
    header->response_type = rdm->response_type;
    header->message_count = rdm->message_count;
    header->sub_device = bswap16(rdm->sub_device);
    header->cc = rdm->cc;
    header->pid = bswap16(rdm->pid);
    header->pdl = rdm->pdl;
    header->checksum_is_valid = (sum == checksum);
    is_rdm = (size >= rdm->message_len + 2);  // Include checksum
  }

  return is_rdm;
}

size_t rdm_encode_disc_unique_branch(rdm_data_t *destination, size_t size,
                                     const rdm_disc_unique_branch_t *param) {
  rdm_disc_unique_branch_data_t *buf = destination->pd;
  uid_to_buf(buf->lower_bound, param->lower_bound);
  uid_to_buf(buf->upper_bound, param->upper_bound);
  return sizeof(*buf);
}

size_t rdm_decode_disc_unique_branch(const rdm_data_t *source, size_t size, 
                                     rdm_disc_unique_branch_t *param) {
  const rdm_disc_unique_branch_data_t *buf = source->pd;
  param->lower_bound = buf_to_uid(buf->lower_bound);
  param->upper_bound = buf_to_uid(buf->upper_bound);
  return 1;
}

size_t rdm_encode_disc_mute(rdm_data_t *destination, size_t size,
                            const rdm_disc_mute_t *param) {
  rdm_disc_mute_data_t *buf = destination->pd;
  buf->managed_proxy = param->managed_proxy;
  buf->sub_device = param->sub_device;
  buf->boot_loader = param->boot_loader;
  buf->proxied_device = param->proxied_device;
  size_t bytes_encoded = sizeof(buf->control_field);
  if (param->binding_uid != 0) {
    // Binding UID is an optional parameter
    uid_to_buf(buf->binding_uid, param->binding_uid);
    bytes_encoded += sizeof(buf->binding_uid);
  }
  return bytes_encoded;
}

size_t rdm_decode_disc_mute(const rdm_data_t *source, size_t size,
                            rdm_disc_mute_t *param) {
  const rdm_disc_mute_data_t *buf = source->pd;
  param->managed_proxy = buf->managed_proxy;
  param->sub_device = buf->sub_device;
  param->boot_loader = buf->boot_loader;
  param->proxied_device = buf->proxied_device;
  if (source->pdl >= 8) {
    // Binding UID is an optional parameter
    param->binding_uid = buf_to_uid(buf->binding_uid);
  } else {
    param->binding_uid = 0;
  }
  return 1;
}

size_t rdm_encode_device_info(rdm_data_t *destination, size_t size,
                              const rdm_device_info_t *param) {
  rdm_device_info_data_t *buf = destination->pd;
  buf->rdm_version = param->rdm_version;
  buf->model_id = param->model_id;
  buf->product_category = param->product_category;
  buf->software_version = param->software_version;
  buf->footprint = param->footprint;
  buf->current_personality = param->current_personality;
  buf->personality_count = param->personality_count;
  buf->start_address = param->start_address;
  buf->sub_device_count = param->sub_device_count;
  buf->sensor_count = param->sensor_count;
  return sizeof(rdm_device_info_data_t);
}

size_t rdm_decode_device_info(const rdm_data_t *source, size_t size, 
                              rdm_device_info_t *param) {
  const rdm_device_info_data_t *buf = source->pd;
  param->rdm_version = buf->rdm_version;
  param->model_id = buf->model_id;
  param->product_category = buf->product_category;
  param->software_version = buf->software_version;
  param->footprint = buf->footprint;
  param->current_personality = buf->current_personality;
  param->start_address = buf->start_address;
  param->sub_device_count = buf->sub_device_count;
  param->sensor_count = buf->sensor_count;
  return 1;
}

#ifdef __cplusplus
}
#endif
