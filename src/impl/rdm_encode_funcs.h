#pragma once

#include <stdint.h>

#include "esp_dmx.h"
#include "impl/rdm_encode_types.h"
#include "rdm_constants.h"
#include "rdm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

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
