/**
 * @file dmx_types.h
 * @author Mitch Weisbrod
 * @brief This file contains the types used for processing RDM as needed in
 * esp_rdm.h. Types that are used by the base driver should be defined in 
 * dmx_types.h instead. Anonymous enums and constants defined in the DMX or RDM
 * standard should be defined in dmx_constants.h or rdm_constants.h instead.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dmx_types.h"
#include "freertos/FreeRTOS.h"
#include "rdm_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
typedef struct rdm_response {
  rdm_err_t err;
  size_t size;
  rdm_response_type_t type;
  union {
    TickType_t timer;
    rdm_nr_t nack_reason;
  };
} rdm_response_t;

typedef struct rdm_disc_unique_branch {
  rdm_uid_t upper_bound;
  rdm_uid_t lower_bound;
} rdm_disc_unique_branch_t;

// TODO: docs
typedef struct rdm_disc_mute {
  bool managed_proxy;
  bool sub_device;
  bool boot_loader;
  bool proxied_device;
  rdm_uid_t binding_uid;
} rdm_disc_mute_t;

// TODO: docs
typedef struct rdm_device_info {
  int rdm_version;
  int model_id;
  int product_category;
  uint32_t software_version;
  size_t footprint;
  size_t current_personality;
  size_t personality_count;
  size_t start_address;
  size_t sub_device_count;
  size_t sensor_count;
} rdm_device_info_t;

typedef struct rdm_software_version_label {
  char software_version_label[32];
} rdm_software_version_label_t;

#ifdef __cplusplus
}
#endif
