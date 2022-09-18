/**
 * @file dmx_types.h
 * @author Mitch Weisbrod
 * @brief This file contains the types used for processing RDM as needed in
 * rdm_tools.h. Types that are used by the base driver should be defined in 
 * dmx_types.h instead. Anonymous enums and constants defined in the DMX or RDM
 * standard should be defined in dmx_constants.h or rdm_constants.h instead.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
typedef struct rdm_disc_mute {
  bool managed_proxy;
  bool sub_device;
  bool boot_loader;
  bool proxied_device;
  int64_t binding_uid;
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


#ifdef __cplusplus
}
#endif
