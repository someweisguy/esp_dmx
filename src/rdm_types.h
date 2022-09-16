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

typedef struct rdm_device_info {
  int rdm_version;
  int device_model_id;
  int product_category;
  uint32_t software_version;
  size_t dmx_footprint;
  int dmx_personality;
  size_t dmx_start_address;
  size_t sub_device_count;
} rdm_device_info_t;


#ifdef __cplusplus
}
#endif
