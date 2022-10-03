/**
 * @file dmx_types.h
 * @author Mitch Weisbrod
 * @brief This file contains the types used for processing DMX and RDM as needed
 * for the base DMX driver. Types that are only used in rdm_tools.h should be
 * defined in rdm_types.h instead. Anonymous enums and constants defined in the
 * DMX or RDM standard should be defined in dmx_constants.h or rdm_constants.h
 * instead.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DMX port type.
 */
typedef unsigned int dmx_port_t;

/**
 * @brief RDM unique ID type.
 */
typedef int64_t rdm_uid_t;

typedef struct dmx_sniffer_data {
  uint32_t break_len;  // Length in microseconds of the last received DMX break.
  uint32_t mab_len;    // Length in microseconds of the last received DMX mark-after-break.
} dmx_sniffer_data_t;

/**
 * @brief Provides a synopsis of the received DMX packet so that users may 
 * quickly and easily process and respond to DMX data.
 */
typedef struct dmx_event {
  esp_err_t err;    // Evaluates to true if an error occurred reading DMX data.
  uint8_t sc;       // Start code of the DMX packet.
  size_t size;      // The size of the received DMX packet in bytes.
  bool is_rdm;      // True if the received packet is RDM.
} dmx_event_t;

#ifdef __cplusplus
}
#endif
