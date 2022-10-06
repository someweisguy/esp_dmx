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

/**
 * @brief Provides a synopsis of the received RDM packet so that users may
 * quickly and easily process and respond to RDM data.
 */
typedef struct rdm_header {
  rdm_uid_t destination_uid;  // The UID of the target device(s).
  rdm_uid_t source_uid;       // The UID of the device originating this packet.
  size_t tn;                  // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
  union {
    int port_id;                        // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
    rdm_response_type_t response_type;  // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
  };
  size_t message_count;       // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
  int sub_device;             // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
  rdm_cc_t cc;                // The command class (CC) specifies the action of the message.
  rdm_pid_t pid;              // The parameter ID (PID) identifies a specific type of parameter data.
  size_t pdl;                 // The parameter data length (PDL) is the number of slots included in the parameter data area that it precedes.
  bool checksum_is_valid;     // True if the RDM checksum is valid.
} rdm_header_t;

// TODO: docs
typedef struct rdm_response {
  esp_err_t err;
  rdm_response_type_t type;
  union {
    unsigned int timer;  // TODO: use ticktype
    rdm_nr_t nack_reason;
    size_t num_params;
  };
} rdm_response_t;

// TODO: docs
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

// TODO: docs
typedef struct rdm_software_version_label {
  char software_version_label[32];
} rdm_software_version_label_t;

typedef struct rdm_identify_device {
  bool on;
} rdm_identify_device_t;

typedef struct rdm_dmx_start_address {
  int address;
} rdm_dmx_start_address_t;

#ifdef __cplusplus
}
#endif
