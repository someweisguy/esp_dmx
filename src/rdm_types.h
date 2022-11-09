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
typedef struct rdm_header_t {
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

/**
 * @brief Provides information about RDM responses.
 */
typedef struct rdm_response_t {
  esp_err_t err;             // Evaluates to true if an error occurred reading DMX data.
  rdm_response_type_t type;  // The type of the RDM response received.
  union {
    TickType_t timer;        // The amount of time in FreeRTOS ticks until the responder device will be ready to respond to the request. This field should be read when the response type received is RDM_RESPONSE_TYPE_ACK_TIMER.
    rdm_nr_t nack_reason;    // The reason that the request was unable to be fulfilled. This field should be read when the response type received is RDM_RESPONSE_TYPE_NACK_REASON.
    size_t num_params;       // The number of parameters received. This field should be read when the response type received is RDM_RESPONSE_TYPE_ACK or RDM_RESPONSE_TYPE_ACK_OVERFLOW.
  };
} rdm_response_t;

/**
 * @brief Parameters for use in RDM discovery requests. Discovery requests are
 * broadcast messages with a lower bound and upper bound. If a responding
 * device's UID falls within the lower bound and upper bound, it will respond to
 * the discovery request.
 */
typedef struct rdm_disc_unique_branch_t {
  rdm_uid_t lower_bound;  // The lower bound of the RDM discovery request.
  rdm_uid_t upper_bound;  // The upper bound of the RDM discovery request.
} rdm_disc_unique_branch_t;

/**
 * @brief Parameters for use with RDM discovery mute and un-mute requests. When
 * a responder device is successfully muted or un-muted, it responds with these
 * parameters.
 */
typedef struct rdm_disc_mute_t {
  bool managed_proxy;     // The manged proxy flag shall be set to 1 when the responder is a proxy device.
  bool sub_device;        // The sub-device flag shall be set to 1 when the responder supports sub-devices.
  bool boot_loader;       // The boot-loader flag shall only be set to 1 when the device is incapable of normal operation until receiving a firmware upload.
  bool proxied_device;    // The proxied device flag shall only be set to 1 when a proxy is responding to discovery on behalf of another device. This flag indicates that the response has come from a proxy rather than the actual device.
  rdm_uid_t binding_uid;  // The binding UID field shall only be included when the responding device contains multiple responder ports. If the device does contain multiple ports then the binding UID field shall contain the UID for the primary port on the device. If the device does not contain multiple responder ports, this field is set to 0.
} rdm_disc_mute_t;

/**
 * @brief Parameter for use with RDM device info requests.
 */
typedef struct rdm_device_info_t {
  uint32_t major_rdm_version;    // This field contains the major version number of the published RDM standard supported by the device.
  uint32_t minor_rdm_version;    // This field contains the minor version number of the published RDM standard supported by the device.
  int model_id;                  // This field identifies the device model ID of the root device or sub-device. The manufacturer shall not use the same ID to represent more than one unique model type.
  int coarse_product_category;   // Devices shall report a product category based on the product's primary function.
  int fine_product_category;     // Devices shall report a product category based on the product's primary function. The fine product category is optional.
  uint32_t software_version_id;  // This field indicates the software version ID for the device. The software version ID is a 32-bit value determined by the manufacturer.
  size_t footprint;              // This field species the DMX footprint - the number of consecutive DMX slots required.
  size_t current_personality;    // The current selected DMX personality of the device. The personality is the configured arrangement of DMX slots used by the device. Many devices may have multiple personalities from which to choose.
  size_t personality_count;      // The number of personalities supported by the device. The personality is the configured arrangement of DMX slots used by the device. Many devices may have multiple personalities from which to choose.
  int start_address;             // The DMX start address of the device. If the device or sub-device that the request is directed to has a DMX footprint of 0, then this field shall be set to -1.
  size_t sub_device_count;       // This parameter is used to retrieve the number of sub-devices respresented by the root device. The response for this field shall always be the same regardless of whether this message is directed to the root device or a sub-device.
  size_t sensor_count;           // This field indicates the number of available sensors in a root device or sub-device. When this parameter is directed to a sub-device, the reply shall be identical for any sub-device owned by a specific root device.
} rdm_device_info_t;

#ifdef __cplusplus
}
#endif
