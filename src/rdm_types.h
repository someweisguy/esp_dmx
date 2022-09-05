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

/**
 * @brief The parameter data area included in the responses to mute and un-mute
 * messages includes a control field to inform the controller about specific
 * properties of the device, and may include an optional binding UID field to
 * indicate the physical arrangement of the responding device.
 */
typedef struct rdm_disc_mute_param {
  uint16_t control_field;  // Informs the RDM controller about specific properties of the device. The managed proxy flag (bit 0) shall be set when the responder is a proxy device. The sub-device flag (bit 1) shall be set when the responder supports sub-devices. The boot-loader flag (bit 2) shall only be set when the device is incapable of normal operation until receiving a firmware upload. The proxied device flag (bit 3) shall only be set when a proxy is responding to discovery on behalf of another device.
  uint64_t binding_uid;    // Shall only be included when the responding device contains multiple responder ports. If the device does contain multiple ports, then the Binding UID field shall contain the UID for the primary port on the device. Is 0 if no binding UID is received.
} rdm_disc_mute_param_t;

#ifdef __cplusplus
}
#endif
