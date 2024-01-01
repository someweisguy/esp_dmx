/**
 * @file responder.h
 * @author Mitch Weisbrod
 * @brief This file contains functions used for registering RDM parameters. It
 * must be included by the user.
 */
#pragma once

#include "rdm/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The function type for user callbacks in RDM responses.
 */ // TODO: better docs
typedef void (*rdm_callback_t)(dmx_port_t dmx_num, rdm_header_t *request_header,
                               rdm_header_t *response_header, void *context);

#ifdef __cplusplus
}
#endif

// #include "rdm/responder/include/device_config.h"  // TODO
#include "rdm/responder/include/device_control.h"
#include "rdm/responder/include/discovery.h"
// #include "rdm/responder/include/display_setting.h"  // TODO
#include "rdm/responder/include/dmx_setup.h"
// #include "rdm/responder/include/power_setting.h"  // TODO
#include "rdm/responder/include/product_info.h"
#include "rdm/responder/include/queue_status.h"
#include "rdm/responder/include/rdm_info.h"
#include "rdm/responder/include/sensor_parameter.h"
