/**
 * @file responder.h
 * @author Mitch Weisbrod
 * @brief This file contains functions used for registering RDM parameters. It
 * must be included by the user.
 */
#pragma once

#include <stdint.h>

#include "dmx/types.h"
#include "rdm/parameters.h"
#include "rdm/types.h"

#include "rdm/responder/device_config.h"
#include "rdm/responder/device_control.h"
#include "rdm/responder/discovery.h"
#include "rdm/responder/display_setting.h"
#include "rdm/responder/dmx_setup.h"
#include "rdm/responder/power_setting.h"
#include "rdm/responder/product_info.h"
#include "rdm/responder/queue_status.h"
#include "rdm/responder/rdm_info.h"
#include "rdm/responder/sensor_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif


// TODO: docs
bool rdm_register_manufacturer_specific_simple(
    dmx_port_t dmx_num, rdm_pid_description_t description, void *data,
    const char *format, rdm_callback_t cb, void *context, bool nvs);


#ifdef __cplusplus
}
#endif
