/**
 * @file rdm/responder/include/rdm_info.h
 * @author Mitch Weisbrod
 * @brief This file contains RDM information functions for the RDM responder.
 * The PIDs in RDM information include RDM_PID_SUPPORTED_PARAMETERS and
 * RDM_PID_PARAMTER_DESCRIPTION. This file also includes getters and setters for
 * these functions as appropriate.
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO docs
bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_callback_t cb,
                                       void *context);

// TODO docs
bool rdm_register_parameter_description(dmx_port_t dmx_num, rdm_callback_t cb,
                                        void *context);

#ifdef __cplusplus
}
#endif
