/**
 * @file rdm/responder/rdm_info.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/responder.h"
#include "rdm/include/types.h"

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
