/**
 * @file rdm/responder/dmx_setup.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO docs
bool rdm_register_dmx_personality(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context);

// TODO docs
bool rdm_register_dmx_personality_description(dmx_port_t dmx_num,
                                              rdm_callback_t cb, void *context);

/**
 * @brief Registers the default response to RDM_PID_DMX_START_ADDRESS requests.
 * This response is required by all RDM-capable devices which use a DMX address.
 * It is called when the DMX driver is initially installed if the DMX start
 * address is not set to DMX_START_ADDRESS_NONE.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_callback_t cb,
                                    void *context);

#ifdef __cplusplus
}
#endif
