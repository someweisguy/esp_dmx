/**
 * @file rdm/responder/include/discovery.h
 * @author Mitch Weisbrod
 * @brief This file contains RDM discovery functions for the RDM responder. The
 * PIDs in discovery include RDM_PID_DISC_UNIQUE_BRANCH, RDM_PID_DISC_MUTE, and
 * RDM_PID_DISC_UN_MUTE.
 */
#pragma once

#include <stdint.h>

#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers the default response to RDM_PID_DISC_UNIQUE_BRANCH requests.
 * This response is required by all RDM-capable devices. It is called when the
 * DMX driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_disc_unique_branch(dmx_port_t dmx_num, rdm_callback_t cb,
                                     void *context);

/**
 * @brief Registers the default response to RDM_PID_DISC_MUTE requests. This
 * response is required by all RDM-capable devices. It is called when the DMX
 * driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_disc_mute(dmx_port_t dmx_num, rdm_callback_t cb,
                            void *context);

/**
 * @brief Registers the default response to RDM_PID_DISC_UN_MUTE requests. This
 * response is required by all RDM-capable devices. It is called when the DMX
 * driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_disc_un_mute(dmx_port_t dmx_num, rdm_callback_t cb,
                               void *context);

#ifdef __cplusplus
}
#endif
