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

/**
 * @brief Registers the RDM_PID_SUPPORTED_PARAMETERS parameter. This parameter
 * is required when supported parameters beyond the minimum required parameters.
 * This function is automatically called when installing the DMX driver.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional
 * parameters.
 */
bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_callback_t cb,
                                       void *context);

/**
 * @brief Registers the RDM_PID_PARAMETER_DESCRIPTION parameter. This parameter
 * provides RDM controllers with a description of the specified parameter. It is
 * designed to provide RDM controllers with information on non-standard RDM
 * parameters.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional
 * parameters.
 */
bool rdm_register_parameter_description(dmx_port_t dmx_num, rdm_callback_t cb,
                                        void *context);

#ifdef __cplusplus
}
#endif
