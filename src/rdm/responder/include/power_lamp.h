/**
 * @file rdm/responder/include/power_lamp.h
 * @author Mitch Weisbrod
 * @brief This file contains power and lamp setting functions for the RDM
 * responder. The PIDs in power and lamp settings include RDM_PID_DEVICE_HOURS,
 * RDM_PID_LAMP_HOURS, RDM_PID_LAMP_STRIKES, RDM_PID_LAMP_STATE,
 * RDM_PID_LAMP_ON_MODE, and RDM_PID_DEVICE_POWER_CYCLES. This file also
 * includes getters and setters for these functions as appropriate.
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers the default response to RDM_PID_DEVICE_HOURS requests. This
 * function attempts to load a value from NVS. If none is found, the default is
 * 0.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional
 * responses.
 */
bool rdm_register_device_hours(dmx_port_t dmx_num, rdm_callback_t cb,
                               void *context);

#ifdef __cplusplus
}
#endif
