/**
 * @file rdm/responder/include/device_control.h
 * @author Mitch Weisbrod
 * @brief This file contains device control functions for the RDM responder. The
 * PIDs in device control include RDM_PID_IDENTIFY_DEVICE, RDM_PID_RESET_DEVICE,
 * RDM_PID_POWER_STATE, RDM_PID_PERFORM_SELFTEST, RDM_PID_SELF_TEST_DESCRIPTION,
 * RDM_PID_CAPTURE_PRESET, and RDM_PID_PRESET_PLAYBACK. This file also includes
 * getters and setters for these functions as appropriate.
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers the default response to RDM_PID_IDENTIFY_DEVICE requests.
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
bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context);

/**
 * @brief Gets a copy of the RDM identify device state of this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] identify A pointer which stores a copy of the identify device
 * state of this device.
 * @return The number of bytes that were copied or 0 on failure.
 */
size_t rdm_get_identify_device(dmx_port_t dmx_num, bool *identify);

/**
 * @brief Sets the RDM identify device state of this device.
 *
 * @param dmx_num The DMX port number.
 * @param identify The identify device state to which to set this device.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_set_identify_device(dmx_port_t dmx_num, const bool identify);

/**
 * @brief Registers the default response to RDM_PID_RESET_DEVICE requests.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_reset_device(dmx_port_t dmx_num, rdm_callback_t cb,
                               void *context);

/**
 * @brief Gets the reset type that was requested. This function is used to
 * determine the reset type that was requested, if any.
 *
 * @param dmx_num The DMX port number.
 * @param[out] reset The reset type that was requested.
 * @return The number of bytes that were copied or 0 on failure.
 */
size_t rdm_get_reset_device(dmx_port_t dmx_num, uint8_t *reset);

#ifdef __cplusplus
}
#endif
