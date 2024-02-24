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

/**
 * @brief Gets a copy of the device hours for this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] device_hours A pointer to a buffer which will store the device
 * hours.
 * @return the number of bytes written to device_hours.
 */
size_t rdm_get_device_hours(dmx_port_t dmx_num, uint32_t *device_hours);

/**
 * @brief Sets the device hours for this device.
 *
 * @param dmx_num The DMX port number.
 * @param device_hours The device hours to which to set this device.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_set_device_hours(dmx_port_t dmx_num, uint32_t device_hours);

/**
 * @brief Registers the default response to RDM_PID_LAMP_HOURS requests. This
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
bool rdm_register_lamp_hours(dmx_port_t dmx_num, rdm_callback_t cb,
                             void *context);

/**
 * @brief Gets a copy of the lamp hours for this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] lamp_hours A pointer to a buffer which will store the lamp
 * hours.
 * @return the number of bytes written to lamp_hours.
 */
size_t rdm_get_lamp_hours(dmx_port_t dmx_num, uint32_t *lamp_hours);

/**
 * @brief Sets the device hours for this device.
 *
 * @param dmx_num The DMX port number.
 * @param lamp_hours The lamp hours to which to set this device.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_set_lamp_hours(dmx_port_t dmx_num, uint32_t lamp_hours);

#ifdef __cplusplus
}
#endif
