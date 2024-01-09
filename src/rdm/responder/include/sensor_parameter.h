/**
 * @file rdm/responder/include/sensor_parameter.h
 * @author Mitch Weisbrod
 * @brief This file contains RDM sensor parameter functions for the RDM
 * responder. The PIDs in sensor parameter include RDM_PID_SENSOR_DEFINITION,
 * RDM_PID_SENSOR_VALUE, and RDM_PID_RECORD_SENSORS. This file also includes
 * getters and setters for these function as appropriate.
 */
#pragma once

#include "dmx/include/parameter.h"
#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
// TODO: implement rdm_register_sensor_definition()
bool rdm_register_sensor_definition(dmx_port_t dmx_num,
                                    uint8_t definition_count, rdm_callback_t cb,
                                    void *context);
*/

/**
 * @brief Registers the RDM_PID_SENSOR_VALUE parameter which is used to retreive
 * or reset sensor data. This function must be called before registering any
 * other sensor parameters. Sensor values can be get or set using the functions
 * rdm_sensor_get() or rdm_sensor_set(). Sensor values should be periodically
 * set within the user's main function.
 *
 * @param dmx_num The DMX port number.
 * @param sensor_count The number of sensors to register with the parameter.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional
 * parameters.
 */
bool rdm_register_sensor_value(dmx_port_t dmx_num, uint8_t sensor_count,
                               rdm_callback_t cb, void *context);

/**
 * @brief Registers the RDM_PID_RECORD_SENSORS parameter which instructs devices
 * such as dimming racks that monitor load changes to store the current value
 * for monitoring sensor changes. The function rdm_register_sensor_value()
 * should be called before calling this function. Recording sensor values
 * involves copying the most recent value set with rdm_sensor_set() to a buffer
 * so that it may be retrieved by RDM controllers.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional
 * parameters.
 */
bool rdm_register_record_sensors(dmx_port_t dmx_num, rdm_callback_t cb,
                                 void *context);

// TODO: docs
uint8_t rdm_sensor_get_count(dmx_port_t dmx_num, rdm_sub_device_t sub_device);

// TODO: docs
size_t rdm_sensor_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                      uint8_t sensor_num, rdm_sensor_value_t *sensor_value);

// TODO: docs
bool rdm_sensor_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                    uint8_t sensor_num, int16_t value);

// TODO: docs
bool rdm_sensor_record(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                       uint8_t sensor_num);

// TODO: docs
bool rdm_sensor_reset(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                      uint8_t sensor_num);

#ifdef __cplusplus
}
#endif
