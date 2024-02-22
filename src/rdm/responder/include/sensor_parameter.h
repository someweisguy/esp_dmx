/**
 * @file rdm/responder/include/sensor_parameter.h
 * @author Mitch Weisbrod
 * @brief This file contains RDM sensor parameter functions for the RDM
 * responder. The PIDs in sensor parameter include RDM_PID_SENSOR_DEFINITION,
 * RDM_PID_SENSOR_VALUE, and RDM_PID_RECORD_SENSORS. This file also includes
 * getters and setters for these functions as appropriate.
 */
#pragma once

#include "dmx/include/parameter.h"
#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers the RDM_PID_SENSOR_DEFINITION parameter which is used to
 * define RDM sensors and the values that they return. RDM_PID_SENSOR_VALUE must
 * be registered before this function is called. Sensors may be defined using 
 * rdm_sensor_definition_add().
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional
 */
bool rdm_register_sensor_definition(dmx_port_t dmx_num, rdm_callback_t cb,
                                    void *context);

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

/**
 * @brief Returns the number of sensors supported by the device. This is the
 * same number passed in the sensor_count argument of the
 * rdm_register_sensor_value() function.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @return The number of sensors supported by the sub-device.
 */
uint8_t rdm_sensor_get_count(dmx_port_t dmx_num, rdm_sub_device_t sub_device);

/**
 * @brief Copies the sensor values of a specified sensor into a user buffer.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param sensor_num The sensor number.
 * @param sensor_value A pointer to a rdm_sensor_value_t into which to store the
 * sensor values.
 * @return The number of bytes written to sensor_value.
 */
size_t rdm_sensor_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                      uint8_t sensor_num, rdm_sensor_value_t *sensor_value);

/**
 * @brief Sets a value to a specified sensor. This function should be called
 * periodically as RDM controllers may request sensor data at any given time.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param sensor_num The sensor number.
 * @param value The value to be written to the sensor.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_sensor_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                    uint8_t sensor_num, int16_t value);

/**
 * @brief Records the current value set in a specified sensor. The value
 * recorded on the sensor is the last value that was set using rdm_sensor_set().
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param sensor_num The sensor number.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_sensor_record(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                       uint8_t sensor_num);

/**
 * @brief Resets all the values in a sensor to 0.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param sensor_num The sensor number.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_sensor_reset(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                      uint8_t sensor_num);

/**
 * @brief Defines a sensor. This definition is returned when an RDM controller
 * sends a GET RDM_PID_SENSOR_DEFINITION request to this device. Sensors may
 * only be defined once. Attempting to define a sensor multiple times will cause
 * this function to fail.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param[in] definition A pointer to a sensor definition.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_sensor_definition_add(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               const rdm_sensor_definition_t *definition);

/**
 * @brief Gets the definition of an RDM sensor. The RDM_PID_SENSOR_DEFINITION 
 * parameter must be registered before this function may be called.
 * 
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param sensor_num The sensor number.
 * @return A pointer to the RDM sensor definition or NULL on failure. 
 */
const rdm_sensor_definition_t *rdm_sensor_definition_get(
    dmx_port_t dmx_num, rdm_sub_device_t sub_device, uint8_t sensor_num);

#ifdef __cplusplus
}
#endif
