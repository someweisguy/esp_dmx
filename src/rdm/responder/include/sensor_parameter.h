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

// TODO: docs
bool rdm_register_sensor_definition(dmx_port_t dmx_num,
                                    uint8_t definition_count, rdm_callback_t cb,
                                    void *context);

// TODO: docs
bool rdm_register_sensor_value(dmx_port_t dmx_num, uint8_t sensor_count,
                               rdm_callback_t cb, void *context);

// TODO: docs
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
