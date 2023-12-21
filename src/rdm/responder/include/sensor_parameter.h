/**
 * @file rdm/responder/include/sensor_param.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/include/parameter.h"
#include "dmx/types.h"
#include "rdm/responder.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool rdm_register_sensor_definition(dmx_port_t dmx_num, uint8_t defintion_count,
                         rdm_callback_t cb, void *context);

bool rdm_register_sensor(dmx_port_t dmx_num, uint8_t sensor_count,
                         rdm_callback_t cb, void *context);

bool rdm_register_record_sensors(dmx_port_t dmx_num, rdm_callback_t cb,
                                 void *context);

uint8_t rdm_sensor_get_count(dmx_port_t dmx_num, rdm_sub_device_t sub_device);

#ifdef __cplusplus
}
#endif
