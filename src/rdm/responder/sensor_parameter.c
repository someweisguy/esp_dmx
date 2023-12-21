#include "rdm/responder/include/sensor_parameter.h"

bool rdm_register_sensor_definition(dmx_port_t dmx_num, uint8_t defintion_count,
                         rdm_callback_t cb, void *context) {
  return false; // TODO
}

bool rdm_register_sensor(dmx_port_t dmx_num, uint8_t sensor_count,
                         rdm_callback_t cb, void *context) {
  return false;  // TODO
}

bool rdm_register_record_sensors(dmx_port_t dmx_num, rdm_callback_t cb,
                                 void *context) {
  return false;  // TODO
}

uint8_t rdm_sensor_get_count(dmx_port_t dmx_num, rdm_sub_device_t sub_device) {
  return 0;  // TODO
}