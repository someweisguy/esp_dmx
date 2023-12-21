#include "rdm/responder/include/sensor_parameter.h"

#include "dmx/include/driver.h"
#include "dmx/include/io.h"
#include "dmx/include/parameter.h"
#include "dmx/include/struct.h"
#include "dmx/types.h"
#include "rdm/responder/include/utils.h"

typedef struct rdm_sensors_t {
  uint8_t sensor_count;
  rdm_sensor_value_t sensor_value[];
} rdm_sensors_t;

static rdm_sensors_t *rdm_get_sensors(dmx_port_t dmx_num,
                                      rdm_sub_device_t sub_device) {
  return dmx_parameter_get(dmx_num, sub_device, RDM_PID_SENSOR_VALUE);
}

static size_t rdm_rhd_get_set_sensor_value(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  // Get a pointer to the desired command
  const struct rdm_command_t *command;
  if (header->cc == RDM_CC_SET_COMMAND) {
    command = &definition->set;
  } else {
    command = &definition->get;
  }

  // Verify the requested sensor num is valid
  uint8_t sensor_num;
  if (!rdm_read_pd(dmx_num, command->request.format, &sensor_num,
                   sizeof(sensor_num))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }
  if (sensor_num > rdm_sensor_get_count(dmx_num, header->sub_device) &&
      sensor_num != RDM_SENSOR_NUM_MAX) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, header->sub_device);
  rdm_sensor_value_t *value;
  assert(sensors != NULL);

  if (header->cc == RDM_CC_GET_COMMAND) {
    // Cannot GET all sensors
    if (sensor_num == RDM_SENSOR_NUM_MAX) {
      return rdm_write_nack_reason(dmx_num, header,
                                   RDM_NR_UNSUPPORTED_COMMAND_CLASS);
    }

    // Get the requested sensor value
    value = &sensors->sensor_value[sensor_num];
    assert(value != NULL);
  } else {
    if (sensor_num == RDM_SENSOR_NUM_MAX) {
      // Reset all the sensors
      for (int i = sensors->sensor_count; i >= 0; --i) {
        value = &sensors->sensor_value[sensor_num];
        assert(value != NULL);
        value->present_value = 0;
        value->lowest_value = 0;
        value->highest_value = 0;
        value->recorded_value = 0;
      }
    } else {
      // Get the requested sensor value and reset it
      value = &sensors->sensor_value[sensor_num];
      assert(value != NULL);
      value->present_value = 0;
      value->lowest_value = 0;
      value->highest_value = 0;
      value->recorded_value = 0;
    }
  }

  return rdm_write_ack(dmx_num, header, command->response.format, value,
                       sizeof(*value));
}

bool rdm_register_sensor_definition(dmx_port_t dmx_num, uint8_t defintion_count,
                                    rdm_callback_t cb, void *context) {
  return false;  // TODO
}

bool rdm_register_sensor(dmx_port_t dmx_num, uint8_t sensor_count,
                         rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_QUEUED_MESSAGE;
  static const rdm_parameter_definition_t definition = {
      .pid = pid,
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_get_set_sensor_value,
              .request.format = "b$",
              .response.format = "bwwww$"},
      .set = {.handler = rdm_rhd_get_set_sensor_value,
              .request.format = "b$",
              .response.format = "bwwww$"},
      .pdl_size = sizeof(uint8_t),
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_parameter_define(&definition);

  // Add the parameter
  const bool nvs = false;
  size_t size =
      sizeof(rdm_sensors_t) + (sizeof(rdm_sensor_value_t) * sensor_count);
  bool success = dmx_parameter_add_dynamic(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                                           nvs, NULL, size);
  if (success) {
    rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, RDM_SUB_DEVICE_ROOT);
    assert(sensors != NULL);
    sensors->sensor_count = sensor_count;
    for (int i = 0; i < sensor_count; ++i) {
      sensors->sensor_value[i].sensor_num = i;
    }
  }

  return true;
}

bool rdm_register_record_sensors(dmx_port_t dmx_num, rdm_callback_t cb,
                                 void *context) {
  return false;  // TODO
}

uint8_t rdm_sensor_get_count(dmx_port_t dmx_num, rdm_sub_device_t sub_device) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX, false, "sub_device error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  assert(sub_device == RDM_SUB_DEVICE_ROOT);  // TODO: implement sub-devices

  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, sub_device);
  if (sensors == NULL) {
    return 0;
  }

  return sensors->sensor_count;
}