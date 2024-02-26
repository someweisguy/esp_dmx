#include "rdm/responder/include/sensor_parameter.h"

#include <string.h>

#include "dmx/include/driver.h"
#include "dmx/include/parameter.h"
#include "dmx/include/service.h"
#include "dmx/include/types.h"
#include "rdm/include/driver.h"
#include "rdm/responder/include/utils.h"

typedef struct rdm_sensors_t {
  uint8_t sensor_count;
  rdm_sensor_value_t sensor_value[];
} rdm_sensors_t;

static rdm_sensors_t *rdm_get_sensors(dmx_port_t dmx_num,
                                      rdm_sub_device_t sub_device) {
  return dmx_parameter_get_data(dmx_num, sub_device, RDM_PID_SENSOR_VALUE);
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
      for (int i = 0; i < sensors->sensor_count; ++i) {
        sensors->sensor_value[i].present_value = 0;
        sensors->sensor_value[i].lowest_value = 0;
        sensors->sensor_value[i].highest_value = 0;
        sensors->sensor_value[i].recorded_value = 0;
      }
      value = &sensors->sensor_value[0];
      assert(value != NULL);
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

static size_t rdm_rhd_set_record_sensors(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  // Verify the requested sensor num is valid
  uint8_t sensor_num;
  if (!rdm_read_pd(dmx_num, definition->set.request.format, &sensor_num,
                   sizeof(sensor_num))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }
  if (sensor_num > rdm_sensor_get_count(dmx_num, header->sub_device) &&
      sensor_num != RDM_SENSOR_NUM_MAX) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Record the RDM sensor
  if (!rdm_sensor_record(dmx_num, header->sub_device, sensor_num)) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
}

static size_t rdm_rhd_get_sensor_definition(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  // Verify the requested sensor num is valid
  uint8_t sensor_num;
  if (!rdm_read_pd(dmx_num, definition->set.request.format, &sensor_num,
                   sizeof(sensor_num))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }
  if (sensor_num > rdm_sensor_get_count(dmx_num, header->sub_device) &&
      sensor_num != RDM_SENSOR_NUM_MAX) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Verify the sensor has a definition
  const rdm_sensor_definition_t *sensor_def =
      rdm_sensor_definition_get(dmx_num, header->sub_device, sensor_num);
  if (sensor_def == NULL) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }

  return rdm_write_ack(dmx_num, header, definition->get.response.format,
                       sensor_def, sizeof(*sensor_def));
}

bool rdm_register_sensor_definition(dmx_port_t dmx_num, rdm_callback_t cb,
                                    void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  const rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, RDM_SUB_DEVICE_ROOT);
  DMX_CHECK(sensors != NULL, false,
            "RDM_PID_SENSOR_VALUE must be registered first");

  const rdm_pid_t pid = RDM_PID_SENSOR_DEFINITION;

  // Check if this is the first time this function has been called
  const bool first_time_func_called =
      dmx_parameter_exists(dmx_num, RDM_SUB_DEVICE_ROOT, pid);

  // Add the parameter as an array of RDM sensor definitions
  const uint8_t sensor_count = sensors->sensor_count;
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_DYNAMIC, NULL,
                         sizeof(rdm_sensor_definition_t) * sensor_count)) {
    return false;
  }

  // Set sensor definition numbers to 0xff to flag they haven't been defined
  if (first_time_func_called) {
    rdm_sensor_definition_t *sensor_defs =
        dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT, pid);
    assert(sensor_defs != NULL);
    for (int i = 0; i < sensor_count; ++i) {
      sensor_defs[i].num = 0xff;
    }
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_get_sensor_definition,
              .request.format = "b$",
              .response.format = "bbbbwwwwba"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = sizeof(uint8_t),
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_register_sensor_value(dmx_port_t dmx_num, uint8_t sensor_count,
                               rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_SENSOR_VALUE;

  // Add the parameter
  size_t size =
      sizeof(rdm_sensors_t) + (sizeof(rdm_sensor_value_t) * sensor_count);
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_DYNAMIC, NULL, size)) {
    return false;
  }
  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, RDM_SUB_DEVICE_ROOT);
  assert(sensors != NULL);
  sensors->sensor_count = sensor_count;
  for (int i = 0; i < sensor_count; ++i) {
    sensors->sensor_value[i].sensor_num = i;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
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
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_register_record_sensors(dmx_port_t dmx_num, rdm_callback_t cb,
                                 void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_RECORD_SENSORS;

  // Add the parameter as NULL static
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_STATIC, NULL, 0)) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_SET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .set = {.handler = rdm_rhd_set_record_sensors,
              .request.format = "b$",
              .response.format = NULL},
      .pdl_size = sizeof(uint8_t),
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

uint8_t rdm_sensor_get_count(dmx_port_t dmx_num, rdm_sub_device_t sub_device) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX, false, "sub_device error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, sub_device);
  if (sensors == NULL) {
    return 0;
  }

  return sensors->sensor_count;
}

size_t rdm_sensor_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                      uint8_t sensor_num, rdm_sensor_value_t *sensor_value) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX, false, "sub_device error");
  DMX_CHECK(sensor_num < rdm_sensor_get_count(dmx_num, sub_device), 0,
            "sensor_num error");
  DMX_CHECK(sensor_value != NULL, 0, "sensor_value is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, sub_device);
  if (sensors == NULL) {
    return 0;
  }

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  *sensor_value = sensors->sensor_value[sensor_num];
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return sizeof(*sensor_value);
}

bool rdm_sensor_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                    uint8_t sensor_num, int16_t value) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX || sub_device == RDM_SUB_DEVICE_ALL,
            false, "sub_device error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Validate the sensor_num
  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, sub_device);
  if (sensors == NULL || (sensor_num > sensors->sensor_count &&
                          sensor_num != RDM_SENSOR_NUM_MAX)) {
    return false;
  }

  // Set the sensor value
  if (sensor_num != RDM_SENSOR_NUM_MAX) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    sensors->sensor_value[sensor_num].present_value = value;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    for (int i = 0; i < sensors->sensor_count; ++i) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      sensors->sensor_value[i].present_value = value;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  }

  return true;
}

bool rdm_sensor_record(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                       uint8_t sensor_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX || sub_device == RDM_SUB_DEVICE_ALL,
            false, "sub_device error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Validate the sensor_num
  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, sub_device);
  if (sensors == NULL || (sensor_num > sensors->sensor_count &&
                          sensor_num != RDM_SENSOR_NUM_MAX)) {
    return false;
  }

  if (sensor_num == 0xff) {
    for (int i = sensors->sensor_count; i >= 0; --i) {
      rdm_sensor_value_t *sensor = &sensors->sensor_value[i];
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      sensor->recorded_value = sensor->present_value;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  } else {
    rdm_sensor_value_t *sensor = &sensors->sensor_value[sensor_num];
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    sensor->recorded_value = sensor->present_value;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  return true;
}

bool rdm_sensor_reset(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                      uint8_t sensor_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX || sub_device == RDM_SUB_DEVICE_ALL,
            false, "sub_device error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Validate the sensor_num
  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, sub_device);
  if (sensors == NULL || (sensor_num > sensors->sensor_count &&
                          sensor_num != RDM_SENSOR_NUM_MAX)) {
    return false;
  }

  if (sensor_num == 0xff) {
    for (int i = sensors->sensor_count; i >= 0; --i) {
      rdm_sensor_value_t *sensor = &sensors->sensor_value[i];
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      sensor->present_value = 0;
      sensor->lowest_value = 0;
      sensor->highest_value = 0;
      sensor->recorded_value = 0;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  } else {
    rdm_sensor_value_t *sensor = &sensors->sensor_value[sensor_num];
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    sensor->present_value = 0;
    sensor->lowest_value = 0;
    sensor->highest_value = 0;
    sensor->recorded_value = 0;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  return true;
}

bool rdm_sensor_definition_add(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               const rdm_sensor_definition_t *definition) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX || sub_device == RDM_SUB_DEVICE_ALL,
            false, "sub_device error");
  DMX_CHECK(definition != NULL, false, "definition is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Validate the definition sensor number
  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, sub_device);
  if (sensors == NULL || definition->num > sensors->sensor_count) {
    return false;
  }

  // Validate that sensor definitions have been registered
  rdm_sensor_definition_t *sensor_defs = dmx_parameter_get_data(
      dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_SENSOR_DEFINITION);
  if (sensor_defs == NULL) {
    return false;
  }

  // Validate that the sensor hasn't already been defined
  if (sensor_defs[definition->num].num != 0xff) {
    return false;
  }

  // Copy the definition to the parameter data
  memcpy(&sensor_defs[definition->num], definition, sizeof(*definition));

  return true;
}

const rdm_sensor_definition_t *rdm_sensor_definition_get(
    dmx_port_t dmx_num, rdm_sub_device_t sub_device, uint8_t sensor_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX || sub_device == RDM_SUB_DEVICE_ALL,
            false, "sub_device error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Validate the definition sensor number
  rdm_sensors_t *sensors = rdm_get_sensors(dmx_num, sub_device);
  if (sensors == NULL || sensor_num > sensors->sensor_count) {
    return NULL;
  }

  // Validate that sensor definitions have been registered
  rdm_sensor_definition_t *sensor_defs = dmx_parameter_get_data(
      dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_SENSOR_DEFINITION);
  if (sensor_defs == NULL) {
    return NULL;
  }

  // Validate the sensor has a definition
  if (sensor_defs[sensor_num].num == 0xff) {
    return NULL;
  }

  return &sensor_defs[sensor_num];
}
