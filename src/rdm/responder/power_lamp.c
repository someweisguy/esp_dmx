#include "rdm/responder/include/power_lamp.h"

#include "dmx/hal/include/nvs.h"
#include "dmx/include/device.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/include/driver.h"
#include "rdm/responder/include/utils.h"

bool rdm_register_device_hours(dmx_port_t dmx_num, rdm_callback_t cb,
                               void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DEVICE_HOURS;

  // Attempt to load the value from NVS
  uint32_t device_hours;
  if (!dmx_nvs_get(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &device_hours,
                   sizeof(device_hours))) {
    device_hours = 0;
  }

  // Allocate parameter data
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, &device_hours,
                         sizeof(device_hours))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
#ifdef CONFIG_RDM_PID_DEVICE_HOURS_DISABLE_SET
      .pid_cc = RDM_CC_GET,
#else
      .pid_cc = RDM_CC_GET_SET,
#endif
      .ds = RDM_DS_UNSIGNED_DWORD,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "d$"},
      .set = {.handler = rdm_simple_response_handler,
              .request.format = "d$",
              .response.format = NULL},
      .pdl_size = sizeof(uint32_t),
      .max_value = UINT32_MAX,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_device_hours(dmx_port_t dmx_num, uint32_t *device_hours) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_hours != NULL, 0, "device_hours is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DEVICE_HOURS,
                            device_hours, sizeof(*device_hours));
}

bool rdm_set_device_hours(dmx_port_t dmx_num, uint32_t device_hours) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DEVICE_HOURS;
  if (!dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &device_hours,
                         sizeof(device_hours))) {
    return false;
  }
  rdm_queue_push(dmx_num, pid);

  return true;
}

bool rdm_register_lamp_hours(dmx_port_t dmx_num, rdm_callback_t cb,
                             void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_LAMP_HOURS;

  // Attempt to load the value from NVS
  uint32_t lamp_hours;
  if (!dmx_nvs_get(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &lamp_hours,
                   sizeof(lamp_hours))) {
    lamp_hours = 0;
  }

  // Allocate parameter data
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, &lamp_hours,
                         sizeof(lamp_hours))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET_SET,
      .ds = RDM_DS_UNSIGNED_DWORD,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "d$"},
      .set = {.handler = rdm_simple_response_handler,
              .request.format = "d$",
              .response.format = NULL},
      .pdl_size = sizeof(uint32_t),
      .max_value = UINT32_MAX,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_lamp_hours(dmx_port_t dmx_num, uint32_t *lamp_hours) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(lamp_hours != NULL, 0, "lamp_hours is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_LAMP_HOURS,
                            lamp_hours, sizeof(*lamp_hours));
}

bool rdm_set_lamp_hours(dmx_port_t dmx_num, uint32_t lamp_hours) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_LAMP_HOURS;
  if (!dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &lamp_hours,
                         sizeof(lamp_hours))) {
    return false;
  }
  rdm_queue_push(dmx_num, pid);

  return true;
}