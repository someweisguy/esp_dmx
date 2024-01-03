#include "rdm/responder/include/device_control.h"

#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/responder/include/utils.h"

bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(cb != NULL, false, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;

  // Allocate parameter data
  const bool nvs = true;
  const uint8_t init_value = 0;
  if (!dmx_parameter_add_dynamic(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs,
                                 &init_value, sizeof(init_value))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET_SET,
      .ds = RDM_DS_UNSIGNED_BYTE,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "b$"},
      .set = {.handler = rdm_simple_response_handler,
              .request.format = "b$",
              .response.format = NULL},
      .pdl_size = sizeof(uint8_t),
      .max_value = 1,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_identify_device(dmx_port_t dmx_num, bool *identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify != NULL, 0, "identify is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT,
                            RDM_PID_IDENTIFY_DEVICE, identify, sizeof(uint8_t));
}

bool rdm_set_identify_device(dmx_port_t dmx_num, const bool identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify == 0 || identify == 1, 0, "identify error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;
  if (!dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &identify,
                         sizeof(uint8_t))) {
    return false;
  }
  rdm_queue_push(dmx_num, pid);

  return true;
}