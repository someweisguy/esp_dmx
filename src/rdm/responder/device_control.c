#include "rdm/responder/device_control.h"

#include "dmx/driver.h"
#include "dmx/struct.h"

bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(cb != NULL, false, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = sizeof(uint8_t),
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
  rdm_pd_set_definition(dmx_num, pid, &definition);

  // Allocate parameter data
  const bool nvs = true;
  const uint8_t init_value = 0;
  if (rdm_pd_add_variable(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, &init_value,
                          sizeof(init_value)) == NULL) {
    return false;
  }

  return rdm_pd_set_callback(dmx_num, pid, cb, context);
}