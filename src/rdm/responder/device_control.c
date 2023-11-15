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
  const rdm_pd_definition_t def = {
      .schema = {.data_type = RDM_DS_UNSIGNED_BYTE,
                 .cc = RDM_CC_GET_SET,
                 .pdl_size = sizeof(uint8_t),
                 .min_value = 0,
                 .max_value = 1,
                 .format = "b$"},
      .pd_size = sizeof(uint8_t),
      .nvs = false,
      .response_handler = rdm_response_handler_simple,
  };

  const uint8_t init_value = 0;
  rdm_pd_add_new(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &def, &init_value);
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}