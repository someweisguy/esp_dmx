#include "rdm/responder/device_control.h"

#include "dmx/driver.h"
#include "dmx/struct.h"

bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(cb != NULL, false, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t pd_def = {.pid = RDM_PID_IDENTIFY_DEVICE,
                                        .pdl_size = sizeof(uint8_t),
                                        .data_type = RDM_DS_UNSIGNED_BYTE,
                                        .cc = RDM_CC_GET_SET,
                                        .unit = RDM_UNITS_NONE,
                                        .prefix = RDM_PREFIX_NONE,
                                        .min_value = 0,
                                        .max_value = 1,
                                        .default_value = 0,
                                        .description = "Identify Device"};
  const char *format = "b$";
  const bool nvs = false;

  uint8_t default_value = 0;
  rdm_pd_add_new(dmx_num, RDM_SUB_DEVICE_ROOT, &pd_def, format, nvs,
                 rdm_simple_response_cb, &default_value);

  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT,
                                RDM_PID_IDENTIFY_DEVICE, cb, context);
}