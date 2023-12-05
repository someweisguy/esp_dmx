#include "parameters.h"

#include "dmx/device.h"
#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"


size_t rdm_get_device_label(dmx_port_t dmx_num, char *label, size_t label_len) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const char *rdm_label = rdm_pd_get_ptr(dmx_num, RDM_PID_DEVICE_LABEL, 0);
  DMX_CHECK(rdm_label != NULL, 0, "RDM_PID_DEVICE_LABEL not found");

  const size_t rdm_label_len = strnlen(rdm_label, 32);  // length without '\0'
  const size_t size = label_len < rdm_label_len ? label_len : rdm_label_len;
  strncpy(label, rdm_label, size);

  return size;
}