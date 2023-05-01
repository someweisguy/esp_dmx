#include "responder.h"

#include "dmx/driver.h"
#include "esp_dmx.h"

static const char *TAG = "rdm_responder";

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

bool rdm_register_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                           rdm_response_cb_t callback, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // TODO: take mutex

  // Iterate the callback list to see if a callback with this PID exists
  int i = 0;
  for (; i < driver->rdm.num_callbacks; ++i) {
    if (driver->rdm.cbs[i].pid == pid) break;
  }

  // Check if there is space for callbacks
  if (i >= 16) {  // TODO: replace 16 with macro configurable in menuconfig
    ESP_LOGE(TAG, "No more space for RDM callbacks");
    return false;
  }
  
  // Add the requested callback to the callback list
  taskENTER_CRITICAL(spinlock);
  driver->rdm.cbs[i].pid = pid;
  driver->rdm.cbs[i].cb = callback;
  driver->rdm.cbs[i].context = context;
  ++driver->rdm.num_callbacks;
  taskEXIT_CRITICAL(spinlock);

  // TODO: give mutex

  return true;
}