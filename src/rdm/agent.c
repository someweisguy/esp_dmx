#include "agent.h"

#include <string.h>

#include "dmx/driver.h"
#include "dmx/types.h"

static const char *TAG = "rdm_responder";  // The log tagline for the file.

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

uint16_t rdm_get_dmx_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  uint16_t dmx_start_address;
  taskENTER_CRITICAL(spinlock);
  dmx_start_address = driver->rdm.device_info.dmx_start_address;
  taskEXIT_CRITICAL(spinlock);

  return dmx_start_address;
}

void rdm_set_dmx_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  DMX_CHECK(dmx_start_address >= 1 && dmx_start_address <= 512, ,
            "dmx_start_address is invalid");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);
  driver->rdm.device_info.dmx_start_address = dmx_start_address;
  taskEXIT_CRITICAL(spinlock);
}
