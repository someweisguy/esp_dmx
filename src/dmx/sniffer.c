#include "dmx/sniffer.h"

#include "dmx/hal/include/gpio.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"

bool dmx_sniffer_enable(dmx_port_t dmx_num, int intr_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_sniffer_pin_is_valid(intr_pin), false, "intr_pin error");
  DMX_CHECK(!dmx_sniffer_is_enabled(dmx_num), false,
            "sniffer is already enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Set sniffer default values
  driver->sniffer.last_neg_edge_ts = -1;  // Negative edge hasn't been seen yet

  dmx_driver[dmx_num]->sniffer.is_enabled = true;

  // Add the GPIO interrupt handler
  return dmx_gpio_init(dmx_num, driver, intr_pin);
}

bool dmx_sniffer_disable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), false, "sniffer is not enabled");

  // Disable the interrupt and remove the interrupt handler
  dmx_gpio_deinit(dmx_num);

  dmx_driver[dmx_num]->sniffer.is_enabled = false;

  return true;
}

bool dmx_sniffer_is_enabled(dmx_port_t dmx_num) {
  return dmx_driver_is_installed(dmx_num) &&
         dmx_driver[dmx_num]->sniffer.is_enabled;
}

bool dmx_sniffer_get_data(dmx_port_t dmx_num, dmx_metadata_t *metadata) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(metadata, false, "metadata is null");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), false, "sniffer is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  *metadata = driver->sniffer.metadata[driver->sniffer.buffer_index];
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return true;
}
