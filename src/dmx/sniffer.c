#include "dmx/include/sniffer.h"

#include "dmx/hal/include/gpio.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"

bool dmx_sniffer_enable(dmx_port_t dmx_num, int intr_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_sniffer_pin_is_valid(intr_pin), false, "intr_pin error");
  DMX_CHECK(!dmx_sniffer_is_enabled(dmx_num), false,
            "sniffer is already enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Allocate the sniffer queue
  driver->sniffer.metadata_queue = xQueueCreate(1, sizeof(dmx_metadata_t));
  DMX_CHECK(driver->sniffer.metadata_queue != NULL, false,
            "DMX sniffer queue malloc error");

  // Set sniffer default values
  driver->sniffer.last_neg_edge_ts = -1;  // Negative edge hasn't been seen yet
  driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_MAB;

  // Add the GPIO interrupt handler
  return dmx_gpio_init(dmx_num, driver, intr_pin);
}

bool dmx_sniffer_disable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), false, "sniffer is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Disable the interrupt and remove the interrupt handler
  dmx_gpio_deinit(dmx_num);

  // Deallocate the sniffer queue
  vQueueDelete(driver->sniffer.metadata_queue);
  driver->sniffer.metadata_queue = NULL;

  return true;
}

bool dmx_sniffer_is_enabled(dmx_port_t dmx_num) {
  return dmx_driver_is_installed(dmx_num) &&
         dmx_driver[dmx_num]->sniffer.metadata_queue != NULL;
}

bool dmx_sniffer_get_data(dmx_port_t dmx_num, dmx_metadata_t *metadata,
                          TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(metadata, false, "metadata is null");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), false, "sniffer is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  return xQueueReceive(driver->sniffer.metadata_queue, metadata, wait_ticks);
}
