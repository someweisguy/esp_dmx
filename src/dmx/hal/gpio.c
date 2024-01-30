#include "include/gpio.h"

#include "dmx/hal/include/timer.h"
#include "dmx/hal/include/uart.h"
#include "dmx/include/service.h"
#include "hal/gpio_hal.h"

struct dmx_gpio_t {
  int sniffer_pin;
} dmx_gpio_context[DMX_NUM_MAX] = {
    {-1},
    {-1},
#if DMX_NUM_MAX > 2
    {-1},
#endif
};

static void DMX_ISR_ATTR dmx_gpio_isr(void *arg) {
  const int64_t now = dmx_timer_get_micros_since_boot();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  const dmx_port_t dmx_num = driver->dmx_num;

  if (dmx_gpio_read(dmx_num)) {
    /* If this ISR is called on a positive edge and the current DMX frame is in
    a break and a negative edge timestamp has been recorded then a break has
    just finished. Therefore the DMX break length is able to be recorded. It can
    also be deduced that the driver is now in a DMX mark-after-break. */

    if (driver->dmx.progress == DMX_PROGRESS_IN_BREAK &&
        driver->sniffer.last_neg_edge_ts > -1) {
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->sniffer.buffer_index = !driver->sniffer.buffer_index;
      driver->sniffer.metadata[driver->sniffer.buffer_index].break_len =
          now - driver->sniffer.last_neg_edge_ts;
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->dmx.progress = DMX_PROGRESS_IN_MAB;
    }
    driver->sniffer.last_pos_edge_ts = now;
  } else {
    /* If this ISR is called on a negative edge in a DMX mark-after-break then
    the DMX mark-after-break has just finished. It can be recorded. Sniffer data
    is now available to be read by the user. */

    if (driver->dmx.progress == DMX_PROGRESS_IN_MAB) {
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->sniffer.metadata[driver->sniffer.buffer_index].mab_len =
          now - driver->sniffer.last_pos_edge_ts;
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->dmx.progress = DMX_PROGRESS_IN_DATA;
    }
    driver->sniffer.last_neg_edge_ts = now;
  }
}

bool dmx_gpio_init(dmx_port_t dmx_num, void *isr_context, int sniffer_pin) {
  struct dmx_gpio_t *gpio = &dmx_gpio_context[dmx_num];
  gpio_set_intr_type(sniffer_pin, GPIO_INTR_ANYEDGE);
  gpio_isr_handler_add(sniffer_pin, dmx_gpio_isr, isr_context);
  gpio->sniffer_pin = sniffer_pin;
  return true;
}

void dmx_gpio_deinit(dmx_port_t dmx_num) {
  struct dmx_gpio_t *gpio = &dmx_gpio_context[dmx_num];
  gpio_set_intr_type(gpio->sniffer_pin, GPIO_INTR_DISABLE);
  gpio_isr_handler_remove(gpio->sniffer_pin);
  gpio->sniffer_pin = -1;
}

int DMX_ISR_ATTR dmx_gpio_read(dmx_port_t dmx_num) {
  struct dmx_gpio_t *gpio = &dmx_gpio_context[dmx_num];
  return gpio_ll_get_level(GPIO_LL_GET_HW(GPIO_PORT_0), gpio->sniffer_pin);
}
