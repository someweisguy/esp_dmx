#include "gpio.h"

#include "dmx/config.h"
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

dmx_gpio_handle_t dmx_gpio_init(dmx_port_t dmx_num, void *isr_handle,
                                void *isr_context, int sniffer_pin) {
  dmx_gpio_handle_t gpio = &dmx_gpio_context[dmx_num];
  gpio_set_intr_type(sniffer_pin, GPIO_INTR_ANYEDGE);
  gpio_isr_handler_add(sniffer_pin, isr_handle, isr_context);
  gpio->sniffer_pin = sniffer_pin;
  return gpio;
}

void dmx_gpio_deinit(dmx_gpio_handle_t gpio) {
  gpio_set_intr_type(gpio->sniffer_pin, GPIO_INTR_DISABLE);
  gpio_isr_handler_remove(gpio->sniffer_pin);
  gpio->sniffer_pin = -1;
}

int DMX_ISR_ATTR dmx_gpio_read(dmx_gpio_handle_t gpio) {
  return gpio_ll_get_level(GPIO_LL_GET_HW(GPIO_PORT_0), gpio->sniffer_pin);
}
