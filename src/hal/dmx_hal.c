#include "hal/dmx_hal.h"

int dmx_hal_readn_rxfifo(uart_hal_context_t *hal, uint8_t *buf, int num) {
    const uint16_t rxfifo_len = uart_ll_get_rxfifo_len(hal->dev);
    if (num > rxfifo_len) num = rxfifo_len;
    uart_ll_read_rxfifo(hal->dev, buf, num);
    return num;
}