#include "hal/dmx_ll.h"

static inline uint16_t dmx_ll_get_idle_num(uart_dev_t *hw) {
    return hw->idle_conf.tx_idle_num;
}

static inline uint8_t dmx_ll_get_break_num(uart_dev_t *hw) {
    return hw->idle_conf.tx_brk_num;
}
