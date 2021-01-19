#pragma once

#include "hal/uart_types.h"
#include "soc/uart_periph.h"

/**
 * @brief Gets the number of bits the UART remains idle after transmitting data.
 * 
 * @param hw Pointer to a UART struct.
 * @return The number of bits the UART is idle after transmitting data. 
 */
static inline uint16_t dmx_ll_get_idle_num(uart_dev_t *hw) {
    return hw->idle_conf.tx_idle_num;
}

/**
 * @brief Gets the number of bits the UART sends as break.
 * 
 * @param hw Pointer to a UART struct.
 * @return The number of bits the UART sends as a break after transmitting.
 */
static inline uint8_t dmx_ll_get_break_num(uart_dev_t *hw) {
    return hw->idle_conf.tx_brk_num;
}

/**
 * @brief Gets the UART rx timeout (unit: time it takes for one word to be sent at current baudrate).
 * 
 * @param hw Pointer to a UART struct.
 * @return The UART rx timeout.
 */
static inline uint8_t dmx_ll_get_rx_tout(uart_dev_t *hw) {
    return hw->conf1.rx_tout_en ? hw->conf1.rx_tout_thrhd : 0;
}

/**
 * @brief Inverts or uninverts tx line on the UART bus.
 * 
 * @param hw  Pointer to a UART struct.
 * @param invert 1 to invert, 0 to un-invert.
 */
static inline void dmx_ll_inverse_txd_signal(uart_dev_t *hw, int invert) {
    hw->conf0.txd_inv = invert ? 1 : 0;
}

/**
 * @brief Inverts or uninverts rts line on the UART bus.
 * 
 * @param hw  Pointer to a UART struct.
 * @param invert 1 to invert, 0 to un-invert.
 */
static inline void dmx_ll_inverse_rts_signal(uart_dev_t *hw, int invert) {
    hw->conf0.rts_inv = invert ? 1 : 0;
}