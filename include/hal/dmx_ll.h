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
 * @brief Inverts or uninverts lines on the UART bus.
 * 
 * @param hw  Pointer to a UART struct.
 * @param inv_mask Invert mask.
 */
static inline void dmx_ll_inverse_signal(uart_dev_t *hw, uint32_t inv_mask)
{
    typeof(hw->conf0) conf0_reg = hw->conf0;
    conf0_reg.irda_tx_inv = (inv_mask & UART_SIGNAL_IRDA_TX_INV) ? 1 : 0;
    conf0_reg.irda_rx_inv = (inv_mask & UART_SIGNAL_IRDA_RX_INV) ? 1 : 0;
    conf0_reg.rxd_inv = (inv_mask & UART_SIGNAL_RXD_INV) ? 1 : 0;
    conf0_reg.cts_inv = (inv_mask & UART_SIGNAL_CTS_INV) ? 1 : 0;
    conf0_reg.dsr_inv = (inv_mask & UART_SIGNAL_DSR_INV) ? 1 : 0;
    conf0_reg.txd_inv = (inv_mask & UART_SIGNAL_TXD_INV) ? 1 : 0;
    conf0_reg.rts_inv = (inv_mask & UART_SIGNAL_RTS_INV) ? 1 : 0;
    conf0_reg.dtr_inv = (inv_mask & UART_SIGNAL_DTR_INV) ? 1 : 0;
    hw->conf0.val = conf0_reg.val;
}