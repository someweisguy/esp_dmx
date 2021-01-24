#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "hal/uart_hal.h"
#include "hal/uart_ll.h"
#include "hal/uart_types.h"
#include "soc/uart_periph.h"

/**
 * @brief Get the current length of th ebytes in the rx FIFO.
 * 
 * @param hal Context of the HAL layer
 * 
 * @return Number of bytes in the rx FIFO
 */
#define dmx_hal_get_rxfifo_len(hal) uart_ll_get_rxfifo_len((hal)->dev)

/**
 * @brief Gets the number of bits the UART remains idle after transmitting data.
 * 
 * @param hw Pointer to a UART struct.
 * @return The number of bits the UART is idle after transmitting data. 
 */
static inline uint16_t dmx_hal_get_idle_num(uart_hal_context_t *hal) {
    return hal->dev->idle_conf.tx_idle_num;
}

/**
 * @brief Gets the number of bits the UART sends as break.
 * 
 * @param hw Pointer to a UART struct.
 * @return The number of bits the UART sends as a break after transmitting.
 */
static inline uint8_t dmx_hal_get_break_num(uart_hal_context_t *hal) {
    return hal->dev->idle_conf.tx_brk_num;
}

/**
 * @brief Gets the UART rx timeout (unit: time it takes for one word to be sent at current baudrate).
 * 
 * @param hw Pointer to a UART struct.
 * @return The UART rx timeout.
 */
static inline uint8_t dmx_hal_get_rx_tout(uart_hal_context_t *hal) {
    return hal->dev->conf1.rx_tout_en ? hal->dev->conf1.rx_tout_thrhd : 0;
}

/**
 * @brief Inverts or uninverts tx line on the UART bus.
 * 
 * @param hw Pointer to a UART struct.
 * @param invert 1 to invert, 0 to un-invert.
 */
static inline void dmx_hal_inverse_txd_signal(uart_hal_context_t *hal, int invert) {
    hal->dev->conf0.txd_inv = invert ? 1 : 0;
}

/**
 * @brief Inverts or uninverts rts line on the UART bus.
 * 
 * @param hw Pointer to a UART struct.
 * @param invert 1 to invert, 0 to un-invert.
 */
static inline void dmx_hal_inverse_rts_signal(uart_hal_context_t *hal, int invert) {
    hal->dev->conf0.rts_inv = invert ? 1 : 0;
}

/**
 * @brief Gets the level of the rx line on the UART bus.
 * 
 * @param hw Pointer to a UART struct.
 * @return UART rx line level.
 */
static inline uint32_t dmx_hal_get_rx_level(uart_hal_context_t *hal) {
    return hal->dev->status.rxd;
}

/**
 * @brief Read the first num characters from the rxfifo.
 * 
 * @param hal Context of the HAL layer
 * @param buf Destination buffer to be read into
 * @param num The maximum number of characters to read
 * 
 * @return The number of characters read
 */
static inline int dmx_hal_readn_rxfifo(uart_hal_context_t *hal, uint8_t *buf, int num) {
    const uint16_t rxfifo_len = uart_ll_get_rxfifo_len(hal->dev);
    if (num > rxfifo_len) num = rxfifo_len;
    uart_ll_read_rxfifo(hal->dev, buf, num);
    return num;
}

#ifdef __cplusplus
}
#endif
