#pragma once

#include "hal/dmx_ll.h"
#include "hal/uart_hal.h"
#include "hal/uart_ll.h"

/**
 * @brief Get the idle num (in bits).
 * 
 * @param  hal Context of the HAL layer
 *
 * @return UART idle num
 */
#define dmx_hal_get_idle_num(hal) dmx_ll_get_idle_num((hal)->dev)

/**
 * @brief Get the break num (in bits).
 * 
 * @param  hal Context of the HAL layer
 *
 * @return UART break num
 */
#define dmx_hal_get_break_num(hal) dmx_ll_get_break_num((hal)->dev)

/**
 * @brief Read the first num characters from the rxfifo.
 * 
 * @param hal Context of the HAL layer
 * @param buf Destination buffer to be read into
 * @param num The maximum number of characters to read
 * 
 * @return The number of characters read
 */
int dmx_hal_readn_rxfifo(uart_hal_context_t *hal, uint8_t *buf, int num);

/**
 * @brief Get the current length of the bytes in the rxfifo.
 * 
 * @param hal Context of the HAL layer
 * 
 * @return The number of bytes in the rx fifo
 */
#define dmx_hal_get_rxfifo_len(hal) uart_ll_get_rxfifo_len((hal)->dev)

/**
 * @brief Get the RX timeout.
 * 
 * @param hal Context of the HAL layer
 * 
 * @return The RX timeout in units of time corresponding to the length of one word to be sent
 */
#define dmx_hal_get_rx_tout(hal) dmx_ll_get_rx_tout((hal)->dev)

/**
 * @brief Inverts or uninverts lines on the UART
 * 
 * @param hal Context of the HAL layer
 * @param inv_mask Inversion mask
 * 
 */
#define dmx_hal_inverse_signal(hal, inv_mask) dmx_ll_inverse_signal((hal)->dev, inv_mask)