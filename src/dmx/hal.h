/**
 * @file dmx_hal.h
 * @author Mitch Weisbrod
 * @brief This file is the Hardware Abstraction Layer (HAL) of esp_dmx. It
 * contains low-level functions to perform tasks relating to the ESP32 UART
 * hardware. Many of these functions are labelled with IRAM_ATTR to place them
 * in Instruction RAM memory. This is done to every function that is called from
 * the DMX interrupt service routines. Functions labelled with IRAM_ATTR must
 * not call any functions that are not also labelled with IRAM_ATTR. Functions
 * called from IRAM HAL functions must be inlined if they are not also in IRAM.
 * Calling non-IRAM, non-inlined functions will cause the ESP32 to crash when
 * the DMX interrupt service routines are called while the ESP32 cache is also
 * disabled.
 */
#pragma once

#include "dmx/types.h"
#include "hal/uart_hal.h"

#ifdef CONFIG_DMX_ISR_IN_IRAM
#define DMX_ISR_ATTR IRAM_ATTR
#else
#define DMX_ISR_ATTR
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the UART for DMX.
 *
 * @param uart A pointer to a UART port.
 */
void dmx_uart_init(uart_dev_t *uart);

/**
 * @brief Gets the UART baud rate of the selected UART hardware.
 *
 * @param uart A pointer to a UART port.
 * @return The baud rate of the UART hardware.
 */
uint32_t dmx_uart_get_baud_rate(uart_dev_t *uart);

/**
 * @brief Sets the baud rate for the UART.
 *
 * @param uart A pointer to a UART port.
 * @param baud_rate The baud rate to use.
 */
void dmx_uart_set_baud_rate(uart_dev_t *uart, uint32_t baud_rate);

/**
 * @brief Sets the number of bytes that the UART must receive to trigger a RX
 * FIFO full interrupt.
 *
 * @param uart A pointer to a UART port.
 * @param threshold The number of bytes needed to trigger an RX FIFO full
 * interrupt.
 */
void dmx_uart_set_rxfifo_full(uart_dev_t *uart, uint8_t threshold);

/**
 * @brief Sets the number of bytes that the UART TX FIFO must have remaining in
 * it to trigger a TX FIFO empty interrupt.
 *
 * @param uart A pointer to a UART port.
 * @param threshold The number of bytes remaining to trigger a TX FIFO empty
 * interrupt.
 */
void dmx_uart_set_txfifo_empty(uart_dev_t *uart, uint8_t threshold);

/**
 * @brief Inverts or un-inverts the TX line on the UART.
 *
 * @param uart A pointer to a UART port.
 * @param invert_mask 1 to invert, 0 to un-invert.
 */
DMX_ISR_ATTR void dmx_uart_invert_tx(uart_dev_t *uart, uint32_t invert);

/**
 * @brief Gets the level of the UART RTS line.
 *
 * @param uart A pointer to a UART port.
 * @return 1 if the UART RTS line is enabled (set low; read), 0 if the UART RTS
 * line is disable (set high; write).
 */
int dmx_uart_get_rts(uart_dev_t *uart);

/**
 * @brief Gets the interrupt status mask from the UART.
 *
 * @param uart A pointer to a UART port.
 * @return The interrupt status mask.
 */
DMX_ISR_ATTR int dmx_uart_get_interrupt_status(uart_dev_t *uart);

/**
 * @brief Enables UART interrupts using an interrupt mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is enabled.
 */
DMX_ISR_ATTR void dmx_uart_enable_interrupt(uart_dev_t *uart, int mask);

/**
 * @brief Disables UART interrupts using an interrupt mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is disabled.
 */
DMX_ISR_ATTR void dmx_uart_disable_interrupt(uart_dev_t *uart, int mask);

/**
 * @brief Clears UART interrupts using a mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is cleared.
 */
DMX_ISR_ATTR void dmx_uart_clear_interrupt(uart_dev_t *uart, int mask);

/**
 * @brief Gets the current length of the bytes in the UART RX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @return The number of bytes in the UART RX FIFO.
 */
DMX_ISR_ATTR uint32_t dmx_uart_get_rxfifo_len(uart_dev_t *uart);

/**
 * @brief Gets the level of the UART RX line.
 *
 * @param uart A pointer to a UART port.
 * @return The UART RX line level.
 */

DMX_ISR_ATTR uint32_t dmx_uart_get_rx_level(uart_dev_t *uart);

/**
 * @brief Reads from the UART RX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @param buf Destination buffer to be read into.
 * @param num The maximum number of characters to read. Set to 0 to read all
 * data.
 * @return The number of characters read.
 */
DMX_ISR_ATTR void dmx_uart_read_rxfifo(uart_dev_t *uart, uint8_t *buf,
                                       int *size);

/**
 * @brief Enables or disables the UART RTS line.
 *
 * @param uart A pointer to a UART port.
 * @param set 1 to enable the UART RTS line (set low; read), 0 to disable the
 * UART RTS line (set high; write).
 */
DMX_ISR_ATTR void dmx_uart_set_rts(uart_dev_t *uart, int set);
/**
 * @brief Resets the UART RX FIFO.
 *
 * @param uart A pointer to a UART port.
 */
DMX_ISR_ATTR void dmx_uart_rxfifo_reset(uart_dev_t *uart);

/**
 * @brief Gets the length of the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @return The length of the UART TX FIFO.
 */
DMX_ISR_ATTR uint32_t dmx_uart_get_txfifo_len(uart_dev_t *uart);

/**
 * @brief Writes to the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @param buf The source buffer from which to write.
 * @param size The number of bytes to write.
 */
DMX_ISR_ATTR void dmx_uart_write_txfifo(uart_dev_t *uart, const void *buf,
                                        size_t *size);

/**
 * @brief Resets the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 */
DMX_ISR_ATTR void dmx_uart_txfifo_reset(uart_dev_t *uart);

#ifdef __cplusplus
}
#endif
