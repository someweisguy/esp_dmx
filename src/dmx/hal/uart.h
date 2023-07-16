/**
 * @file uart.h
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

#include "uart.h"
#include "hal/uart_hal.h"
#include "dmx/types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum dmx_interrupt_mask_t {
  DMX_INTR_RX_FIFO_OVERFLOW = UART_INTR_RXFIFO_OVF,
  DMX_INTR_RX_FRAMING_ERR = UART_INTR_PARITY_ERR | UART_INTR_FRAM_ERR,
  DMX_INTR_RX_ERR = DMX_INTR_RX_FIFO_OVERFLOW | DMX_INTR_RX_FRAMING_ERR,

  DMX_INTR_RX_BREAK = UART_INTR_BRK_DET,
  DMX_INTR_RX_DATA = UART_INTR_RXFIFO_FULL,
  DMX_INTR_RX_ALL = DMX_INTR_RX_DATA | DMX_INTR_RX_BREAK | DMX_INTR_RX_ERR,

  DMX_INTR_TX_DATA = UART_INTR_TXFIFO_EMPTY,
  DMX_INTR_TX_DONE = UART_INTR_TX_DONE,
  DMX_INTR_TX_ALL = DMX_INTR_TX_DATA | DMX_INTR_TX_DONE,

  DMX_ALL_INTR_MASK = -1
};

typedef struct dmx_uart_t dmx_uart_t;

/**
 * @brief Initializes the UART for DMX.
 *
 * @param uart A pointer to a UART port.
 */
dmx_uart_t *dmx_uart_init(dmx_port_t dmx_num, void *isr_handle,
                          void *isr_context, int isr_flags);

// TODO: docs
void dmx_uart_deinit(dmx_uart_t *uart);

/**
 * @brief Gets the UART baud rate of the selected UART hardware.
 *
 * @param uart A pointer to a UART port.
 * @return The baud rate of the UART hardware.
 */
uint32_t dmx_uart_get_baud_rate(dmx_uart_t *uart);

/**
 * @brief Sets the baud rate for the UART.
 *
 * @param uart A pointer to a UART port.
 * @param baud_rate The baud rate to use.
 */
void dmx_uart_set_baud_rate(dmx_uart_t *uart, uint32_t baud_rate);

/**
 * @brief Sets the number of bytes that the UART must receive to trigger a RX
 * FIFO full interrupt.
 *
 * @param uart A pointer to a UART port.
 * @param threshold The number of bytes needed to trigger an RX FIFO full
 * interrupt.
 */
void dmx_uart_set_rxfifo_full(dmx_uart_t *uart, uint8_t threshold);

/**
 * @brief Sets the number of bytes that the UART TX FIFO must have remaining in
 * it to trigger a TX FIFO empty interrupt.
 *
 * @param uart A pointer to a UART port.
 * @param threshold The number of bytes remaining to trigger a TX FIFO empty
 * interrupt.
 */
void dmx_uart_set_txfifo_empty(dmx_uart_t *uart, uint8_t threshold);

/**
 * @brief Inverts or un-inverts the TX line on the UART.
 *
 * @param uart A pointer to a UART port.
 * @param invert_mask 1 to invert, 0 to un-invert.
 */
void dmx_uart_invert_tx(dmx_uart_t *uart, uint32_t invert);

/**
 * @brief Gets the level of the UART RTS line.
 *
 * @param uart A pointer to a UART port.
 * @return 1 if the UART RTS line is enabled (set low; read), 0 if the UART RTS
 * line is disable (set high; write).
 */
int dmx_uart_get_rts(dmx_uart_t *uart);

/**
 * @brief Gets the interrupt status mask from the UART.
 *
 * @param uart A pointer to a UART port.
 * @return The interrupt status mask.
 */
int dmx_uart_get_interrupt_status(dmx_uart_t *uart);

/**
 * @brief Enables UART interrupts using an interrupt mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is enabled.
 */
void dmx_uart_enable_interrupt(dmx_uart_t *uart, int mask);

/**
 * @brief Disables UART interrupts using an interrupt mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is disabled.
 */
void dmx_uart_disable_interrupt(dmx_uart_t *uart, int mask);

/**
 * @brief Clears UART interrupts using a mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is cleared.
 */
void dmx_uart_clear_interrupt(dmx_uart_t *uart, int mask);

/**
 * @brief Gets the current length of the bytes in the UART RX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @return The number of bytes in the UART RX FIFO.
 */
uint32_t dmx_uart_get_rxfifo_len(dmx_uart_t *uart);

/**
 * @brief Gets the level of the UART RX line.
 *
 * @param uart A pointer to a UART port.
 * @return The UART RX line level.
 */

uint32_t dmx_uart_get_rx_level(dmx_uart_t *uart);

/**
 * @brief Reads from the UART RX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @param buf Destination buffer to be read into.
 * @param num The maximum number of characters to read. Set to 0 to read all
 * data.
 * @return The number of characters read.
 */
void dmx_uart_read_rxfifo(dmx_uart_t *uart, uint8_t *buf, int *size);

/**
 * @brief Enables or disables the UART RTS line.
 *
 * @param uart A pointer to a UART port.
 * @param set 1 to enable the UART RTS line (set low; read), 0 to disable the
 * UART RTS line (set high; write).
 */
void dmx_uart_set_rts(dmx_uart_t *uart, int set);
/**
 * @brief Resets the UART RX FIFO.
 *
 * @param uart A pointer to a UART port.
 */
void dmx_uart_rxfifo_reset(dmx_uart_t *uart);

/**
 * @brief Gets the length of the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @return The length of the UART TX FIFO.
 */
uint32_t dmx_uart_get_txfifo_len(dmx_uart_t *uart);

/**
 * @brief Writes to the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @param buf The source buffer from which to write.
 * @param size The number of bytes to write.
 */
void dmx_uart_write_txfifo(dmx_uart_t *uart, const void *buf, size_t *size);

/**
 * @brief Resets the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 */
void dmx_uart_txfifo_reset(dmx_uart_t *uart);

#ifdef __cplusplus
}
#endif
