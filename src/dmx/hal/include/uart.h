/**
 * @file dmx/hal/include/uart.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file is the UART Hardware Abstraction Layer (HAL) of esp_dmx. It
 * contains low-level functions to perform tasks relating to the UART hardware.
 * The UART is used to send and receive DMX and RDM packets. DMX and RDM are
 * generated using a standard UART port and transmitted using the RS-485
 * electrical standard. This file is not considered part of the API and should
 * not be included by the user.
 */
#pragma once

#include "dmx/include/types.h"
#include "hal/uart_hal.h"

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
};

/**
 * @brief Initializes the UART for DMX.
 *
 * @param[inout] isr_context Context to be used in the DMX UART ISR.
 * @return A handle to the DMX UART or NULL on failure.
 */
bool dmx_uart_init(dmx_port_t dmx_num, void *isr_context, int isr_flags);

/**
 * @brief De-initializes the UART.
 *
 * @param uart A handle to the DMX UART.
 */
void dmx_uart_deinit(dmx_port_t dmx_num);

/**
 * @brief Sets the pins to be used by the DMX UART.
 *
 * @param uart A handle to the DMX UART.
 * @param tx The transmit pin number to use.
 * @param rx The receive pin number to use.
 * @param rts The RTS pin number to use.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_uart_set_pin(dmx_port_t dmx_num, int tx, int rx, int rts);

/**
 * @brief Gets the UART baud rate of the selected UART hardware.
 *
 * @param uart A handle to the DMX UART.
 * @return The baud rate of the UART hardware.
 */
uint32_t dmx_uart_get_baud_rate(dmx_port_t dmx_num);

/**
 * @brief Sets the baud rate for the UART.
 *
 * @param uart A handle to the DMX UART.
 * @param baud_rate The baud rate to use.
 */
void dmx_uart_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate);

/**
 * @brief Inverts or un-inverts the TX line on the UART.
 *
 * @param uart A handle to the DMX UART.
 * @param invert_mask 1 to invert, 0 to un-invert.
 */
void dmx_uart_invert_tx(dmx_port_t dmx_num, int invert);

/**
 * @brief Gets the level of the UART RTS line.
 *
 * @param uart A handle to the DMX UART.
 * @return 1 if the UART RTS line is enabled (set low; read), 0 if the UART RTS
 * line is disable (set high; write).
 */
int dmx_uart_get_rts(dmx_port_t dmx_num);

/**
 * @brief Gets the interrupt status mask from the UART.
 *
 * @param uart A handle to the DMX UART.
 * @return The interrupt status mask.
 */
int dmx_uart_get_interrupt_status(dmx_port_t dmx_num);

/**
 * @brief Enables UART interrupts using an interrupt mask.
 *
 * @param uart A handle to the DMX UART.
 * @param mask The UART mask that is enabled.
 */
void dmx_uart_enable_interrupt(dmx_port_t dmx_num, int mask);

/**
 * @brief Disables UART interrupts using an interrupt mask.
 *
 * @param uart A handle to the DMX UART.
 * @param mask The UART mask that is disabled.
 */
void dmx_uart_disable_interrupt(dmx_port_t dmx_num, int mask);

/**
 * @brief Clears UART interrupts using a mask.
 *
 * @param uart A handle to the DMX UART.
 * @param mask The UART mask that is cleared.
 */
void dmx_uart_clear_interrupt(dmx_port_t dmx_num, int mask);

/**
 * @brief Gets the current length of the bytes in the UART RX FIFO.
 *
 * @param uart A handle to the DMX UART.
 * @return The number of bytes in the UART RX FIFO.
 */
uint32_t dmx_uart_get_rxfifo_len(dmx_port_t dmx_num);

/**
 * @brief Reads from the UART RX FIFO.
 *
 * @param uart A handle to the DMX UART.
 * @param[out] buf Destination buffer to be read into.
 * @param[inout] num The maximum number of characters to read. Set to 0 to read
 * all data. Is set to the number of characters read.
 */
void dmx_uart_read_rxfifo(dmx_port_t dmx_num, uint8_t *buf, int *size);

/**
 * @brief Enables or disables the UART RTS line.
 *
 * @param uart A handle to the DMX UART.
 * @param set 1 to enable the UART RTS line (set low; read), 0 to disable the
 * UART RTS line (set high; write).
 */
void dmx_uart_set_rts(dmx_port_t dmx_num, int set);

/**
 * @brief Resets the UART RX FIFO.
 *
 * @param uart A handle to the DMX UART.
 */
void dmx_uart_rxfifo_reset(dmx_port_t dmx_num);

/**
 * @brief Gets the length of the UART TX FIFO.
 *
 * @param uart A handle to the DMX UART.
 * @return The length of the UART TX FIFO.
 */
uint32_t dmx_uart_get_txfifo_len(dmx_port_t dmx_num);

/**
 * @brief Writes to the UART TX FIFO.
 *
 * @param uart A handle to the DMX UART.
 * @param[out] buf The source buffer from which to write.
 * @param[inout] size The number of bytes to write. Is set to the number of
 * bytes written.
 */
void dmx_uart_write_txfifo(dmx_port_t dmx_num, const void *buf, int *size);

/**
 * @brief Resets the UART TX FIFO.
 *
 * @param uart A handle to the DMX UART.
 */
void dmx_uart_txfifo_reset(dmx_port_t dmx_num);

#ifdef __cplusplus
}
#endif
