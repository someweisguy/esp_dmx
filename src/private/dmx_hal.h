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

#include "dmx_types.h"
#include "driver.h"
#include "hal/uart_hal.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_private/esp_clk.h"
#endif

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
void dmx_uart_init(uart_dev_t *uart) {
  // Configure the UART for DMX output
  uart_ll_set_sclk(uart, UART_SCLK_APB);
#if ESP_IDF_VERSION_MAJOR >= 5
  uart_ll_set_baudrate(uart, DMX_BAUD_RATE, esp_clk_apb_freq());
#else
  uart_ll_set_baudrate(uart, DMX_BAUD_RATE);
#endif
  uart_ll_set_mode(uart, UART_MODE_UART);
  uart_ll_set_parity(uart, UART_PARITY_DISABLE);
  uart_ll_set_data_bit_num(uart, UART_DATA_8_BITS);
  uart_ll_set_stop_bits(uart, UART_STOP_BITS_2);
  uart_ll_tx_break(uart, 0);
  uart_ll_set_tx_idle_num(uart, 0);
  uart_ll_set_hw_flow_ctrl(uart, UART_HW_FLOWCTRL_DISABLE, 0);
}

/**
 * @brief Gets the UART baud rate of the selected UART hardware.
 *
 * @param uart A pointer to a UART port.
 * @return The baud rate of the UART hardware.
 */
uint32_t dmx_uart_get_baud_rate(uart_dev_t *uart) {
#if ESP_IDF_VERSION_MAJOR >= 5
  return uart_ll_get_baudrate(uart, esp_clk_apb_freq());
#else
  return uart_ll_get_baudrate(uart);
#endif
}

/**
 * @brief Sets the baud rate for the UART.
 *
 * @param uart A pointer to a UART port.
 * @param baud_rate The baud rate to use.
 */
void dmx_uart_set_baud_rate(uart_dev_t *uart, uint32_t baud_rate) {
#if ESP_IDF_VERSION_MAJOR >= 5
  uart_ll_set_baudrate(uart, baud_rate, esp_clk_apb_freq());
#else
  uart_ll_set_baudrate(uart, baud_rate);
#endif
}

/**
 * @brief Sets the number of bytes that the UART must receive to trigger a RX
 * FIFO full interrupt.
 *
 * @param uart A pointer to a UART port.
 * @param threshold The number of bytes needed to trigger an RX FIFO full
 * interrupt.
 */
void dmx_uart_set_rxfifo_full(uart_dev_t *uart, uint8_t threshold) {
  uart_ll_set_rxfifo_full_thr(uart, threshold);
}

/**
 * @brief Sets the number of bytes that the UART TX FIFO must have remaining in
 * it to trigger a TX FIFO empty interrupt.
 *
 * @param uart A pointer to a UART port.
 * @param threshold The number of bytes remaining to trigger a TX FIFO empty
 * interrupt.
 */
void dmx_uart_set_txfifo_empty(uart_dev_t *uart, uint8_t threshold) {
  uart_ll_set_txfifo_empty_thr(uart, threshold);
}

/**
 * @brief Inverts or un-inverts the TX line on the UART.
 *
 * @param uart A pointer to a UART port.
 * @param invert_mask 1 to invert, 0 to un-invert.
 */
DMX_ISR_ATTR void dmx_uart_invert_tx(uart_dev_t *uart, uint32_t invert) {
  uart->conf0.txd_inv = invert;
}

/**
 * @brief Gets the level of the UART RTS line.
 *
 * @param uart A pointer to a UART port.
 * @return 1 if the UART RTS line is enabled (set low; read), 0 if the UART RTS
 * line is disable (set high; write).
 */
int dmx_uart_get_rts(uart_dev_t *uart) {
  return uart->conf0.sw_rts;
}

/**
 * @brief Gets the interrupt status mask from the UART.
 *
 * @param uart A pointer to a UART port.
 * @return The interrupt status mask.
 */
DMX_ISR_ATTR int dmx_uart_get_interrupt_status(uart_dev_t *uart) {
  return uart_ll_get_intsts_mask(uart);
}

/**
 * @brief Enables UART interrupts using an interrupt mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is enabled.
 */
DMX_ISR_ATTR void dmx_uart_enable_interrupt(uart_dev_t *uart, int mask) {
  uart_ll_ena_intr_mask(uart, mask);
}

/**
 * @brief Disables UART interrupts using an interrupt mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is disabled.
 */
DMX_ISR_ATTR void dmx_uart_disable_interrupt(uart_dev_t *uart, int mask) {
  uart_ll_disable_intr_mask(uart, mask);
}

/**
 * @brief Clears UART interrupts using a mask.
 *
 * @param uart A pointer to a UART port.
 * @param mask The UART mask that is cleared.
 */
DMX_ISR_ATTR void dmx_uart_clear_interrupt(uart_dev_t *uart, int mask) {
  uart_ll_clr_intsts_mask(uart, mask);
}

/**
 * @brief Gets the current length of the bytes in the UART RX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @return The number of bytes in the UART RX FIFO.
 */
DMX_ISR_ATTR uint32_t dmx_uart_get_rxfifo_len(uart_dev_t *uart) {
  return uart_ll_get_rxfifo_len(uart);
}

/**
 * @brief Gets the level of the UART RX line.
 *
 * @param uart A pointer to a UART port.
 * @return The UART RX line level.
 */

DMX_ISR_ATTR uint32_t dmx_uart_get_rx_level(uart_dev_t *uart) {
  return uart->status.rxd;
}

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
                                       int *size) {
  const size_t rxfifo_len = uart_ll_get_rxfifo_len(uart);
  if (*size > rxfifo_len) {
    *size = rxfifo_len;
  }
  uart_ll_read_rxfifo(uart, buf, *size);
}

/**
 * @brief Enables or disables the UART RTS line.
 *
 * @param uart A pointer to a UART port.
 * @param set 1 to enable the UART RTS line (set low; read), 0 to disable the
 * UART RTS line (set high; write).
 */
DMX_ISR_ATTR void dmx_uart_set_rts(uart_dev_t *uart, int set) {
  uart_ll_set_rts_active_level(uart, set);
}

/**
 * @brief Resets the UART RX FIFO.
 *
 * @param uart A pointer to a UART port.
 */
DMX_ISR_ATTR void dmx_uart_rxfifo_reset(uart_dev_t *uart) {
  uart_ll_rxfifo_rst(uart);
}

/**
 * @brief Gets the length of the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @return The length of the UART TX FIFO.
 */
DMX_ISR_ATTR uint32_t dmx_uart_get_txfifo_len(uart_dev_t *uart) {
  return uart_ll_get_txfifo_len(uart);
}

/**
 * @brief Writes to the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 * @param buf The source buffer from which to write.
 * @param size The number of bytes to write.
 */
DMX_ISR_ATTR void dmx_uart_write_txfifo(uart_dev_t *uart, const void *buf,
                                        size_t *size) {
  const size_t txfifo_len = uart_ll_get_txfifo_len(uart);
  if (*size > txfifo_len) *size = txfifo_len;
  uart_ll_write_txfifo(uart, (uint8_t *)buf, *size);
}

/**
 * @brief Resets the UART TX FIFO.
 *
 * @param uart A pointer to a UART port.
 */
DMX_ISR_ATTR void dmx_uart_txfifo_reset(uart_dev_t *uart) {
  uart_ll_txfifo_rst(uart);
}

#ifdef __cplusplus
}
#endif
