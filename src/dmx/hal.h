#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "hal/uart_hal.h"

/* Some of these functions are marked with IRAM_ATTR to place them in IRAM.
Any function here that is used in the DMX ISR must be placed in IRAM, or 
cache miss crashes will (randomly) occur. In ESP-IDF v4.4.1, HAL functions in
the UART ISR are either built in a separate source file that is specifically
placed in IRAM or the functions are #defines instead of actual functions. I'm
not a big fan of the "#defines as functions design pattern," so every function
defined in this file is an actual function. If the function is used in the DMX
ISR, it shall be declared an IRAM_ATTR. */

/**
 * @brief The the interrupt status mask from the UART.
 * 
 * @param hal Pointer to a UART HAL context.
 * 
 * @return The interrupt status mask. 
 */
 uint32_t dmx_hal_get_intsts_mask(uart_hal_context_t *hal) {
  return uart_hal_get_intsts_mask(hal);
}

/**
 * @brief Enables UART interrupts using an interrupt mask.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param mask The UART mask that is enabled.
 */
 void dmx_hal_ena_intr_mask(uart_hal_context_t *hal, uint32_t mask) {
  uart_hal_ena_intr_mask(hal, mask);
}

/**
 * @brief Disables UART interrupts using an interrupt mask.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param mask The UART mask that is disabled.
 */
 void dmx_hal_disable_intr_mask(uart_hal_context_t *hal, uint32_t mask) {
  uart_hal_disable_intr_mask(hal, mask);
}

/**
 * @brief Clears UART interrupts using a mask.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param mask The UART mask that is cleared.
 */
 void dmx_hal_clr_intsts_mask(uart_hal_context_t *hal, uint32_t mask) {
  uart_hal_clr_intsts_mask(hal, mask);
}

/**
 * @brief Get the current length of the bytes in the rx FIFO.
 * 
 * @param hal Context of the HAL layer
 * 
 * @return Number of bytes in the rx FIFO
 */
 IRAM_ATTR uint32_t dmx_hal_get_rxfifo_len(uart_hal_context_t *hal) {
  return uart_hal_get_rxfifo_len(hal);
}

/**
 * @brief Gets the number of bits the UART remains idle after transmitting data.
 * 
 * @param hal Pointer to a UART HAL context.
 * @return The number of bits the UART is idle after transmitting data. 
 */
 uint16_t dmx_hal_get_idle_num(uart_hal_context_t *hal) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  
  #elif defined(CONFIG_IDF_TARGET_ESP32S2)

  #elif defined(CONFIG_IDF_TARGET_ESP32S3)

  #elif defined(CONFIG_IDF_TARGET_ESP32C3)
  
  #endif
  return 5; // FIXME: no equivalent HAL function
}

/**
 * @brief Gets the number of bits the UART sends as break.
 * 
 * @param hal Pointer to a UART HAL context.
 * @return The number of bits the UART sends as a break after transmitting.
 */
 uint8_t dmx_hal_get_break_num(uart_hal_context_t *hal) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  
  #elif defined(CONFIG_IDF_TARGET_ESP32S2)

  #elif defined(CONFIG_IDF_TARGET_ESP32S3)

  #elif defined(CONFIG_IDF_TARGET_ESP32C3)
  
  #endif
  return 45; // FIXME: no equivalent HAL function
}

/**
 * @brief Gets the UART rx timeout (unit: time it takes for one word to be sent at current baud_rate).
 * 
 * @param hal Pointer to a UART HAL context.
 * @return The UART rx timeout.
 */
 uint8_t dmx_hal_get_rx_tout(uart_hal_context_t *hal) {
  return uart_hal_get_rx_tout_thr(hal);
}

/**
 * @brief Inverts or un-inverts lines on the UART bus using a mask.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param invert_mask Inversion mask.
 */
 void dmx_hal_inverse_signal(uart_hal_context_t *hal, uint32_t invert_mask) {
  uart_hal_inverse_signal(hal, invert_mask);
}

/**
 * @brief Gets the level of the rx line on the UART bus.
 * 
 * @param hal Pointer to a UART HAL context.
 * @return UART rx line level.
 */
 uint32_t dmx_hal_get_rx_level(uart_hal_context_t *hal) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  
  #elif defined(CONFIG_IDF_TARGET_ESP32S2)

  #elif defined(CONFIG_IDF_TARGET_ESP32S3)

  #elif defined(CONFIG_IDF_TARGET_ESP32C3)
  
  #endif
  return 0; // FIXME: doesn't have equivalent HAL function - used in RX timing tool
}

/**
 * @brief Read the first num characters from the rxfifo.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param buf Destination buffer to be read into
 * @param num The maximum number of characters to read. Set to 0 to read all data.
 * 
 * @return The number of characters read
 */
 IRAM_ATTR void dmx_hal_read_rxfifo(uart_hal_context_t *hal, uint8_t *buf, int num) {
  uart_hal_read_rxfifo(hal, buf, &num);
}

/**
 * @brief Enables or disables the UART RTS line.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param set 1 to enable the RTS line (set low), 0 to disable the RTS line (set high).
 */
 void dmx_hal_set_rts(uart_hal_context_t *hal, int set) {
  uart_hal_set_rts(hal, set);
}

/**
 * @brief Gets the enabled UART interrupt status.
 * 
 * @param hal Pointer to a UART HAL context.
 * @return Gets the enabled UART interrupt status.
 */
 uint32_t dmx_hal_get_intr_ena_status(uart_hal_context_t *hal){
  return uart_hal_get_intr_ena_status(hal);
}

/**
 * @brief Initializes the UART for DMX mode.
 * 
 * @param hal Pointer to a UART HAL context.
 */
 void dmx_hal_init(uart_hal_context_t *hal) {
  uart_ll_set_sclk(hal->dev, UART_SCLK_APB);
  uart_ll_set_baudrate(hal->dev, DMX_TYP_BAUD_RATE);
  uart_ll_set_mode(hal->dev, UART_MODE_RS485_HALF_DUPLEX);
  uart_ll_set_parity(hal->dev, UART_PARITY_DISABLE);
  uart_ll_set_data_bit_num(hal->dev, UART_DATA_8_BITS);
  uart_ll_set_stop_bits(hal->dev, UART_STOP_BITS_2);
  uart_ll_tx_break(hal->dev, 45); // 45 == 180us
  uart_ll_set_tx_idle_num(hal->dev, 5); // 20 == 20us
  uart_ll_set_hw_flow_ctrl(hal->dev, UART_HW_FLOWCTRL_DISABLE, 100);
}

/**
 * @brief Sets the baud rate for the UART.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param baud_rate The baud rate to use.
 */
 void dmx_hal_set_baudrate(uart_hal_context_t *hal, uint32_t baud_rate) {
  uart_hal_set_baudrate(hal, baud_rate);
}

/**
 * @brief Sets the number of mark bits to transmit after a break has been transmitted.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param idle_num The number of idle bits to transmit.
 */
 void dmx_hal_set_tx_idle_num(uart_hal_context_t *hal, uint16_t idle_num) {
  uart_hal_set_tx_idle_num(hal, idle_num);
}

/**
 * @brief Enables or disables transmitting UART hardware break.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param break_num The number of break bits to transmit when a break is transmitted.
 */
 void dmx_hal_set_tx_break_num(uart_hal_context_t *hal, uint8_t break_num) {
  uart_hal_tx_break(hal, break_num);
}

/**
 * @brief Get the UART baud rate of the selected UART hardware.
 * 
 * @param hal Pointer to a UART HAL context.
 * 
 * @return The baud rate of the UART hardware. 
 */
 IRAM_ATTR uint32_t dmx_hal_get_baudrate(uart_hal_context_t *hal) {
  uint32_t baud_rate;
  uart_hal_get_baudrate(hal, &baud_rate);
  return baud_rate;
}

/**
 * @brief Set the duration for the UART RX inactivity timeout that triggers the RX timeout interrupt.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param rx_timeout_thresh The RX timeout duration (unit: time of sending one byte).
 */
 IRAM_ATTR void dmx_hal_set_rx_timeout(uart_hal_context_t *hal, const uint8_t rx_timeout_thresh) {
  uart_hal_set_rx_timeout(hal, rx_timeout_thresh);
}

/**
 * @brief Sets the number of bytes that the UART must receive to trigger a RX FIFO full interrupt.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param rxfifo_full_thresh The number of bytes needed to trigger an RX FIFO full interrupt.
 */
 IRAM_ATTR void dmx_hal_set_rxfifo_full_thr(uart_hal_context_t *hal, uint8_t rxfifo_full_thresh) {
  uart_hal_set_rxfifo_full_thr(hal, rxfifo_full_thresh);
}

/**
 * @brief Sets the number of bytes that the UART TX FIFO must have remaining in it to trigger a TX FIFO empty interrupt.
 * 
 * @param hal Pointer to a UART HAL context.
 * @param threshold The number of bytes remaining to trigger a TX FIFO empty interrupt.
 */
 IRAM_ATTR void dmx_hal_set_txfifo_empty_thr(uart_hal_context_t *hal, uint8_t threshold) {
  uart_hal_set_txfifo_empty_thr(hal, threshold);
}

/**
 * @brief Resets the UART RX FIFO.
 * 
 * @param hal Pointer to a UART HAL context.
 */
 IRAM_ATTR void dmx_hal_rxfifo_rst(uart_hal_context_t *hal) {
  uart_hal_rxfifo_rst(hal);
}

/**
 * @brief Get the length of the UART TX FIFO.
 * 
 * @param hal Pointer to a UART HAL context.
 * @return The length of the UART TX FIFO. 
 */
 IRAM_ATTR uint32_t dmx_hal_get_txfifo_len(uart_hal_context_t *hal) {
  return uart_hal_get_txfifo_len(hal);
}

 IRAM_ATTR void dmx_hal_write_txfifo(uart_hal_context_t *hal, const uint8_t *buf, uint32_t data_size, uint32_t *write_size) {
  uart_hal_write_txfifo(hal, buf, data_size, write_size);
}

 IRAM_ATTR void dmx_hal_txfifo_rst(uart_hal_context_t *hal) {
  uart_hal_txfifo_rst(hal);
}

#ifdef __cplusplus
}
#endif
