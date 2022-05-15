#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "hal/uart_hal.h"

/**
 * @brief The the interrupt status mask from the UART.
 * 
 * @param dev Pointer to a UART struct.
 * 
 * @return The interrupt status mask. 
 */
static inline uint32_t dmx_hal_get_intsts_mask(uart_hal_context_t *hal) {
  return uart_hal_get_intsts_mask(hal);
}

/**
 * @brief Enables UART interrupts using an interrupt mask.
 * 
 * @param dev Pointer to a UART struct.
 * @param mask The UART mask that is enabled.
 */
static inline void dmx_hal_ena_intr_mask(uart_hal_context_t *hal, uint32_t mask) {
  uart_hal_ena_intr_mask(hal, mask);
}

/**
 * @brief Disables UART interrupts using an interrupt mask.
 * 
 * @param dev Pointer to a UART struct.
 * @param mask The UART mask that is disabled.
 */
static inline void dmx_hal_disable_intr_mask(uart_hal_context_t *hal, uint32_t mask) {
  uart_hal_disable_intr_mask(hal, mask);
}

/**
 * @brief Clears UART interrupts using a mask.
 * 
 * @param dev Pointer to a UART struct.
 * @param mask The UART mask that is cleared.
 */
static inline void dmx_hal_clr_intsts_mask(uart_hal_context_t *hal, uint32_t mask) {
  uart_hal_clr_intsts_mask(hal, mask);
}

/**
 * @brief Get the current length of the bytes in the rx FIFO.
 * 
 * @param hal Context of the HAL layer
 * 
 * @return Number of bytes in the rx FIFO
 */
static inline IRAM_ATTR uint32_t dmx_hal_get_rxfifo_len(uart_hal_context_t *hal) {
  return uart_hal_get_rxfifo_len(hal);
}

/**
 * @brief Gets the number of bits the UART remains idle after transmitting data.
 * 
 * @param dev Pointer to a UART struct.
 * @return The number of bits the UART is idle after transmitting data. 
 */
static inline uint16_t dmx_hal_get_idle_num(uart_hal_context_t *hal) {
  return 0; // TODO: no equivalent HAL function
}

/**
 * @brief Gets the number of bits the UART sends as break.
 * 
 * @param dev Pointer to a UART struct.
 * @return The number of bits the UART sends as a break after transmitting.
 */
static inline uint8_t dmx_hal_get_break_num(uart_hal_context_t *hal) {
  return 0; // TODO: no equivalent HAL function
}

/**
 * @brief Gets the UART rx timeout (unit: time it takes for one word to be sent at current baud_rate).
 * 
 * @param dev Pointer to a UART struct.
 * @return The UART rx timeout.
 */
static inline uint8_t dmx_hal_get_rx_tout(uart_hal_context_t *hal) {
  return uart_hal_get_rx_tout_thr(hal);
}

/**
 * @brief Inverts or uninverts lines on the UART bus using a mask.
 * 
 * @param dev Pointer to a UART struct.
 * @param invert_mask Inversion mask.
 */
static inline void dmx_hal_inverse_signal(uart_hal_context_t *hal, uint32_t invert_mask) {
  // TODO: library previously had invert tx and rts signals separately - combine them
}

/**
 * @brief Gets the level of the rx line on the UART bus.
 * 
 * @param dev Pointer to a UART struct.
 * @return UART rx line level.
 */
static inline uint32_t dmx_hal_get_rx_level(uart_hal_context_t *hal) {
  return 0; // TODO: doesn't have equivalent HAL function
}

/**
 * @brief Read the first num characters from the rxfifo.
 * 
 * @param dev Pointer to a UART struct.
 * @param buf Destination buffer to be read into
 * @param num The maximum number of characters to read. Set to 0 to read all data.
 * 
 * @return The number of characters read
 */
static inline IRAM_ATTR int dmx_hal_readn_rxfifo(uart_hal_context_t *hal, uint8_t *buf, int num) {
  uart_hal_read_rxfifo(hal, buf, &num);
  return num;
}

/**
 * @brief Enables or disables the UART RTS line.
 * 
 * @param dev Pointer to a UART struct.
 * @param set 1 to enable the RTS line (set low), 0 to disable the RTS line (set high).
 */
static inline void dmx_hal_set_rts(uart_hal_context_t *hal, int set) {
  return; // TODO: no equivalent HAL function
}

/**
 * @brief Gets the enabled UART interrupt status.
 * 
 * @param dev Pointer to a UART struct.
 * @return Gets the enabled UART interrupt status.
 */
static inline uint32_t dmx_hal_get_intr_ena_status(uart_hal_context_t *hal){
  return uart_hal_get_intr_ena_status(hal);
}

/**
 * @brief Initializes the UART for DMX mode.
 * 
 * @param dev Pointer to a UART struct.
 * @param dmx_num The UART number to initialize.
 */
static inline void dmx_hal_init(uart_hal_context_t *hal, dmx_port_t dmx_num) {
  // Set default clock source
  uart_ll_set_sclk(hal->dev, UART_SCLK_APB);
  // Set default baud: 250000, use APB clock.
  const uint32_t baud_def = 250000;
  uart_ll_set_baudrate(hal->dev, baud_def);
  // Set RS485 Half Duplex mode.
  uart_ll_set_mode(hal->dev, UART_MODE_RS485_HALF_DUPLEX);
  // Disable UART parity
  uart_ll_set_parity(hal->dev, UART_PARITY_DISABLE);
  // 8-bit world
  uart_ll_set_data_bit_num(hal->dev, UART_DATA_8_BITS);
  // 1-bit stop bit
  uart_ll_set_stop_bits(hal->dev, UART_STOP_BITS_2);
  // TODO: set break num?
  // Set tx idle
  uart_ll_set_tx_idle_num(hal->dev, 5);
  // Disable hw-flow control
  uart_ll_set_hw_flow_ctrl(hal->dev, UART_HW_FLOWCTRL_DISABLE, 100);
}

/**
 * @brief Sets the baud rate for the UART.
 * 
 * @param dev Pointer to a UART struct.
 * @param baud_rate The baud rate to use.
 */
static inline void dmx_hal_set_baudrate(uart_hal_context_t *hal, uint32_t baud_rate) {
  uart_hal_set_baudrate(hal, baud_rate);
}

/**
 * @brief Sets the number of mark bits to transmit after a break has been transmitted.
 * 
 * @param dev Pointer to a UART struct.
 * @param idle_num The number of idle bits to transmit.
 */
static inline void dmx_hal_set_tx_idle_num(uart_hal_context_t *hal, uint16_t idle_num) {
  return; // TODO: no equivalent HAL function
}

/**
 * @brief Enables or disables transmitting UART hardware break.
 * 
 * @param dev Pointer to a UART struct.
 * @param break_num The number of break bits to transmit when a break is transmitted.
 */
static inline void dmx_hal_set_tx_break_num(uart_hal_context_t *hal, uint8_t break_num) {
  uart_hal_tx_break(hal, break_num);
}

/**
 * @brief Get the UART baud rate of the selected UART hardware.
 * 
 * @param dev Pointer to a UART struct.
 * 
 * @return The baud rate of the UART hardware. 
 */
static inline IRAM_ATTR uint32_t dmx_hal_get_baudrate(uart_hal_context_t *hal) {
  uint32_t baud_rate;
  uart_hal_get_baudrate(hal, &baud_rate);
  return baud_rate;
}

/**
 * @brief Set the duration for the UART RX inactivity timeout that triggers the RX timeout interrupt.
 * 
 * @param dev Pointer to a UART struct.
 * @param rx_timeout_thresh The RX timeout duration (unit: time of sending one byte).
 */
static inline IRAM_ATTR void dmx_hal_set_rx_timeout(uart_hal_context_t *hal, const uint8_t rx_timeout_thresh) {
  uart_hal_set_rx_timeout(hal, rx_timeout_thresh);
}

/**
 * @brief Sets the number of bytes that the UART must receive to trigger a RX FIFO full interrupt.
 * 
 * @param dev Pointer to a UART struct.
 * @param rxfifo_full_thresh The number of bytes needed to trigger an RX FIFO full interrupt.
 */
static inline IRAM_ATTR void dmx_hal_set_rxfifo_full_thr(uart_hal_context_t *hal, uint8_t rxfifo_full_thresh) {
  uart_hal_set_rxfifo_full_thr(hal, rxfifo_full_thresh);
}

/**
 * @brief Sets the number of bytes that the UART TX FIFO must have remaining in it to trigger a TX FIFO empty interrupt.
 * 
 * @param dev Pointer to a UART struct.
 * @param threshold The number of bytes remaining to trigger a TX FIFO empty interrupt.
 */
static inline IRAM_ATTR void dmx_hal_set_txfifo_empty_thr(uart_hal_context_t *hal, uint8_t threshold) {
  uart_hal_set_txfifo_empty_thr(hal, threshold);
}

/**
 * @brief Resets the UART RX FIFO.
 * 
 * @param dev Pointer to a UART struct.
 */
static inline IRAM_ATTR void dmx_hal_rxfifo_rst(uart_hal_context_t *hal) {
  uart_hal_rxfifo_rst(hal);
}

/**
 * @brief Get the length of the UART TX FIFO.
 * 
 * @param dev Pointer to a UART struct.
 * @return The length of the UART TX FIFO. 
 */
static inline IRAM_ATTR uint32_t dmx_hal_get_txfifo_len(uart_hal_context_t *hal) {
  return uart_hal_get_txfifo_len(hal);
}

static inline IRAM_ATTR void dmx_hal_write_txfifo(uart_hal_context_t *hal, const uint8_t *buf, uint32_t data_size, uint32_t *write_size) {
  uart_hal_write_txfifo(hal, buf, data_size, write_size);
}

static inline IRAM_ATTR void dmx_hal_txfifo_rst(uart_hal_context_t *hal) {
  uart_hal_txfifo_rst(hal);
}

#ifdef __cplusplus
}
#endif
