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

#include "hal/uart_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets the interrupt status mask from the UART.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The interrupt status mask.
 */
IRAM_ATTR int dmx_hal_get_interrupt_status(uart_hal_context_t *hal) {
  return uart_ll_get_intsts_mask(hal->dev);
}

/**
 * @brief Enables UART interrupts using an interrupt mask.
 *
 * @param hal A pointer to a UART HAL context.
 * @param mask The UART mask that is enabled.
 */
IRAM_ATTR void dmx_hal_enable_interrupt(uart_hal_context_t *hal, int mask) {
  uart_ll_ena_intr_mask(hal->dev, mask);
}

/**
 * @brief Disables UART interrupts using an interrupt mask.
 *
 * @param hal A pointer to a UART HAL context.
 * @param mask The UART mask that is disabled.
 */
IRAM_ATTR void dmx_hal_disable_interrupt(uart_hal_context_t *hal, int mask) {
  uart_ll_disable_intr_mask(hal->dev, mask);
}

/**
 * @brief Clears UART interrupts using a mask.
 *
 * @param hal A pointer to a UART HAL context.
 * @param mask The UART mask that is cleared.
 */
IRAM_ATTR void dmx_hal_clear_interrupt(uart_hal_context_t *hal, int mask) {
  uart_ll_clr_intsts_mask(hal->dev, mask);
}

/**
 * @brief Gets the current length of the bytes in the UART RX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The number of bytes in the UART RX FIFO.
 */
IRAM_ATTR uint32_t dmx_hal_get_rxfifo_len(uart_hal_context_t *hal) {
  return uart_ll_get_rxfifo_len(hal->dev);
}

/**
 * @brief Inverts or un-inverts the TX line on the UART.
 *
 * @param hal A pointer to a UART HAL context.
 * @param invert_mask 1 to invert, 0 to un-invert.
 */
void dmx_hal_invert_tx(uart_hal_context_t *hal, uint32_t invert) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  hal->dev->conf0.txd_inv = invert ? 1 : 0;
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#error ESP32-C2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  hal->dev->conf0.txd_inv = invert ? 1 : 0;
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#error ESP32-H2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  hal->dev->conf0.txd_inv = invert ? 1 : 0;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  hal->dev->uart_conf0_reg_t.txd_inv = invert ? 1 : 0;
#else
#error Unknown target hardware.
#endif
}

/**
 * @brief Gets the level of the UART RX line.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The UART RX line level.
 */
IRAM_ATTR uint32_t dmx_hal_get_rx_level(uart_hal_context_t *hal) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return hal->dev->status.rxd;
// #elif defined(CONFIG_IDF_TARGET_ESP32C2)
// Not yet supported by ESP-IDF.
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hal->dev->status.rxd;
// #elif defined(CONFIG_IDF_TARGET_ESP32H2)
// Not yet supported by ESP-IDF.
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  return hal->dev->status.rxd;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  return hal->dev->uart_status_reg_t.rxd;
#else
#define DMX_GET_RX_LEVEL_NOT_IMPLEMENTED
  return 0;
#endif
}

/**
 * @brief Reads from the UART RX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 * @param buf Destination buffer to be read into.
 * @param num The maximum number of characters to read. Set to 0 to read all
 * data.
 * @return The number of characters read.
 */
IRAM_ATTR void dmx_hal_read_rxfifo(uart_hal_context_t *hal, uint8_t *buf,
                                   size_t *size) {
  const size_t rxfifo_len = uart_ll_get_rxfifo_len(hal->dev);
  if (*size > rxfifo_len) {
    *size = rxfifo_len;
  }
  uart_ll_read_rxfifo(hal->dev, buf, *size);
}

/**
 * @brief Enables or disables the UART RTS line.
 *
 * @param hal A pointer to a UART HAL context.
 * @param set 1 to enable the RTS line (set low; read), 0 to disable the RTS
 * line (set high; write).
 */
IRAM_ATTR void dmx_hal_set_rts(uart_hal_context_t *hal, int set) {
  uart_ll_set_rts_active_level(hal->dev, set);
}

bool dmx_hal_get_rts(uart_hal_context_t *hal) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return hal->dev->conf0.sw_rts;
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#error ESP32-C2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hal->dev->conf0.sw_rts;
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#error ESP32-H2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  return hal->dev->conf0.sw_rts;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  return hal->dev->uart_conf0_reg_t.sw_rts;
#else
#error Unknown target hardware.
#endif
}

/**
 * @brief Initializes the UART for DMX.
 *
 * @param hal A pointer to a UART HAL context.
 */
void dmx_hal_init(uart_hal_context_t *hal) {
  uart_ll_set_sclk(hal->dev, UART_SCLK_APB);
  uart_ll_set_baudrate(hal->dev, DMX_BAUD_RATE);
  uart_ll_set_mode(hal->dev, UART_MODE_RS485_HALF_DUPLEX);
  uart_ll_set_parity(hal->dev, UART_PARITY_DISABLE);
  uart_ll_set_data_bit_num(hal->dev, UART_DATA_8_BITS);
  uart_ll_set_stop_bits(hal->dev, UART_STOP_BITS_2);
  uart_ll_tx_break(hal->dev, 0);
  uart_ll_set_tx_idle_num(hal->dev, 0);
  uart_ll_set_hw_flow_ctrl(hal->dev, UART_HW_FLOWCTRL_DISABLE, 100);

#if defined(CONFIG_IDF_TARGET_ESP32C3)
  // Fix inter-byte time on ESP32-C3. See below:
  // https://github.com/someweisguy/esp_dmx/issues/17#issuecomment-1133748359
  hal->dev->rs485_conf.dl0_en = 0;
  hal->dev->rs485_conf.dl1_en = 0;
#endif
}

/**
 * @brief Sets the baud rate for the UART.
 *
 * @param hal A pointer to a UART HAL context.
 * @param baud_rate The baud rate to use.
 */
void dmx_hal_set_baud_rate(uart_hal_context_t *hal, uint32_t baud_rate) {
  uart_hal_set_baudrate(hal, baud_rate);
}

/**
 * @brief Gets the UART baud rate of the selected UART hardware.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The baud rate of the UART hardware.
 */
uint32_t dmx_hal_get_baud_rate(uart_hal_context_t *hal) {
  uint32_t baud_rate;
  uart_hal_get_baudrate(hal, &baud_rate);
  return baud_rate;
}

/**
 * @brief Sets the duration for the UART RX inactivity timeout that triggers the
 * RX timeout interrupt.
 *
 * @param hal A pointer to a UART HAL context.
 * @param threshold The RX timeout duration (unit: time of sending one byte).
 */
void dmx_hal_set_rx_timeout(uart_hal_context_t *hal, uint8_t threshold) {
  uart_hal_set_rx_timeout(hal, threshold);
}

/**
 * @brief Sets the number of bytes that the UART must receive to trigger a RX
 * FIFO full interrupt.
 *
 * @param hal A pointer to a UART HAL context.
 * @param threshold The number of bytes needed to trigger an RX FIFO full
 * interrupt.
 */
void dmx_hal_set_rxfifo_full(uart_hal_context_t *hal, uint8_t threshold) {
  uart_hal_set_rxfifo_full_thr(hal, threshold);
}

/**
 * @brief Sets the number of bytes that the UART TX FIFO must have remaining in
 * it to trigger a TX FIFO empty interrupt.
 *
 * @param hal A pointer to a UART HAL context.
 * @param threshold The number of bytes remaining to trigger a TX FIFO empty
 * interrupt.
 */
void dmx_hal_set_txfifo_empty(uart_hal_context_t *hal, uint8_t threshold) {
  uart_hal_set_txfifo_empty_thr(hal, threshold);
}

/**
 * @brief Resets the UART RX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 */
IRAM_ATTR void dmx_hal_rxfifo_rst(uart_hal_context_t *hal) {
  uart_ll_rxfifo_rst(hal->dev);
}

/**
 * @brief Gets the length of the UART TX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The length of the UART TX FIFO.
 */
IRAM_ATTR uint32_t dmx_hal_get_txfifo_len(uart_hal_context_t *hal) {
  return uart_ll_get_txfifo_len(hal->dev);
}

/**
 * @brief Writes to the UART TX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 * @param buf The source buffer from which to write.
 * @param size The number of bytes to write.
 */
IRAM_ATTR void dmx_hal_write_txfifo(uart_hal_context_t *hal, const void *buf,
                                    size_t *size) {
  const size_t txfifo_len = uart_ll_get_txfifo_len(hal->dev);
  if (*size > txfifo_len) {
    *size = txfifo_len;
  }
  uart_ll_write_txfifo(hal->dev, buf, *size);
}

/**
 * @brief Resets the UART TX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 */
IRAM_ATTR void dmx_hal_txfifo_rst(uart_hal_context_t *hal) {
  uart_ll_txfifo_rst(hal->dev);
}

#ifdef __cplusplus
}
#endif
