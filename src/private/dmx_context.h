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

#include "driver.h"
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
 * @brief The context for the DMX driver. Contains a pointer to UART registers
 * as well as a spinlock for synchronizing access to resources and tracks if the
 * UART hardware has been enabled.
 */
typedef struct dmx_context_t {
  spinlock_t spinlock;  // Synchronizes hardware and driver operations.
} dmx_context_t;

/**
 * @brief Initializes the UART for DMX.
 *
 * @param hal A pointer to a UART HAL context.
 */
void dmx_uart_init(dmx_port_t dmx_num, dmx_context_t *ctx) {  // FIXME: remove ctx
  // Initialize the UART peripheral
  taskENTER_CRITICAL(&ctx->spinlock);
  periph_module_enable(uart_periph_signal[dmx_num].module);
  if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
#if SOC_UART_REQUIRE_CORE_RESET
    // ESP32-C3 workaround to prevent UART outputting garbage data.
    uart_ll_set_reset_core(dev, true);
    periph_module_reset(uart_periph_signal[dmx_num].module);
    uart_ll_set_reset_core(dev, false);
#else
    periph_module_reset(uart_periph_signal[dmx_num].module);
#endif
  }
  taskEXIT_CRITICAL(&ctx->spinlock);

  uart_dev_t *const restrict dev = UART_LL_GET_HW(dmx_num);

  // Configure the UART for DMX output
  uart_ll_set_sclk(dev, UART_SCLK_APB);
  uart_ll_set_baudrate(dev, DMX_BAUD_RATE);
  uart_ll_set_mode(dev, UART_MODE_RS485_HALF_DUPLEX);
  uart_ll_set_parity(dev, UART_PARITY_DISABLE);
  uart_ll_set_data_bit_num(dev, UART_DATA_8_BITS);
  uart_ll_set_stop_bits(dev, UART_STOP_BITS_2);
  uart_ll_tx_break(dev, 0);
  uart_ll_set_tx_idle_num(dev, 0);
  uart_ll_set_hw_flow_ctrl(dev, UART_HW_FLOWCTRL_DISABLE, 0);

#if defined(CONFIG_IDF_TARGET_ESP32C3)
  // Fix inter-byte time on ESP32-C3. See below:
  // https://github.com/someweisguy/esp_dmx/issues/17#issuecomment-1133748359
  dev->rs485_conf.dl0_en = 0;
  dev->rs485_conf.dl1_en = 0;
#endif
}

/**
 * @brief Gets the UART baud rate of the selected UART hardware.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The baud rate of the UART hardware.
 */
uint32_t dmx_uart_get_baud_rate(uart_dev_t *dev) {
  return uart_ll_get_baudrate(dev);
}

/**
 * @brief Sets the baud rate for the UART.
 *
 * @param hal A pointer to a UART HAL context.
 * @param baud_rate The baud rate to use.
 */
void dmx_uart_set_baud_rate(uart_dev_t *dev, uint32_t baud_rate) {
  uart_ll_set_baudrate(dev, baud_rate);
}

/**
 * @brief Sets the number of bytes that the UART must receive to trigger a RX
 * FIFO full interrupt.
 *
 * @param hal A pointer to a UART HAL context.
 * @param threshold The number of bytes needed to trigger an RX FIFO full
 * interrupt.
 */
void dmx_uart_set_rxfifo_full(uart_dev_t *dev, uint8_t threshold) {
  uart_ll_set_rxfifo_full_thr(dev, threshold);
}

/**
 * @brief Sets the number of bytes that the UART TX FIFO must have remaining in
 * it to trigger a TX FIFO empty interrupt.
 *
 * @param hal A pointer to a UART HAL context.
 * @param threshold The number of bytes remaining to trigger a TX FIFO empty
 * interrupt.
 */
void dmx_uart_set_txfifo_empty(uart_dev_t *dev, uint8_t threshold) {
  uart_ll_set_txfifo_empty_thr(dev, threshold);
}

/**
 * @brief Inverts or un-inverts the TX line on the UART.
 *
 * @param hal A pointer to a UART HAL context.
 * @param invert_mask 1 to invert, 0 to un-invert.
 */
DMX_ISR_ATTR void dmx_uart_invert_tx(uart_dev_t *dev, uint32_t invert) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  dev->conf0.txd_inv = invert ? 1 : 0;
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#error ESP32-C2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  dev->conf0.txd_inv = invert ? 1 : 0;
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#error ESP32-H2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  dev->conf0.txd_inv = invert ? 1 : 0;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  dev->uart_conf0_reg_t.txd_inv = invert ? 1 : 0;
#else
#error Unknown target hardware.
#endif
}

/**
 * @brief Gets the level of the UART RTS line.
 *
 * @param hal A pointer to a UART HAL context.
 * @return 1 if the UART RTS line is enabled (set low; read), 0 if the UART RTS
 * line is disable (set high; write).
 */
int dmx_uart_get_rts(uart_dev_t *dev) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return dev->conf0.sw_rts;
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#error ESP32-C2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return dev->conf0.sw_rts;
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#error ESP32-H2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  return dev->conf0.sw_rts;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  return dev->uart_conf0_reg_t.sw_rts;
#else
#error Unknown target hardware.
#endif
}

/**
 * @brief Gets the interrupt status mask from the UART.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The interrupt status mask.
 */
DMX_ISR_ATTR int dmx_uart_get_interrupt_status(uart_dev_t *dev) {
  return uart_ll_get_intsts_mask(dev);
}

/**
 * @brief Enables UART interrupts using an interrupt mask.
 *
 * @param hal A pointer to a UART HAL context.
 * @param mask The UART mask that is enabled.
 */
DMX_ISR_ATTR void dmx_uart_enable_interrupt(uart_dev_t *dev, int mask) {
  uart_ll_ena_intr_mask(dev, mask);
}

/**
 * @brief Disables UART interrupts using an interrupt mask.
 *
 * @param hal A pointer to a UART HAL context.
 * @param mask The UART mask that is disabled.
 */
DMX_ISR_ATTR void dmx_uart_disable_interrupt(uart_dev_t *dev, int mask) {
  uart_ll_disable_intr_mask(dev, mask);
}

/**
 * @brief Clears UART interrupts using a mask.
 *
 * @param hal A pointer to a UART HAL context.
 * @param mask The UART mask that is cleared.
 */
DMX_ISR_ATTR void dmx_uart_clear_interrupt(uart_dev_t *dev, int mask) {
  uart_ll_clr_intsts_mask(dev, mask);
}

/**
 * @brief Gets the current length of the bytes in the UART RX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The number of bytes in the UART RX FIFO.
 */
DMX_ISR_ATTR uint32_t dmx_uart_get_rxfifo_len(uart_dev_t *dev) {
  return uart_ll_get_rxfifo_len(dev);
}

/**
 * @brief Gets the level of the UART RX line.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The UART RX line level.
 */

DMX_ISR_ATTR uint32_t dmx_uart_get_rx_level(uart_dev_t *dev) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return dev->status.rxd;
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#error ESP32-C2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return dev->status.rxd;
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#error ESP32-H2 is not yet supported.
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  return dev->status.rxd;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  return dev->uart_status_reg_t.rxd;
#else
#error Unknown target hardware.
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
DMX_ISR_ATTR void dmx_uart_read_rxfifo(uart_dev_t *dev, uint8_t *buf,
                                       int *size) {
  const size_t rxfifo_len = uart_ll_get_rxfifo_len(dev);
  if (*size > rxfifo_len) {
      *size = rxfifo_len;
  }
  uart_ll_read_rxfifo(dev, buf, *size);
}

/**
 * @brief Enables or disables the UART RTS line.
 *
 * @param hal A pointer to a UART HAL context.
 * @param set 1 to enable the UART RTS line (set low; read), 0 to disable the
 * UART RTS line (set high; write).
 */
DMX_ISR_ATTR void dmx_uart_set_rts(uart_dev_t *dev, int set) {
  uart_ll_set_rts_active_level(dev, set);
}

/**
 * @brief Resets the UART RX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 */
DMX_ISR_ATTR void dmx_uart_rxfifo_reset(uart_dev_t *dev) {
  uart_ll_rxfifo_rst(dev);
}

/**
 * @brief Gets the length of the UART TX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 * @return The length of the UART TX FIFO.
 */
DMX_ISR_ATTR uint32_t dmx_uart_get_txfifo_len(uart_dev_t *dev) {
  return uart_ll_get_txfifo_len(dev);
}

/**
 * @brief Writes to the UART TX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 * @param buf The source buffer from which to write.
 * @param size The number of bytes to write.
 */
DMX_ISR_ATTR void dmx_uart_write_txfifo(uart_dev_t *dev, const void *buf,
                                        size_t *size) {
  const size_t txfifo_len = uart_ll_get_txfifo_len(dev);
  if (*size > txfifo_len) *size = txfifo_len;
  uart_ll_write_txfifo(dev, buf, *size);
}

/**
 * @brief Resets the UART TX FIFO.
 *
 * @param hal A pointer to a UART HAL context.
 */
DMX_ISR_ATTR void dmx_uart_txfifo_reset(uart_dev_t *dev) {
  uart_ll_txfifo_rst(dev);
}

#ifdef __cplusplus
}
#endif
