#include "hal.h"

#include "dmx/caps.h"
#include "dmx/driver.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_private/esp_clk.h"
#endif

void dmx_uart_init(uart_dev_t *uart) {
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

uint32_t dmx_uart_get_baud_rate(uart_dev_t *uart) {
#if ESP_IDF_VERSION_MAJOR >= 5
  return uart_ll_get_baudrate(uart, esp_clk_apb_freq());
#else
  return uart_ll_get_baudrate(uart);
#endif
}

void dmx_uart_set_baud_rate(uart_dev_t *uart, uint32_t baud_rate) {
#if ESP_IDF_VERSION_MAJOR >= 5
  uart_ll_set_baudrate(uart, baud_rate, esp_clk_apb_freq());
#else
  uart_ll_set_baudrate(uart, baud_rate);
#endif
}

void dmx_uart_set_rxfifo_full(uart_dev_t *uart, uint8_t threshold) {
  uart_ll_set_rxfifo_full_thr(uart, threshold);
}

void dmx_uart_set_txfifo_empty(uart_dev_t *uart, uint8_t threshold) {
  uart_ll_set_txfifo_empty_thr(uart, threshold);
}

void DMX_ISR_ATTR dmx_uart_invert_tx(uart_dev_t *uart, uint32_t invert) {
  uart->conf0.txd_inv = invert;
}

int dmx_uart_get_rts(uart_dev_t *uart) { return uart->conf0.sw_rts; }

int DMX_ISR_ATTR dmx_uart_get_interrupt_status(uart_dev_t *uart) {
  return uart_ll_get_intsts_mask(uart);
}

void DMX_ISR_ATTR dmx_uart_enable_interrupt(uart_dev_t *uart, int mask) {
  uart_ll_ena_intr_mask(uart, mask);
}

void DMX_ISR_ATTR dmx_uart_disable_interrupt(uart_dev_t *uart, int mask) {
  uart_ll_disable_intr_mask(uart, mask);
}

void DMX_ISR_ATTR dmx_uart_clear_interrupt(uart_dev_t *uart, int mask) {
  uart_ll_clr_intsts_mask(uart, mask);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_rxfifo_len(uart_dev_t *uart) {
  return uart_ll_get_rxfifo_len(uart);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_rx_level(uart_dev_t *uart) {
  return uart->status.rxd;
}

void DMX_ISR_ATTR dmx_uart_read_rxfifo(uart_dev_t *uart, uint8_t *buf,
                                       int *size) {
  const size_t rxfifo_len = uart_ll_get_rxfifo_len(uart);
  if (*size > rxfifo_len) {
    *size = rxfifo_len;
  }
  uart_ll_read_rxfifo(uart, buf, *size);
}

void DMX_ISR_ATTR dmx_uart_set_rts(uart_dev_t *uart, int set) {
  uart_ll_set_rts_active_level(uart, set);
}

void DMX_ISR_ATTR dmx_uart_rxfifo_reset(uart_dev_t *uart) {
  uart_ll_rxfifo_rst(uart);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_txfifo_len(uart_dev_t *uart) {
  return uart_ll_get_txfifo_len(uart);
}

void DMX_ISR_ATTR dmx_uart_write_txfifo(uart_dev_t *uart, const void *buf,
                                        size_t *size) {
  const size_t txfifo_len = uart_ll_get_txfifo_len(uart);
  if (*size > txfifo_len) *size = txfifo_len;
  uart_ll_write_txfifo(uart, (uint8_t *)buf, *size);
}

void DMX_ISR_ATTR dmx_uart_txfifo_reset(uart_dev_t *uart) {
  uart_ll_txfifo_rst(uart);
}
