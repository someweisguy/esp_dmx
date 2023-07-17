#include "uart.h"

#include "dmx/caps.h"
#include "dmx/struct.h"
#include "esp_dmx.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_private/esp_clk.h"
#include "esp_private/periph_ctrl.h"
#include "esp_timer.h"
#else
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#endif

#define DMX_UART_FULL_DEFAULT 1
#define DMX_UART_EMPTY_DEFAULT 8

static struct dmx_uart_t {
  const int num;
  uart_dev_t *const dev;
  intr_handle_t isr_handle;
} dmx_uart_context[DMX_NUM_MAX] = {
    {.num = 0, .dev = UART_LL_GET_HW(0)},
    {.num = 1, .dev = UART_LL_GET_HW(1)},
#if DMX_NUM_MAX > 2
    {.num = 2, .dev = UART_LL_GET_HW(2)},
#endif
};

dmx_uart_handle_t dmx_uart_init(dmx_port_t dmx_num, void *isr_handle,
                                void *isr_context, int isr_flags) {
  dmx_uart_handle_t uart = &dmx_uart_context[dmx_num];

  // taskENTER_CRITICAL(spinlock);  FIXME
  periph_module_enable(uart_periph_signal[dmx_num].module);
  if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
#if SOC_UART_REQUIRE_CORE_RESET
    // ESP32C3 workaround to prevent UART outputting garbage data
    uart_ll_set_reset_core(uart->dev, true);
    periph_module_reset(uart_periph_signal[dmx_num].module);
    uart_ll_set_reset_core(uart->dev, false);
#else
    periph_module_reset(uart_periph_signal[dmx_num].module);
#endif
  }
  // taskEXIT_CRITICAL(spinlock); FIXME

  uart_ll_set_sclk(uart->dev, UART_SCLK_APB);
#if ESP_IDF_VERSION_MAJOR >= 5
  uart_ll_set_baudrate(uart->dev, DMX_BAUD_RATE, esp_clk_apb_freq());
#else
  uart_ll_set_baudrate(uart->dev, DMX_BAUD_RATE);
#endif
  uart_ll_set_mode(uart->dev, UART_MODE_UART);
  uart_ll_set_parity(uart->dev, UART_PARITY_DISABLE);
  uart_ll_set_data_bit_num(uart->dev, UART_DATA_8_BITS);
  uart_ll_set_stop_bits(uart->dev, UART_STOP_BITS_2);
  uart_ll_tx_break(uart->dev, 0);
  uart_ll_set_tx_idle_num(uart->dev, 0);
  uart_ll_set_hw_flow_ctrl(uart->dev, UART_HW_FLOWCTRL_DISABLE, 0);

  dmx_uart_rxfifo_reset(uart);
  dmx_uart_txfifo_reset(uart);
  dmx_uart_disable_interrupt(uart, UART_LL_INTR_MASK);
  dmx_uart_clear_interrupt(uart, UART_LL_INTR_MASK);
  dmx_uart_set_txfifo_empty(uart, DMX_UART_EMPTY_DEFAULT);
  dmx_uart_set_rxfifo_full(uart, DMX_UART_FULL_DEFAULT);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, isr_flags, isr_handle,
                 isr_context, &uart->isr_handle);

  return uart;
}

void dmx_uart_deinit(dmx_uart_handle_t uart) {
  if (uart->num != CONFIG_ESP_CONSOLE_UART_NUM) {
    periph_module_disable(uart_periph_signal[uart->num].module);
  }
}

uint32_t dmx_uart_get_baud_rate(dmx_uart_handle_t uart) {
  ;
#if ESP_IDF_VERSION_MAJOR >= 5
  return uart_ll_get_baudrate(uart->dev, esp_clk_apb_freq());
#else
  return uart_ll_get_baudrate(uart->dev);
#endif
}

void dmx_uart_set_baud_rate(dmx_uart_handle_t uart, uint32_t baud_rate) {
#if ESP_IDF_VERSION_MAJOR >= 5
  uart_ll_set_baudrate(uart->dev, baud_rate, esp_clk_apb_freq());
#else
  uart_ll_set_baudrate(uart->dev, baud_rate);
#endif
}

void dmx_uart_set_rxfifo_full(dmx_uart_handle_t uart, uint8_t threshold) {
  uart_ll_set_rxfifo_full_thr(uart->dev, threshold);
}

void dmx_uart_set_txfifo_empty(dmx_uart_handle_t uart, uint8_t threshold) {
  uart_ll_set_txfifo_empty_thr(uart->dev, threshold);
}

void DMX_ISR_ATTR dmx_uart_invert_tx(dmx_uart_handle_t uart, uint32_t invert) {
  uart->dev->conf0.txd_inv = invert;
}

int dmx_uart_get_rts(dmx_uart_handle_t uart) { return uart->dev->conf0.sw_rts; }

int DMX_ISR_ATTR dmx_uart_get_interrupt_status(dmx_uart_handle_t uart) {
  return uart_ll_get_intsts_mask(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_enable_interrupt(dmx_uart_handle_t uart, int mask) {
  uart_ll_ena_intr_mask(uart->dev, mask);
}

void DMX_ISR_ATTR dmx_uart_disable_interrupt(dmx_uart_handle_t uart, int mask) {
  uart_ll_disable_intr_mask(uart->dev, mask);
}

void DMX_ISR_ATTR dmx_uart_clear_interrupt(dmx_uart_handle_t uart, int mask) {
  uart_ll_clr_intsts_mask(uart->dev, mask);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_rxfifo_len(dmx_uart_handle_t uart) {
  return uart_ll_get_rxfifo_len(uart->dev);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_rx_level(dmx_uart_handle_t uart) {
  return uart->dev->status.rxd;
}

void DMX_ISR_ATTR dmx_uart_read_rxfifo(dmx_uart_handle_t uart, uint8_t *buf,
                                       int *size) {
  const size_t rxfifo_len = uart_ll_get_rxfifo_len(uart->dev);
  if (*size > rxfifo_len) {
    *size = rxfifo_len;
  }
  uart_ll_read_rxfifo(uart->dev, buf, *size);
}

void DMX_ISR_ATTR dmx_uart_set_rts(dmx_uart_handle_t uart, int set) {
  uart_ll_set_rts_active_level(uart->dev, set);
}

void DMX_ISR_ATTR dmx_uart_rxfifo_reset(dmx_uart_handle_t uart) {
  uart_ll_rxfifo_rst(uart->dev);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_txfifo_len(dmx_uart_handle_t uart) {
  return uart_ll_get_txfifo_len(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_write_txfifo(dmx_uart_handle_t uart, const void *buf,
                                        size_t *size) {
  const size_t txfifo_len = uart_ll_get_txfifo_len(uart->dev);
  if (*size > txfifo_len) *size = txfifo_len;
  uart_ll_write_txfifo(uart->dev, (uint8_t *)buf, *size);
}

void DMX_ISR_ATTR dmx_uart_txfifo_reset(dmx_uart_handle_t uart) {
  uart_ll_txfifo_rst(uart->dev);
}
