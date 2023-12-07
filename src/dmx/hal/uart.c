#include "include/uart.h"

#include "dmx/include/struct.h"
#include "driver/uart.h"
#include "esp_dmx.h"
#include "rdm/utils/bus_ctl.h"
#include "rdm/utils/uid.h"

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

static void DMX_ISR_ATTR dmx_uart_isr(void *arg) {
  const int64_t now = dmx_timer_get_micros_since_boot();
  dmx_driver_t *const driver = arg;
  const dmx_port_t dmx_num = driver->dmx_num;
  dmx_uart_handle_t uart = driver->uart;
  dmx_timer_handle_t timer = driver->timer;
  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_uart_get_interrupt_status(uart);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_ALL) {
      // Stop the DMX driver hardware timer if it is running
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      dmx_timer_stop(timer);
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

      // Read data into the DMX buffer if there is enough space
      const bool is_in_break = (intr_flags & DMX_INTR_RX_BREAK);
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      if (driver->head >= 0 && driver->head < DMX_PACKET_SIZE_MAX) {
        int read_len = DMX_PACKET_SIZE_MAX - driver->head - is_in_break;
        dmx_uart_read_rxfifo(uart, &driver->data[driver->head], &read_len);
        driver->head += read_len;
        if (driver->head > driver->rx_size) {
          driver->rx_size = driver->head;  // Update expected rx_size
        }
      } else {
        if (driver->head > 0) {
          // Record the number of slots received for error reporting
          driver->head += dmx_uart_get_rxfifo_len(uart);
        }
        dmx_uart_rxfifo_reset(uart);
      }
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

      // Handle DMX break condition
      if (is_in_break) {
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        // Handle receiveing a valid packet with smaller than expected size
        if (!(driver->flags & DMX_FLAGS_DRIVER_IS_IDLE) && driver->head > 0 &&
            driver->head < DMX_PACKET_SIZE_MAX) {
          driver->rx_size = driver->head - 1;
        }

        // Set driver flags
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IDLE;
        driver->head = 0;  // Driver is ready for data
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      }

      // Set last slot timestamp, DMX break flag, and clear interrupts
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->last_slot_ts = now;
      if (is_in_break) {
        driver->flags |= DMX_FLAGS_DRIVER_IS_IN_BREAK;
      } else {
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;
      }
      const int dmx_flags = driver->flags;
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      dmx_uart_clear_interrupt(uart, DMX_INTR_RX_ALL);

      // Don't process data if end-of-packet condition already reached
      if (dmx_flags & DMX_FLAGS_DRIVER_IS_IDLE) {
        continue;
      }

      // Continue loop if an end-of-packet condition has not been met
      dmx_err_t err = DMX_OK;
      if (intr_flags & DMX_INTR_RX_ERR) {
        err = intr_flags & DMX_INTR_RX_FIFO_OVERFLOW
                  ? DMX_ERR_UART_OVERFLOW   // UART overflow
                  : DMX_ERR_IMPROPER_SLOT;  // Missing stop bits
      } else if ((driver->data[0] != RDM_SC &&
                  driver->data[0] != RDM_PREAMBLE &&
                  driver->data[0] != RDM_DELIMITER &&
                  driver->head < driver->rx_size) ||
                 !(driver->head > 16 && rdm_read_header(dmx_num, NULL))) {
        // DMX packet is too small or RDM packet is not complete
        continue;
      }

      // Set driver flags and notify task
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->flags |= (DMX_FLAGS_DRIVER_HAS_DATA | DMX_FLAGS_DRIVER_IS_IDLE);
      driver->flags &= ~DMX_FLAGS_DRIVER_SENT_LAST;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, err, eSetValueWithOverwrite,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // Write data to the UART and clear the interrupt
      size_t write_size = driver->tx_size - driver->head;
      dmx_uart_write_txfifo(uart, &driver->data[driver->head], &write_size);
      driver->head += write_size;
      dmx_uart_clear_interrupt(uart, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->head == driver->tx_size) {
        dmx_uart_disable_interrupt(uart, DMX_INTR_TX_DATA);
      }
    } else if (intr_flags & DMX_INTR_TX_DONE) {
      // Disable write interrupts and clear the interrupt
      dmx_uart_disable_interrupt(uart, DMX_INTR_TX_ALL);
      dmx_uart_clear_interrupt(uart, DMX_INTR_TX_DONE);

      // Record timestamp, unset sending flag, and notify task
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_SENDING;
      driver->last_slot_ts = now;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, DMX_OK, eNoAction,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

      // Skip the rest of the ISR loop if an RDM response is not expected
      rdm_header_t header;
      if (!rdm_read_header(dmx_num, &header) || !rdm_cc_is_request(header.cc) ||
          (rdm_uid_is_broadcast(&header.dest_uid) &&
           header.pid != RDM_PID_DISC_UNIQUE_BRANCH)) {
        continue;
      }

      // Determine if a DMX break is expected in the response packet
      if (header.cc == RDM_CC_DISC_COMMAND &&
          header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
        driver->head = 0;  // Not expecting a DMX break
      } else {
        driver->head = -1;  // Expecting a DMX break
      }

      // Flip the DMX bus so the response may be read
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->flags &= ~(DMX_FLAGS_DRIVER_IS_IDLE | DMX_FLAGS_DRIVER_HAS_DATA);
      dmx_uart_rxfifo_reset(uart);
      dmx_uart_set_rts(uart, 1);
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    }
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

dmx_uart_handle_t dmx_uart_init(dmx_port_t dmx_num, void *isr_context,
                                int isr_flags) {
  dmx_uart_handle_t uart = &dmx_uart_context[dmx_num];

  periph_module_enable(uart_periph_signal[dmx_num].module);
  if (dmx_num != 0) {  // Default UART port for console
#if SOC_UART_REQUIRE_CORE_RESET
    // ESP32C3 workaround to prevent UART outputting garbage data
    uart_ll_set_reset_core(uart->dev, true);
    periph_module_reset(uart_periph_signal[dmx_num].module);
    uart_ll_set_reset_core(uart->dev, false);
#else
    periph_module_reset(uart_periph_signal[dmx_num].module);
#endif
  }
#if ESP_IDF_VERSION_MAJOR >= 5
  uart_ll_set_sclk(uart->dev, UART_SCLK_DEFAULT);
  uint32_t sclk_freq;
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
  uart_ll_set_baudrate(uart->dev, DMX_BAUD_RATE, sclk_freq);
#else
  uart_ll_set_sclk(uart->dev, UART_SCLK_APB);
  uart_ll_set_baudrate(uart->dev, DMX_BAUD_RATE);
#endif
  uart_ll_set_mode(uart->dev, UART_MODE_UART);
  uart_ll_set_parity(uart->dev, UART_PARITY_DISABLE);
  uart_ll_set_data_bit_num(uart->dev, UART_DATA_8_BITS);
  uart_ll_set_stop_bits(uart->dev, UART_STOP_BITS_2);
  uart_ll_tx_break(uart->dev, 0);
  uart_ll_set_tx_idle_num(uart->dev, 0);
  uart_ll_set_hw_flow_ctrl(uart->dev, UART_HW_FLOWCTRL_DISABLE, 0);
  uart_ll_set_txfifo_empty_thr(uart->dev, DMX_UART_EMPTY_DEFAULT);
  uart_ll_set_rxfifo_full_thr(uart->dev, DMX_UART_FULL_DEFAULT);

  dmx_uart_rxfifo_reset(uart);
  dmx_uart_txfifo_reset(uart);
  dmx_uart_disable_interrupt(uart, UART_LL_INTR_MASK);
  dmx_uart_clear_interrupt(uart, UART_LL_INTR_MASK);

  esp_intr_alloc(uart_periph_signal[dmx_num].irq, isr_flags, dmx_uart_isr,
                 isr_context, &uart->isr_handle);

  return uart;
}

void dmx_uart_deinit(dmx_uart_handle_t uart) {
  if (uart->num != 0) {  // Default UART port for console
    periph_module_disable(uart_periph_signal[uart->num].module);
  }
}

bool dmx_uart_set_pin(dmx_uart_handle_t uart, int tx, int rx, int rts) {
  esp_err_t err = uart_set_pin(uart->num, tx, rx, rts, -1);
  return (err == ESP_OK);
}

uint32_t dmx_uart_get_baud_rate(dmx_uart_handle_t uart) {
#if ESP_IDF_VERSION_MAJOR >= 5
  uint32_t sclk_freq;
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
  return uart_ll_get_baudrate(uart->dev, sclk_freq);
#else
  return uart_ll_get_baudrate(uart->dev);
#endif
}

void dmx_uart_set_baud_rate(dmx_uart_handle_t uart, uint32_t baud_rate) {
#if ESP_IDF_VERSION_MAJOR >= 5
  uint32_t sclk_freq;
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
  uart_ll_set_baudrate(uart->dev, baud_rate, sclk_freq);
#else
  uart_ll_set_baudrate(uart->dev, baud_rate);
#endif
}

void DMX_ISR_ATTR dmx_uart_invert_tx(dmx_uart_handle_t uart, uint32_t invert) {
#if CONFIG_IDF_TARGET_ESP32C6
  uart->dev->conf0_sync.txd_inv = invert;
  uart_ll_update(uart->dev);
#else
  uart->dev->conf0.txd_inv = invert;
#endif
}

int dmx_uart_get_rts(dmx_uart_handle_t uart) {
#if CONFIG_IDF_TARGET_ESP32C6
  return uart->dev->conf0_sync.sw_rts;
#else
  return uart->dev->conf0.sw_rts;
#endif
}

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
