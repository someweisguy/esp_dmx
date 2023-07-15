#include "uart.h"

#include "dmx/caps.h"
#include "dmx/driver.h"
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

#ifdef CONFIG_DMX_ISR_IN_IRAM
#define DMX_ISR_ATTR IRAM_ATTR
#else
#define DMX_ISR_ATTR
#endif

static struct dmx_uart_t {
  uart_dev_t *const dev;
  intr_handle_t isr_handle;
  spinlock_t spinlock;
} dmx_uart_context[DMX_NUM_MAX] = {
    {.dev = UART_LL_GET_HW(1), .spinlock = portMUX_INITIALIZER_UNLOCKED},
    {.dev = UART_LL_GET_HW(2), .spinlock = portMUX_INITIALIZER_UNLOCKED},
#if DMX_NUM_MAX > 2
    {.dev = UART_LL_GET_HW(3), .spinlock = portMUX_INITIALIZER_UNLOCKED},
#endif
};

static void DMX_ISR_ATTR dmx_uart_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = arg;
  const dmx_port_t dmx_num = driver->dmx_num;
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_uart_get_interrupt_status(dmx_num);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_ALL) {
      // Stop the DMX driver hardware timer if it is running
      taskENTER_CRITICAL_ISR(&uart->spinlock);
      if (driver->flags & DMX_FLAGS_TIMER_IS_RUNNING) {
#if ESP_IDF_VERSION_MAJOR >= 5
        gptimer_stop(driver->gptimer_handle);
#else
        timer_group_set_counter_enable_in_isr(driver->timer_group,
                                              driver->timer_idx, 0);
#endif
        driver->flags &= ~DMX_FLAGS_TIMER_IS_RUNNING;
      }
      taskEXIT_CRITICAL_ISR(&uart->spinlock);

      // Read data into the DMX buffer if there is enough space
      const bool is_in_break = (intr_flags & DMX_INTR_RX_BREAK);
      taskENTER_CRITICAL_ISR(&uart->spinlock);
      if (driver->head >= 0 && driver->head < DMX_PACKET_SIZE_MAX) {
        int read_len = DMX_PACKET_SIZE_MAX - driver->head - is_in_break;
        dmx_uart_read_rxfifo(dmx_num, &driver->data[driver->head], &read_len);
        driver->head += read_len;
        if (driver->head > driver->rx_size) {
          driver->rx_size = driver->head;  // Update expected rx_size
        }
      } else {
        if (driver->head > 0) {
          // Record the number of slots received for error reporting
          driver->head += dmx_uart_get_rxfifo_len(dmx_num);
        }
        dmx_uart_rxfifo_reset(dmx_num);
      }
      taskEXIT_CRITICAL_ISR(&uart->spinlock);

      // Handle DMX break condition
      if (is_in_break) {
        taskENTER_CRITICAL_ISR(&uart->spinlock);
        // Handle receiveing a valid packet with smaller than expected size
        if (!(driver->flags & DMX_FLAGS_DRIVER_IS_IDLE) && driver->head > 0 &&
            driver->head < DMX_PACKET_SIZE_MAX) {
          driver->rx_size = driver->head - 1;
        }

        // Set driver flags
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IDLE;
        driver->head = 0;  // Driver is ready for data
        taskEXIT_CRITICAL_ISR(&uart->spinlock);
      }

      // Set last slot timestamp, DMX break flag, and clear interrupts
      taskENTER_CRITICAL_ISR(&uart->spinlock);
      driver->last_slot_ts = now;
      if (is_in_break) {
        driver->flags |= DMX_FLAGS_DRIVER_IS_IN_BREAK;
      } else {
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;
      }
      const int dmx_flags = driver->flags;
      taskEXIT_CRITICAL_ISR(&uart->spinlock);
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_RX_ALL);

      // Don't process data if end-of-packet condition already reached
      if (dmx_flags & DMX_FLAGS_DRIVER_IS_IDLE) {
        continue;
      }

      // Check for potential end-of-packet condition
      int rdm_type = 0;
      rdm_header_t header;
      esp_err_t err = ESP_OK;
      if (intr_flags & DMX_INTR_RX_ERR) {
        err = intr_flags & DMX_INTR_RX_FIFO_OVERFLOW
                  ? ESP_ERR_NOT_FINISHED  // UART overflow
                  : ESP_FAIL;             // Missing stop bits
      } else if (driver->head > 16 &&
                 driver->head ==
                     dmx_read_rdm(driver->dmx_num, &header, NULL, 0)) {
        rdm_type |= DMX_FLAGS_RDM_IS_VALID;
        rdm_uid_t my_uid;
        rdm_uid_get(driver->dmx_num, &my_uid);
        if (header.cc == RDM_CC_DISC_COMMAND ||
            header.cc == RDM_CC_GET_COMMAND ||
            header.cc == RDM_CC_SET_COMMAND) {
          rdm_type |= DMX_FLAGS_RDM_IS_REQUEST;
        }
        if (rdm_uid_is_broadcast(&header.dest_uid)) {
          rdm_type |= DMX_FLAGS_RDM_IS_BROADCAST;
        }
        if (header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
          rdm_type |= DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
        }
        if (rdm_uid_is_target(&my_uid, &header.dest_uid)) {
          rdm_type |= DMX_FLAGS_RDM_IS_RECIPIENT;
        }
      } else if (driver->head < driver->rx_size) {
        continue;
      }

      // Set driver flags and notify task
      taskENTER_CRITICAL_ISR(&uart->spinlock);
      driver->flags |= (DMX_FLAGS_DRIVER_HAS_DATA | DMX_FLAGS_DRIVER_IS_IDLE);
      driver->flags &= ~DMX_FLAGS_DRIVER_SENT_LAST;
      driver->rdm_type = rdm_type;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, err, eSetValueWithOverwrite,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(&uart->spinlock);
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // Write data to the UART and clear the interrupt
      size_t write_size = driver->tx_size - driver->head;
      dmx_uart_write_txfifo(dmx_num, &driver->data[driver->head], &write_size);
      driver->head += write_size;
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->head == driver->tx_size) {
        dmx_uart_disable_interrupt(dmx_num, DMX_INTR_TX_DATA);
      }
    } else if (intr_flags & DMX_INTR_TX_DONE) {
      // Disable write interrupts and clear the interrupt
      dmx_uart_disable_interrupt(dmx_num, DMX_INTR_TX_ALL);
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_TX_DONE);

      // Record timestamp, unset sending flag, and notify task
      taskENTER_CRITICAL_ISR(&uart->spinlock);
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_SENDING;
      driver->last_slot_ts = now;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, ESP_OK, eNoAction,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(&uart->spinlock);

      // Turn DMX bus around quickly if expecting an RDM response
      bool expecting_response = false;
      if (driver->rdm_type & DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH) {
        expecting_response = true;
        driver->head = 0;  // Not expecting a DMX break
      } else if (driver->rdm_type & DMX_FLAGS_RDM_IS_REQUEST) {
        expecting_response = true;
        driver->head = -1;  // Expecting a DMX break
      }
      if (expecting_response) {
        taskENTER_CRITICAL_ISR(&uart->spinlock);
        driver->flags &=
            ~(DMX_FLAGS_DRIVER_IS_IDLE | DMX_FLAGS_DRIVER_HAS_DATA);
        dmx_uart_rxfifo_reset(dmx_num);
        dmx_uart_set_rts(dmx_num, 1);
        taskEXIT_CRITICAL_ISR(&uart->spinlock);
      }
    }
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

void dmx_uart_init(dmx_port_t dmx_num, int isr_flags) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];

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

  dmx_uart_rxfifo_reset(dmx_num);
  dmx_uart_txfifo_reset(dmx_num);
  dmx_uart_disable_interrupt(dmx_num, DMX_ALL_INTR_MASK);
  dmx_uart_clear_interrupt(dmx_num, DMX_ALL_INTR_MASK);
  dmx_uart_set_txfifo_empty(dmx_num, DMX_UART_EMPTY_DEFAULT);
  dmx_uart_set_rxfifo_full(dmx_num, DMX_UART_FULL_DEFAULT);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, isr_flags, &dmx_uart_isr,
                 &dmx_driver[dmx_num], &uart->isr_handle);
}

void dmx_uart_deinit(dmx_port_t dmx_num) {
  // TODO
}

uint32_t dmx_uart_get_baud_rate(dmx_port_t dmx_num) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
#if ESP_IDF_VERSION_MAJOR >= 5
  return uart_ll_get_baudrate(uart->dev, esp_clk_apb_freq());
#else
  return uart_ll_get_baudrate(uart->dev);
#endif
}

void dmx_uart_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
#if ESP_IDF_VERSION_MAJOR >= 5
  uart_ll_set_baudrate(uart->dev, baud_rate, esp_clk_apb_freq());
#else
  uart_ll_set_baudrate(uart->dev, baud_rate);
#endif
}

void dmx_uart_set_rxfifo_full(dmx_port_t dmx_num, uint8_t threshold) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart_ll_set_rxfifo_full_thr(uart->dev, threshold);
}

void dmx_uart_set_txfifo_empty(dmx_port_t dmx_num, uint8_t threshold) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart_ll_set_txfifo_empty_thr(uart->dev, threshold);
}

void DMX_ISR_ATTR dmx_uart_invert_tx(dmx_port_t dmx_num, uint32_t invert) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart->dev->conf0.txd_inv = invert;
}

int dmx_uart_get_rts(dmx_port_t dmx_num) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  return uart->dev->conf0.sw_rts;
}

int DMX_ISR_ATTR dmx_uart_get_interrupt_status(dmx_port_t dmx_num) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_intsts_mask(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_enable_interrupt(dmx_port_t dmx_num, int mask) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart_ll_ena_intr_mask(uart->dev, mask);
}

void DMX_ISR_ATTR dmx_uart_disable_interrupt(dmx_port_t dmx_num, int mask) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart_ll_disable_intr_mask(uart->dev, mask);
}

void DMX_ISR_ATTR dmx_uart_clear_interrupt(dmx_port_t dmx_num, int mask) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart_ll_clr_intsts_mask(uart->dev, mask);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_rxfifo_len(dmx_port_t dmx_num) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_rxfifo_len(uart->dev);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_rx_level(dmx_port_t dmx_num) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  return uart->dev->status.rxd;
}

void DMX_ISR_ATTR dmx_uart_read_rxfifo(dmx_port_t dmx_num, uint8_t *buf,
                                       int *size) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  const size_t rxfifo_len = uart_ll_get_rxfifo_len(uart->dev);
  if (*size > rxfifo_len) {
    *size = rxfifo_len;
  }
  uart_ll_read_rxfifo(uart->dev, buf, *size);
}

void DMX_ISR_ATTR dmx_uart_set_rts(dmx_port_t dmx_num, int set) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart_ll_set_rts_active_level(uart->dev, set);
}

void DMX_ISR_ATTR dmx_uart_rxfifo_reset(dmx_port_t dmx_num) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart_ll_rxfifo_rst(uart->dev);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_txfifo_len(dmx_port_t dmx_num) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_txfifo_len(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_write_txfifo(dmx_port_t dmx_num, const void *buf,
                                        size_t *size) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  const size_t txfifo_len = uart_ll_get_txfifo_len(uart->dev);
  if (*size > txfifo_len) *size = txfifo_len;
  uart_ll_write_txfifo(uart->dev, (uint8_t *)buf, *size);
}

void DMX_ISR_ATTR dmx_uart_txfifo_reset(dmx_port_t dmx_num) {
  dmx_uart_t *const uart = &dmx_uart_context[dmx_num];
  uart_ll_txfifo_rst(uart->dev);
}
