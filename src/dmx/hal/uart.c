#include "include/uart.h"

#include "dmx/hal/include/timer.h"
#include "dmx/include/service.h"
#include "driver/uart.h"
#include "endian.h"
#include "rdm/include/driver.h"
#include "rdm/include/uid.h"

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
  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_uart_get_interrupt_status(dmx_num);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_ALL) {
      // Read data into the DMX buffer if there is enough space
      const int dmx_head = driver->dmx.head;
      if (dmx_head >= 0 && dmx_head < DMX_PACKET_SIZE_MAX) {
        int read_len = DMX_PACKET_SIZE_MAX - dmx_head;
        dmx_uart_read_rxfifo(dmx_num, &driver->dmx.data[dmx_head], &read_len);
        driver->dmx.head += read_len;
      } else {
        if (driver->dmx.head > 0) {
          // Record the number of slots received for error reporting
          driver->dmx.head += dmx_uart_get_rxfifo_len(dmx_num);
        }
        dmx_uart_rxfifo_reset(dmx_num);
      }
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_RX_ALL);

      // Handle DMX break condition
      if (intr_flags & DMX_INTR_RX_BREAK) {
        // Handle possible condition where expected packet size is too large
        if (driver->dmx.status == DMX_STATUS_NOT_READY &&
            driver->dmx.head > 0) {
          taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
          driver->dmx.rx_size = driver->dmx.head - 1;  // Attempt to fix
          if (driver->task_waiting) {
            xTaskNotifyFromISR(driver->task_waiting, DMX_ERR_NOT_ENOUGH_SLOTS,
                               eSetValueWithOverwrite, &task_awoken);
          }
          taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        }

        // Reset the DMX buffer for the next packet
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        driver->dmx.status = DMX_STATUS_NOT_READY;
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        driver->dmx.head = 0;
        continue;  // Nothing else to do on DMX break
      } else if (driver->flags & DMX_FLAGS_DRIVER_IS_IN_BREAK) {
        // TODO: update these vars with sniffer-specific vars
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      }

      // Guard against notifying multiple times for the same packet
      if (driver->dmx.status != DMX_STATUS_NOT_READY) {
        continue;
      }

      // Process the data depending on the type of packet that was received
      dmx_err_t err;
      bool packet_is_complete;
      if (intr_flags & DMX_INTR_RX_ERR) {
        packet_is_complete = true;
        err = intr_flags & DMX_INTR_RX_FIFO_OVERFLOW
                  ? DMX_ERR_UART_OVERFLOW   // UART overflow
                  : DMX_ERR_IMPROPER_SLOT;  // Missing stop bits
        driver->dmx.sent_last = false;
      } else {
        // Determine the type of the packet that was received
        if (dmx_head == 0 && driver->dmx.head > 0) {
          uint8_t initial_rdm_flags;
          const uint8_t sc = driver->dmx.data[0];  // DMX start-code.
          if (sc == RDM_SC) {
            initial_rdm_flags = DMX_TYPE_IS_RDM;
          } else if (sc == RDM_PREAMBLE || sc == RDM_DELIMITER) {
            initial_rdm_flags = DMX_TYPE_IS_RDM | DMX_TYPE_IS_DISCOVERY;
          } else {
            initial_rdm_flags = DMX_TYPE_IS_NOT_RDM;
          }
          taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
          driver->dmx.is_rdm = initial_rdm_flags;
          taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
          driver->dmx.sent_last = false;
        }
        err = DMX_OK;
      }
      while (err == DMX_OK) {
        if (driver->dmx.is_rdm & DMX_TYPE_IS_DISCOVERY) {
          // Parse an RDM discovery response packet
          if (driver->dmx.head < 17) {
            packet_is_complete = false;
            break;  // Haven't received the minimum packet size
          }

          // Get the delimiter index
          uint8_t delimiter_idx = 0;
          for (; delimiter_idx <= 7; ++delimiter_idx) {
            const uint8_t slot_value = driver->dmx.data[delimiter_idx];
            if (slot_value != RDM_PREAMBLE) {
              if (slot_value != RDM_DELIMITER) {
                delimiter_idx = 9;  // Force invalid packet type
              }
              break;
            }
          }
          if (delimiter_idx > 8) {
            taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            driver->dmx.is_rdm = DMX_TYPE_IS_NOT_RDM;
            taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            continue;  // Packet is malformed - treat it as DMX
          }

          // Process RDM discovery response packet
          if (driver->dmx.head < delimiter_idx + 17) {
            packet_is_complete = false;
            break;  // Haven't received full RDM_PID_DISC_UNIQUE_BRANCH response
          } else if (!rdm_read_header(dmx_num, NULL)) {
            taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            driver->dmx.is_rdm = DMX_TYPE_IS_NOT_RDM;
            taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            continue;  // Packet is malformed - treat it as DMX
          } else {
            packet_is_complete = true;
            break;
          }
        } else if (driver->dmx.is_rdm & DMX_TYPE_IS_RDM) {
          // Parse a standard RDM packet
          size_t msg_len;
          if (driver->dmx.head < sizeof(rdm_header_t) + 2) {
            packet_is_complete = false;
            break;  // Haven't received full RDM header and checksum yet
          } else if (driver->dmx.data[1] != RDM_SUB_SC ||
                     (msg_len = driver->dmx.data[2]) < sizeof(rdm_header_t)) {
            taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            driver->dmx.is_rdm = DMX_TYPE_IS_NOT_RDM;
            taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            continue;  // Packet is malformed - treat it as DMX
          } else if (driver->dmx.head < msg_len + 2) {
            packet_is_complete = false;
            break;  // Haven't received full RDM packet and checksum yet
          } else if (!rdm_read_header(dmx_num, NULL)) {
            taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            driver->dmx.is_rdm = DMX_TYPE_IS_NOT_RDM;
            taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
            continue;  // Packet is malformed - treat it as DMX
          } else {
            packet_is_complete = true;
            break;
          }
        } else {
          // Parse a standard DMX packet
          packet_is_complete = (driver->dmx.head >= driver->dmx.rx_size);
          break;
        }
      }
      if (!packet_is_complete) {
        continue;
      }

      // Fill out remaining RDM flags
      uint8_t additional_rdm_flags;
      if (driver->dmx.is_rdm & DMX_TYPE_IS_DISCOVERY) {
        additional_rdm_flags = 0;  // Discovery response packets don't get flags
      } else if (driver->dmx.is_rdm & DMX_TYPE_IS_RDM) {
        const rdm_cc_t cc = driver->dmx.data[20];
        additional_rdm_flags = rdm_cc_is_valid(cc) && rdm_cc_is_request(cc)
                        ? DMX_TYPE_IS_REQUEST
                        : 0;
        const rdm_uid_t *uid_ptr = (rdm_uid_t *)&driver->dmx.data[3];
        const rdm_uid_t dest_uid = {.man_id = bswap16(uid_ptr->man_id),
                                    .dev_id = bswap32(uid_ptr->dev_id)};
        additional_rdm_flags |= rdm_uid_is_target(&driver->uid, &dest_uid)
                         ? DMX_TYPE_IS_ADDRESSEE
                         : 0;
      } else {
        additional_rdm_flags = 0;  // DMX packets don't get RDM flags
      }
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->dmx.is_rdm |= additional_rdm_flags;
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

      // Set driver flags and notify task
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->dmx.status = DMX_STATUS_READY;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, err, eSetValueWithOverwrite,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // Write data to the UART and clear the interrupt
      size_t write_size = driver->dmx.tx_size - driver->dmx.head;
      dmx_uart_write_txfifo(dmx_num, &driver->dmx.data[driver->dmx.head],
                            &write_size);
      driver->dmx.head += write_size;
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->dmx.head == driver->dmx.tx_size) {
        dmx_uart_disable_interrupt(dmx_num, DMX_INTR_TX_DATA);
      }
    } else if (intr_flags & DMX_INTR_TX_DONE) {
      // Disable write interrupts and clear the interrupt
      dmx_uart_disable_interrupt(dmx_num, DMX_INTR_TX_ALL);
      dmx_uart_clear_interrupt(dmx_num, DMX_INTR_TX_DONE);

      // Record timestamp, unset sending flag, and notify task
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_SENDING;
      driver->dmx.last_slot_ts = now;
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
        driver->dmx.head = 0;  // Not expecting a DMX break
      } else {
        driver->dmx.head = -1;  // Expecting a DMX break
      }

      // Flip the DMX bus so the response may be read
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->flags &= ~(DMX_FLAGS_DRIVER_IS_IDLE | DMX_FLAGS_DRIVER_HAS_DATA);
      dmx_uart_rxfifo_reset(dmx_num);
      dmx_uart_set_rts(dmx_num, 1);
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    }
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

bool dmx_uart_init(dmx_port_t dmx_num, void *isr_context, int isr_flags) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];

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

  dmx_uart_rxfifo_reset(dmx_num);
  dmx_uart_txfifo_reset(dmx_num);
  dmx_uart_disable_interrupt(dmx_num, UART_LL_INTR_MASK);
  dmx_uart_clear_interrupt(dmx_num, UART_LL_INTR_MASK);

  esp_intr_alloc(uart_periph_signal[dmx_num].irq, isr_flags, dmx_uart_isr,
                 isr_context, &uart->isr_handle);

  return uart;
}

void dmx_uart_deinit(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  if (uart->num != 0) {  // Default UART port for console
    periph_module_disable(uart_periph_signal[uart->num].module);
  }
}

bool dmx_uart_set_pin(dmx_port_t dmx_num, int tx, int rx, int rts) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  esp_err_t err = uart_set_pin(uart->num, tx, rx, rts, -1);
  return (err == ESP_OK);
}

uint32_t dmx_uart_get_baud_rate(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
#if ESP_IDF_VERSION_MAJOR >= 5
  uint32_t sclk_freq;
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
  return uart_ll_get_baudrate(uart->dev, sclk_freq);
#else
  return uart_ll_get_baudrate(uart->dev);
#endif
}

void dmx_uart_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
#if ESP_IDF_VERSION_MAJOR >= 5
  uint32_t sclk_freq;
  uart_get_sclk_freq(UART_SCLK_DEFAULT, &sclk_freq);
  uart_ll_set_baudrate(uart->dev, baud_rate, sclk_freq);
#else
  uart_ll_set_baudrate(uart->dev, baud_rate);
#endif
}

void DMX_ISR_ATTR dmx_uart_invert_tx(dmx_port_t dmx_num, uint32_t invert) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
#if CONFIG_IDF_TARGET_ESP32C6
  uart->dev->conf0_sync.txd_inv = invert;
  uart_ll_update(uart->dev);
#else
  uart->dev->conf0.txd_inv = invert;
#endif
}

int dmx_uart_get_rts(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
#if CONFIG_IDF_TARGET_ESP32C6
  return uart->dev->conf0_sync.sw_rts;
#else
  return uart->dev->conf0.sw_rts;
#endif
}

int DMX_ISR_ATTR dmx_uart_get_interrupt_status(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_intsts_mask(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_enable_interrupt(dmx_port_t dmx_num, int mask) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_ena_intr_mask(uart->dev, mask);
}

void DMX_ISR_ATTR dmx_uart_disable_interrupt(dmx_port_t dmx_num, int mask) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_disable_intr_mask(uart->dev, mask);
}

void DMX_ISR_ATTR dmx_uart_clear_interrupt(dmx_port_t dmx_num, int mask) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_clr_intsts_mask(uart->dev, mask);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_rxfifo_len(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_rxfifo_len(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_read_rxfifo(dmx_port_t dmx_num, uint8_t *buf,
                                       int *size) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  const size_t rxfifo_len = uart_ll_get_rxfifo_len(uart->dev);
  if (*size > rxfifo_len) {
    *size = rxfifo_len;
  }
  uart_ll_read_rxfifo(uart->dev, buf, *size);
}

void DMX_ISR_ATTR dmx_uart_set_rts(dmx_port_t dmx_num, int set) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_set_rts_active_level(uart->dev, set);
}

void DMX_ISR_ATTR dmx_uart_rxfifo_reset(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_rxfifo_rst(uart->dev);
}

uint32_t DMX_ISR_ATTR dmx_uart_get_txfifo_len(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  return uart_ll_get_txfifo_len(uart->dev);
}

void DMX_ISR_ATTR dmx_uart_write_txfifo(dmx_port_t dmx_num, const void *buf,
                                        size_t *size) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  const size_t txfifo_len = uart_ll_get_txfifo_len(uart->dev);
  if (*size > txfifo_len) *size = txfifo_len;
  uart_ll_write_txfifo(uart->dev, (uint8_t *)buf, *size);
}

void DMX_ISR_ATTR dmx_uart_txfifo_reset(dmx_port_t dmx_num) {
  struct dmx_uart_t *uart = &dmx_uart_context[dmx_num];
  uart_ll_txfifo_rst(uart->dev);
}
