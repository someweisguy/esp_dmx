#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_dmx.h"
#include "esp_system.h"
#include "impl/dmx_hal.h"
#include "impl/driver.h"

/**
 * @brief Helper function that takes an RDM UID from a most-significant-byte
 * first buffer and copies it to least-significant-byte first endianness, which
 * is what ESP32 uses.
 *
 * @note This function looks horrible, but it is the most efficient way to swap
 * endianness of a 6-byte number on the Xtensa compiler, which is important
 * because it will be used exclusively in an interrupt handler.
 *
 * @param buf A pointer to an RDM buffer.
 * @return uint64_t The properly formatted RDM UID.
 */
FORCE_INLINE_ATTR uint64_t uidcpy(const void *buf) {
  uint64_t val;
  ((uint8_t *)&val)[5] = ((uint8_t *)buf)[0];
  ((uint8_t *)&val)[4] = ((uint8_t *)buf)[1];
  ((uint8_t *)&val)[3] = ((uint8_t *)buf)[2];
  ((uint8_t *)&val)[2] = ((uint8_t *)buf)[3];
  ((uint8_t *)&val)[1] = ((uint8_t *)buf)[4];
  ((uint8_t *)&val)[0] = ((uint8_t *)buf)[5];
  return val;
}

enum {
  DMX_INTR_RX_FIFO_OVERFLOW = UART_INTR_RXFIFO_OVF,  // Interrupt mask that triggers when the UART overflows.
  DMX_INTR_RX_FRAMING_ERR = (UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR | UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR),  // Interrupt mask that represents a byte framing error.
  DMX_INTR_RX_ERR = (DMX_INTR_RX_FIFO_OVERFLOW | DMX_INTR_RX_FRAMING_ERR),  // TODO

  DMX_INTR_RX_BREAK = UART_INTR_BRK_DET,  // Interrupt mask that trigger when a DMX break is received.
  DMX_INTR_RX_DATA = (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT),  // Interrupt mask that is triggered when it is time to service the receive FIFO.
  DMX_INTR_RX_TIMEOUT = UART_INTR_RXFIFO_TOUT,  // TODO
  DMX_INTR_RX_CLASH = UART_INTR_RS485_CLASH,  // Interrupt mask that represents a DMX collision.
  DMX_INTR_RX_ALL = (DMX_INTR_RX_DATA | DMX_INTR_RX_BREAK | DMX_INTR_RX_ERR | DMX_INTR_RX_CLASH),  // Interrupt mask that represents all receive conditions.

  DMX_INTR_TX_DATA = UART_INTR_TXFIFO_EMPTY,  // Interrupt mask that is triggered when the UART is ready to send data.
  DMX_INTR_TX_DONE = UART_INTR_TX_DONE,  // Interrupt mask that is triggered when the UART has finished writing data.
  DMX_INTR_TX_ALL = (DMX_INTR_TX_DATA | DMX_INTR_TX_DONE),  // Interrupt mask that represents all send conditions.

  DMX_ALL_INTR_MASK = -1  // Interrupt mask for all interrupts.
};

static void IRAM_ATTR dmx_uart_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const hardware = &dmx_context[driver->dmx_num];

  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_hal_get_interrupt_status(&hardware->hal);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_ERR) {
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_ERR);

      // Don't process errors if the DMX bus is inactive
      if (!driver->is_active) {
        continue;
      }

      // Indicate the driver is inactive and not in a DMX break
      driver->is_active = false;
      driver->is_in_break = false;

      // Set driver buffer error flags
      if (intr_flags & DMX_INTR_RX_FIFO_OVERFLOW) {
        driver->data.err = ESP_FAIL;
      } else {
        driver->data.err = ESP_ERR_INVALID_RESPONSE;
      }

      // Notify task that an error occurred if a task is waiting
      if (driver->data.task_waiting) {
        xTaskNotifyFromISR(driver->data.task_waiting, driver->data.err,
                           eSetValueWithOverwrite, &task_awoken);
      }

      // Reset the FIFO
      dmx_hal_rxfifo_rst(&hardware->hal);
    }

    else if (intr_flags & DMX_INTR_RX_BREAK) {
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_BREAK);

      // Indicate driver is active and in a DMX break
      driver->is_in_break = true;

      // Update packet size guess if driver is still reading data
      if (driver->is_active) {
        // Send a task notification if it hasn't been sent yet
        if (driver->data.task_waiting) {
          xTaskNotifyFromISR(driver->data.task_waiting, driver->data.err,
                             eSetValueWithOverwrite, &task_awoken);
        }
        driver->data.size = driver->data.head;  // Update packet size guess
      }

      // Set driver active flag, reset head and err, and reset the FIFO
      driver->is_active = true;
      driver->data.head = 0;
      driver->data.err = ESP_OK;
      dmx_hal_rxfifo_rst(&hardware->hal);

      // TODO: reset sniffer values
    }

    else if (intr_flags & DMX_INTR_RX_DATA) {
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_DATA);

      // Indicate driver is not in a DMX break
      if (driver->is_in_break) {
        driver->is_in_break = false;
      }

      // Determine the timestamp of the last slot
      if (intr_flags & DMX_INTR_RX_TIMEOUT) {
        uint8_t timeout = dmx_hal_get_rx_timeout_threshold(&hardware->hal);
        driver->data.last_received_ts = now - (timeout * 44);
      } else {
        driver->data.last_received_ts = now;
      }

      // Read from the FIFO if there is data and if the driver is ready
      size_t read_len = DMX_MAX_PACKET_SIZE - driver->data.head;
      if (driver->is_active && read_len > 0) {
        uint8_t *data_ptr = &driver->data.buffer[driver->data.head];
        dmx_hal_read_rxfifo(&hardware->hal, data_ptr, &read_len);
        driver->data.head += read_len;
      } else {
        dmx_hal_rxfifo_rst(&hardware->hal);
      }

      // Don't process data if driver already has or no task waiting
      if (!driver->is_active || driver->data.task_waiting == NULL) {
        continue;
      }

      // Determine if a full packet has been received
      bool packet_received = false;
      const uint8_t sc = driver->data.buffer[0];  // Received DMX start code.
      if (sc == DMX_SC) {
        // A DMX packet should equal the driver's expected packet size
        if (driver->data.head >= driver->data.size) {
          driver->data.last_received_packet = DMX_DIMMER_PACKET;
          packet_received = true;
        }
      } else if (sc == RDM_SC) {
        // An RDM packet is at least 26 bytes long
        if (driver->data.head >= 26) {
          // An RDM packet's length should match the message length slot value
          if (driver->data.head >= driver->data.buffer[3]) {
            driver->data.last_received_packet = driver->data.buffer[20];
            packet_received = true;
          }
        }
      } else if (sc == RDM_PREAMBLE || sc == RDM_DELIMITER) {
        // An RDM discovery response packet is at least 17 bytes long
        if (driver->data.head >= 17) {
          // Find the length of the discovery response preamble (0-7 bytes)
          int delimiter_idx = 0;
          for (; delimiter_idx < 7; ++delimiter_idx) {
            if (driver->data.buffer[delimiter_idx] == RDM_DELIMITER) {
              break;
            }
          }
          // Discovery response packets are 17 bytes long after the delimiter
          if (driver->data.head >= delimiter_idx + 17) {
            driver->data.last_received_packet = RDM_DISCOVERY_COMMAND_RESPONSE;
            packet_received = true;
          }
        }
      }

      // Notify the blocked task when a packet is received
      if (packet_received) {
        driver->is_active = false;
        xTaskNotifyFromISR(driver->data.task_waiting, driver->data.err,
                            eSetValueWithOverwrite, &task_awoken);
      }
    }

    else if (intr_flags & DMX_INTR_RX_CLASH) {
      // Multiple devices sent data at once (typical of RDM discovery)
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_CLASH);
      // TODO: this code should only run when using RDM
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // UART is ready to write more DMX data
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_TX_DATA);

      // Write data to the UART
      size_t write_size = driver->data.size - driver->data.head;
      dmx_hal_write_txfifo(&hardware->hal, driver->data.buffer, &write_size);
      driver->data.head += write_size;

      // Allow FIFO to empty when done writing data
      if (driver->data.head == driver->data.size) {
        taskENTER_CRITICAL_ISR(&hardware->spinlock);
        dmx_hal_disable_interrupt(&hardware->hal, DMX_INTR_TX_DATA);
        taskEXIT_CRITICAL_ISR(&hardware->spinlock);
      }
    }

    else if (intr_flags & DMX_INTR_TX_DONE) {
      // UART has finished sending DMX data
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_TX_DONE);

      // Record timestamp of last sent slot
      driver->data.last_sent_ts = now;

      // Set flags and signal data is sent
      driver->is_active = false;
      xSemaphoreGiveFromISR(driver->sent_semaphore, &task_awoken);
    }

    else {
      // disable interrupts that shouldn't be handled
      // this code shouldn't be called but it can prevent crashes when it is
      taskENTER_CRITICAL_ISR(&hardware->spinlock);
      dmx_hal_disable_interrupt(&hardware->hal, intr_flags);
      taskEXIT_CRITICAL_ISR(&hardware->spinlock);
      dmx_hal_clear_interrupt(&hardware->hal, intr_flags);
    }
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

static void IRAM_ATTR dmx_gpio_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;

  /* If this ISR is called on a positive edge and the current DMX frame is in a
  break and a negative edge condition has already occurred, then the break has
  just finished, so we can update the length of the break as well as unset the
  rx_is_in_brk flag. If this ISR is called on a negative edge and the
  mark-after-break has not been recorded while the break has been recorded,
  then we know that the mark-after-break has just completed so we should record
  its duration. */
/*
  if (dmx_hal_get_rx_level(&(dmx_context[driver->dmx_num].hal))) {
    if (driver->rx.is_in_brk && driver->rx.last_neg_edge_ts > -1) {
      driver->rx.break_len = now - driver->rx.last_neg_edge_ts;
      driver->rx.is_in_brk = false;
    }
    driver->rx.last_pos_edge_ts = now;
  } else {
    if (driver->rx.mab_len == -1 && driver->rx.break_len != -1)
      driver->rx.mab_len = now - driver->rx.last_pos_edge_ts;
    driver->rx.last_neg_edge_ts = now;
  }
  */
}

static bool IRAM_ATTR dmx_timer_isr(void *arg) {
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const hardware = &dmx_context[driver->dmx_num];
  int task_awoken = false;

  if (driver->is_in_break) {
    // End the DMX break
    dmx_hal_invert_signal(&hardware->hal, 0);
    driver->is_in_break = false;

    // Get the configured length of the DMX mark-after-break
    taskENTER_CRITICAL_ISR(&hardware->spinlock);
    const uint32_t mab_len = driver->tx.mab_len;
    taskEXIT_CRITICAL_ISR(&hardware->spinlock);

    // Reset the alarm for the end of the DMX mark-after-break
    timer_group_set_alarm_value_in_isr(driver->rst_seq_hw, driver->timer_idx,
                                       mab_len);
  } else {
    // Write data to the UART
    size_t write_size = driver->data.size - driver->data.head;
    dmx_hal_write_txfifo(&hardware->hal, driver->data.buffer, &write_size);
    driver->data.head += write_size;

    // Enable DMX write interrupts
    taskENTER_CRITICAL_ISR(&hardware->spinlock);
    dmx_hal_enable_interrupt(&hardware->hal, DMX_INTR_TX_ALL);
    taskEXIT_CRITICAL_ISR(&hardware->spinlock);

    // Pause the timer
    timer_pause(driver->rst_seq_hw, driver->timer_idx);
  }

  return task_awoken;
}

#ifdef __cplusplus
}
#endif
