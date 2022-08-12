#pragma once

#include "esp_dmx.h"
#include "impl/dmx_hal.h"
#include "impl/driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Helper function that takes an RDM UID from a most-significant-byte
 * first buffer and copies it to least-significant-byte first endianness, which
 * is what ESP32 uses.
 *
 * @note This function looks horrible, but it is the quickest way to swap
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
  DMX_INTR_RX_ERR = (DMX_INTR_RX_FIFO_OVERFLOW | DMX_INTR_RX_FRAMING_ERR),  // Interrupt mask that triggers when an error occurs.

  DMX_INTR_RX_BREAK = UART_INTR_BRK_DET,  // Interrupt mask that triggers when a DMX break is received.
  DMX_INTR_RX_DATA = UART_INTR_RXFIFO_FULL,  // Interrupt mask that is triggered when it is time to service the receive FIFO.
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
      // Read from the FIFO on a framing error then clear the FIFO and interrupt
      if (intr_flags & DMX_INTR_RX_FRAMING_ERR) {
        size_t read_len = DMX_MAX_PACKET_SIZE - driver->data.head;
        if (!driver->received_packet && read_len > 0) {
          uint8_t *data_ptr = &driver->data.buffer[driver->data.head];
          dmx_hal_read_rxfifo(&hardware->hal, data_ptr, &read_len);
          driver->data.head += read_len;
        } else {
          dmx_hal_rxfifo_rst(&hardware->hal);
        }
        driver->data.err = DMX_ERR_IMPROPERLY_FRAMED_SLOT;
      } else {
        driver->data.err = DMX_ERR_HARDWARE_OVERFLOW;
      }
      dmx_hal_rxfifo_rst(&hardware->hal);
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_ERR);

      // Don't process errors if the DMX bus is inactive
      if (driver->received_packet) {
        continue;
      }

      // Unset DMX break and receiving flags and notify task
      driver->is_in_break = false;
      driver->received_packet = true;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                           eSetValueWithOverwrite, &task_awoken);
      }
    }

    else if (intr_flags & DMX_INTR_RX_BREAK) {
      // Reset the FIFO and clear the interrupt
      dmx_hal_rxfifo_rst(&hardware->hal);
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_BREAK);

      // Update packet size guess if driver hasn't received a packet yet
      if (!driver->received_packet) {
        driver->data.rx_size = driver->data.head;
      }

      // Set driver flags and reset data head
      driver->received_packet = false;
      driver->is_in_break = true;
      driver->data.head = 0;

      // TODO: reset sniffer values
    }

    else if (intr_flags & DMX_INTR_RX_DATA) {
      // Read from the FIFO if ready and clear the interrupt
      size_t read_len = DMX_MAX_PACKET_SIZE - driver->data.head;
      if (!driver->received_packet && read_len > 0) {
        uint8_t *data_ptr = &driver->data.buffer[driver->data.head];
        dmx_hal_read_rxfifo(&hardware->hal, data_ptr, &read_len);
        driver->data.head += read_len;
      } else {
        dmx_hal_rxfifo_rst(&hardware->hal);
      }
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_DATA);

      // Unset DMX break flag and record the timestamp of the last slot
      if (driver->is_in_break) {
        driver->is_in_break = false;
      }
      driver->data.previous_ts = now;

      // Stop the receive timeout if it is running
      if (driver->timer_running) {
        timer_pause(driver->timer_group, driver->timer_num);
        driver->timer_running = false;
      }

      // Don't process data if the driver is done receiving
      if (driver->received_packet) {
        continue;
      }

      // Determine if a full packet has been received and notify the task
      const uint8_t sc = driver->data.buffer[0];  // DMX start code.
      if (sc == RDM_SC) {
        // An RDM packet is at least 26 bytes long
        if (driver->data.head >= 26) {
          // An RDM packet's length should match the message length slot value
          const rdm_data_t *rdm = (rdm_data_t *)driver->data.buffer;
          if (driver->data.head >= rdm->message_len + 2) {
            driver->data.previous_type = rdm->cc;
            driver->data.previous_uid = uidcpy(rdm->destination_uid);
            driver->received_packet = true;
          }
        }
      } else if (sc == RDM_PREAMBLE || sc == RDM_DELIMITER) {
        // An RDM discovery response packet is at least 17 bytes long
        if (driver->data.head >= 17) {
          // Find the length of the discovery response preamble (0-7 bytes)
          size_t preamble_len = 0;
          for (; preamble_len < 7; ++preamble_len) {
            if (driver->data.buffer[preamble_len] == RDM_DELIMITER) {
              break;
            }
          }
          // Discovery response packets are 17 bytes long after the preamble
          if (driver->data.head >= preamble_len + 17) {
            driver->data.previous_type = RDM_DISCOVERY_COMMAND_RESPONSE;
            driver->received_packet = true;
          }
        }
      } else {
        // A DMX packet size should be equal to the expected packet size
        if (driver->data.head >= driver->data.rx_size) {
          driver->data.previous_type = DMX_NON_RDM_PACKET;
          driver->received_packet = true;
        }
      }
      if (driver->received_packet) {
        driver->data.err = DMX_OK;
        driver->data.sent_previous = false;
        if (driver->task_waiting) {
          xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                             eSetValueWithOverwrite, &task_awoken);
        }
      }
    }

    else if (intr_flags & DMX_INTR_RX_CLASH) {
      // Multiple devices sent data at once (typical of RDM discovery)
      dmx_hal_rxfifo_rst(&hardware->hal);
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_CLASH);
      driver->data.err = DMX_ERR_DATA_COLLISION;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                           eSetValueWithOverwrite, &task_awoken);
      }
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // Write data to the UART and clear the interrupt
      size_t write_size = driver->data.tx_size - driver->data.head;
      const uint8_t *src = &driver->data.buffer[driver->data.head];
      dmx_hal_write_txfifo(&hardware->hal, src, &write_size);
      driver->data.head += write_size;
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->data.head == driver->data.tx_size) {
        dmx_hal_disable_interrupt(&hardware->hal, DMX_INTR_TX_DATA);
      }
    }

    else if (intr_flags & DMX_INTR_TX_DONE) {
      // Disable write interrupts and clear the interrupt
      dmx_hal_disable_interrupt(&hardware->hal, DMX_INTR_TX_ALL);
      dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_TX_DONE);

      // Record timestamp, unset sending flag, and notify task
      taskENTER_CRITICAL_ISR(&hardware->spinlock);
      driver->is_sending = false;
      driver->data.previous_ts = now;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, 0, eNoAction, &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(&hardware->spinlock);

      // Turn DMX bus around quickly if expecting an RDM response
      bool turn_bus_around = false;
      const rdm_data_t *rdm = (rdm_data_t *)driver->data.buffer;
      if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC) {
        // If packet was RDM and non-broadcast expect a response
        if (rdm->cc == RDM_GET_COMMAND || rdm->cc == RDM_SET_COMMAND) {
          const uint64_t destination_uid = uidcpy(rdm->destination_uid);
          if (destination_uid != RDM_BROADCAST_UID) {
            turn_bus_around = true;
          }
        } else if (rdm->cc == RDM_DISCOVERY_COMMAND) {
          // All discovery commands expect a response
          driver->received_packet = false;
          driver->data.head = 0;  // Response doesn't have a DMX break
          turn_bus_around = true;
        }
      }
      if (turn_bus_around) {
        dmx_hal_rxfifo_rst(&hardware->hal);
        dmx_hal_set_rts(&hardware->hal, 1);
        dmx_hal_clear_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
        dmx_hal_enable_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
      }
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
  // TODO
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

  if (!driver->is_sending && driver->task_waiting) {
    // Notify the task and pause the timer
    timer_pause(driver->timer_group, driver->timer_num);
    driver->timer_running = false;
    xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                       eSetValueWithOverwrite, &task_awoken);
  } else if (driver->is_in_break) {
    // End the DMX break
    dmx_hal_invert_tx(&hardware->hal, 0);
    driver->is_in_break = false;

    // Get the configured length of the DMX mark-after-break
    taskENTER_CRITICAL_ISR(&hardware->spinlock);
    const uint32_t mab_len = driver->mab_len;
    taskEXIT_CRITICAL_ISR(&hardware->spinlock);

    // Reset the alarm for the end of the DMX mark-after-break
    timer_group_set_alarm_value_in_isr(driver->timer_group, driver->timer_num,
                                       mab_len);
  } else {
    // Write data to the UART and pause the timer
    size_t write_size = driver->data.tx_size;
    dmx_hal_write_txfifo(&hardware->hal, driver->data.buffer, &write_size);
    driver->data.head += write_size;
    timer_pause(driver->timer_group, driver->timer_num);
    driver->timer_running = false;

    // Enable DMX write interrupts
    dmx_hal_enable_interrupt(&hardware->hal, DMX_INTR_TX_ALL);
  }

  return task_awoken;
}

#ifdef __cplusplus
}
#endif
