#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_dmx.h"
#include "esp_system.h"
#include "impl/dmx_hal.h"
#include "impl/driver.h"

#define SWAP16(x) ((uint16_t)x << 8 | (uint16_t)x >> 8)
#define GET_CHECKSUM(x) (*x << 8 | *(x + 1))

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
  DMX_INTR_RX_DATA = (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT),  // Interrupt mask that is triggered when it is time to service the receive FIFO.
  DMX_INTR_RX_BREAK = UART_INTR_BRK_DET,  // Interrupt mask that trigger when a DMX break is received.
  DMX_INTR_RX_FRAMING_ERR = (UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR | UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR),  // Interrupt mask that represents a byte framing error.
  DMX_INTR_RX_CLASH = UART_INTR_RS485_CLASH,  // Interrupt mask that represents a DMX collision.
  DMX_INTR_RX_ALL = (DMX_INTR_RX_DATA | DMX_INTR_RX_BREAK | DMX_INTR_RX_FIFO_OVERFLOW | DMX_INTR_RX_FRAMING_ERR | DMX_INTR_RX_CLASH),  // Interrupt mask that represents all receive conditions.

  DMX_INTR_TX_DATA = UART_INTR_TXFIFO_EMPTY,  // Interrupt mask that is triggered when the UART is ready to send data.
  DMX_INTR_TX_DONE = UART_INTR_TX_DONE,  // Interrupt mask that is triggered when the UART has finished writing data.
  DMX_INTR_TX_ALL = (DMX_INTR_TX_DATA | DMX_INTR_TX_DONE),  // Interrupt mask that represents all send conditions.

  DMX_ALL_INTR_MASK = -1  // Interrupt mask for all interrupts.
};

static void IRAM_ATTR dmx_intr_handler(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const hardware = &dmx_context[driver->dmx_num];

  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_hal_get_intsts_mask(&hardware->hal);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_FIFO_OVERFLOW) {
      // handle a UART overflow
      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_FIFO_OVERFLOW);

      if (driver->slot_idx > -1) {
        const uint32_t rxfifo_len = dmx_hal_get_rxfifo_len(&hardware->hal);
        dmx_event_t event = {.status = DMX_ERR_DATA_OVERFLOW,
                             .size = driver->slot_idx + rxfifo_len,
                             .timing = {.break_len = driver->rx.break_len,
                                        .mab_len = driver->rx.mab_len}};
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.event_sent = true;
        driver->slot_idx = -1;  // set error state
      }
      dmx_hal_rxfifo_rst(&hardware->hal);
    }

    else if (intr_flags & DMX_INTR_RX_FRAMING_ERR) {
      // handle situation where a malformed slot is received
      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_FRAMING_ERR);

      if (driver->slot_idx > -1) {
        const uint32_t rxfifo_len = dmx_hal_get_rxfifo_len(&hardware->hal);
        dmx_event_t event = {.status = DMX_ERR_IMPROPER_SLOT,
                             .size = driver->slot_idx + rxfifo_len,
                             .timing = {.break_len = driver->rx.break_len,
                                        .mab_len = driver->rx.mab_len}};
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.event_sent = true;
        driver->slot_idx = -1;  // set error state
      }
      dmx_hal_rxfifo_rst(&hardware->hal);
    }

    else if (intr_flags & DMX_INTR_RX_BREAK) {
      // handle receiving the DMX break
      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_BREAK);

      driver->rx.is_in_brk = true;  // notify sniffer
      if (!driver->rx.event_sent) {
        // haven't sent a queue event yet
        dmx_event_t event = {.status = DMX_OK,
                             .size = driver->slot_idx,
                             .timing = {.break_len = driver->rx.break_len,
                                        .mab_len = driver->rx.mab_len},
                             .is_late = true};
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.size_guess = driver->slot_idx;  // update guess
      } else if (driver->slot_idx > -1) {
        // check for errors that haven't been reported yet
        if (driver->slot_idx > DMX_MAX_PACKET_SIZE) {
          // packet length is longer than is allowed
          dmx_event_t event = {.status = DMX_ERR_PACKET_SIZE,
                               .size = driver->slot_idx};
          xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        } else if (driver->slot_idx > driver->buf_size) {
          // packet length is longer than the driver buffer
          dmx_event_t event = {.status = DMX_ERR_BUFFER_SIZE,
                               .size = driver->slot_idx};
          xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        }
      }

      // ready driver to read data into the buffer
      driver->slot_idx = 0;
      driver->rx.event_sent = false;
      driver->rx.break_len = -1;
      driver->rx.mab_len = -1;
      dmx_hal_rxfifo_rst(&hardware->hal);
    }

    else if (intr_flags & DMX_INTR_RX_DATA) {
      // data was received or timed out waiting for new data
      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_DATA);

      // read from the uart fifo
      if (driver->slot_idx > -1) {
        // driver is not in error state
        const uint32_t rxfifo_len = dmx_hal_get_rxfifo_len(&hardware->hal);
        const int16_t slots_rem = driver->buf_size - driver->slot_idx;
        if (driver->slot_idx < driver->buf_size) {
          // there are slots remaining to be read
          int rd_len = slots_rem > rxfifo_len ? rxfifo_len : slots_rem;
          uint8_t *slot_ptr = driver->buffer + driver->slot_idx;
          dmx_hal_read_rxfifo(&hardware->hal, slot_ptr, &rd_len);
          driver->slot_idx += rd_len;
        } else {
          // no slots remaining to be read
          dmx_hal_rxfifo_rst(&hardware->hal);
          driver->slot_idx += rxfifo_len;  // track for error reporting
        }
      } else {
        // driver is in error state
        dmx_hal_rxfifo_rst(&hardware->hal);
      }

      // Process received data
      if (driver->rx.event_sent) continue; // Only process data once
      const uint8_t sc = driver->buffer[0];  // Packet start-code.
      if (sc == DMX_SC) {
        if (driver->slot_idx < driver->rx.size_guess)
          continue;  // Haven't yet received a full DMX packet
        // Send a DMX event to the event queue
        dmx_event_t event = {.status = DMX_OK,
                             .size = driver->slot_idx,
                             .data_class = DMX_DATA_CLASS,
                             .timing = {.break_len = driver->rx.break_len,
                                        .mab_len = driver->rx.mab_len}};
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.event_sent = true;
      } else if (sc == RDM_SC && driver->slot_idx > 26) {
        if (driver->slot_idx < driver->buffer[2] + 2)
          continue;  // Haven't yet received a full RDM packet
        // TODO: verify checksum
        // TODO: check for sub start code
        // TODO: decode the rest of the RDM packet
        dmx_event_t event = {.status = DMX_OK,
                             .size = driver->slot_idx,
                             .data_class = RDM_DATA_CLASS,
                             .timing = {.break_len = driver->rx.break_len,
                                        .mab_len = driver->rx.mab_len},
                             .rdm = {
                                 // .destination_uid =  // TODO
                                 // .source_uid = // TODO
                                 // .transaction_num = // TODO
                                 // .port_id = // TODO
                                 // .message_count = // TODO
                                 // .sub_device = // TODO
                                 // .command_class = // TODO
                                 // .parameter_id = // TODO
                                 // .parameter_data_len = // TODO
                                 // .parameter_data = // TODO
                                 .checksum_is_valid = false  // TODO
                             }};
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.event_sent = true;
      } else if (sc == RDM_PREAMBLE || sc == RDM_DELIMITER) {
        if (driver->slot_idx < 17)
          continue;  // Haven't yet received a full RDM discovery response
        // Find the length of the discovery response preamble (0-7 bytes)
        int delimiter_idx = 0;
        for (; delimiter_idx < 7; ++delimiter_idx) {
          if (driver->buffer[delimiter_idx] == RDM_DELIMITER) break;
        }
        if (driver->buffer[delimiter_idx] != RDM_DELIMITER ||
            driver->slot_idx < delimiter_idx + 17)
          continue;  // No valid response or full packet not yet received
        // Decode the 6-byte UID and get the packet sum
        uint64_t uid;
        uint16_t sum = 0;
        const uint8_t *response = &driver->buffer[delimiter_idx + 1];
        for (int i = 5, j = 0; i >= 0; --i, j += 2) {
          ((uint8_t *)&uid)[i] = response[j] & 0x55;
          ((uint8_t *)&uid)[i] |= response[j + 1] & 0xaa;
          sum += ((uint8_t *)&uid)[i] + 0xff;
        }
        // Decode the checksum received in the response
        uint16_t checksum;
        for (int i = 1, j = 12; i >= 0; --i, j += 2) {
          ((uint8_t *)&checksum)[i] = response[j] & 0x55;
          ((uint8_t *)&checksum)[i] |= response[j + 1] & 0xaa;
        }
        // Send a discovery response event to the event queue
        dmx_event_t event = {
            .status = DMX_OK,
            .size = driver->slot_idx,
            .data_class = RDM_DATA_CLASS,
            .timing = {.break_len = 0, .mab_len = 0},  // No break or mab
            .rdm = {.source_uid = uid,
                    .command_class = DISCOVERY_COMMAND_RESPONSE,
                    .checksum_is_valid = (sum == checksum)}};
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.event_sent = true;
      }
      if (driver->rx.event_sent && driver->mode == DMX_MODE_WRITE) {
        // Turn bus around to write mode after receiving RDM response
        portENTER_CRITICAL(&hardware->spinlock);
        dmx_hal_disable_intr_mask(&hardware->hal, DMX_INTR_RX_ALL);
        dmx_hal_set_rts(&hardware->hal, 0);  // set rts high
        // Transmit interrupts are enabled when data is transmitted
        portEXIT_CRITICAL(&hardware->spinlock);
        dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_ALL);
      }
    }

    else if (intr_flags & DMX_INTR_RX_CLASH) {
      // Multiple devices sent data at once (typical of RDM discovery)
      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_CLASH);
      // TODO: this code should only run when using RDM
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // handle condition when UART is ready to send more data
      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_TX_DATA);

      // write slots
      int16_t wr_len = driver->tx.size - driver->slot_idx;
      const uint8_t *slot_ptr = driver->buffer + driver->slot_idx;
      dmx_hal_write_txfifo(&hardware->hal, slot_ptr, wr_len, &wr_len);
      driver->slot_idx += wr_len;

      // check if done writing data
      if (driver->slot_idx == driver->tx.size) {
        // allow UART to empty and unblock tasks
        portENTER_CRITICAL_ISR(&hardware->spinlock);
        dmx_hal_disable_intr_mask(&hardware->hal, DMX_INTR_TX_DATA);
        portEXIT_CRITICAL_ISR(&hardware->spinlock);
      }

    } else if (intr_flags & DMX_INTR_TX_DONE) {
      // handle condition when packet has finished being sent
      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_TX_DONE);

      driver->tx.last_data = now;
      if (driver->buffer[0] == RDM_SC) {
        // Turn the bus around to receive RDM response
        const uint64_t rdm_timeout = 2800;  // TODO: replace with macro
        timer_set_alarm_value(driver->rst_seq_hw, driver->timer_idx,
                              rdm_timeout);
        timer_start(driver->rst_seq_hw, driver->timer_idx);
        driver->awaiting_response = true;
        portENTER_CRITICAL(&hardware->spinlock);
        dmx_hal_disable_intr_mask(&hardware->hal, DMX_INTR_TX_ALL);
        portEXIT_CRITICAL(&hardware->spinlock);
        dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_TX_ALL);
        // RDM turnaround appears as a break. The slot_idx and event_sent will
        // be reset when the break is received.
        driver->slot_idx = -1;
        driver->rx.event_sent = true;
        dmx_hal_rxfifo_rst(&hardware->hal);
        portENTER_CRITICAL(&hardware->spinlock);
        dmx_hal_set_rts(&hardware->hal, 1);  // set rts low
        dmx_hal_ena_intr_mask(&hardware->hal, DMX_INTR_RX_ALL);
        portEXIT_CRITICAL(&hardware->spinlock);
      } else {
        // disable interrupt and unblock tasks
        portENTER_CRITICAL(&hardware->spinlock);
        dmx_hal_disable_intr_mask(&hardware->hal, DMX_INTR_TX_DONE);
        portEXIT_CRITICAL(&hardware->spinlock);
      }
      xSemaphoreGiveFromISR(driver->tx.sent_sem, &task_awoken);
    }

    else {
      // disable interrupts that shouldn't be handled
      // this code shouldn't be called but it can prevent crashes when it is
      portENTER_CRITICAL_ISR(&hardware->spinlock);
      dmx_hal_disable_intr_mask(&hardware->hal, intr_flags);
      portEXIT_CRITICAL_ISR(&hardware->spinlock);
      dmx_hal_clr_intsts_mask(&hardware->hal, intr_flags);
    }
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

static void IRAM_ATTR dmx_timing_intr_handler(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;

  /* If this ISR is called on a positive edge and the current DMX frame is in a
  break and a negative edge condition has already occurred, then the break has
  just finished, so we can update the length of the break as well as unset the
  rx_is_in_brk flag. If this ISR is called on a negative edge and the
  mark-after-break has not been recorded while the break has been recorded,
  then we know that the mark-after-break has just completed so we should record
  its duration. */

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
}

static bool IRAM_ATTR dmx_timer_intr_handler(void *arg) {
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const hardware = &dmx_context[driver->dmx_num];
  int task_awoken = false;

  if (driver->awaiting_response) {
    // Handle DMX timeouts
    // send timeout event to event queue
    dmx_event_t event = {
        .status = DMX_ERR_TIMEOUT,
        .data_class = RDM_DATA_CLASS,
        .size = driver->slot_idx,
    };
    xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
    driver->rx.event_sent = true;
    driver->awaiting_response = false;
    timer_pause(driver->rst_seq_hw, driver->timer_idx);
    // FIXME: turn line around if necessary
  } else if (driver->slot_idx == -1) {
    // end break, start mab
    dmx_hal_inverse_signal(&hardware->hal, 0);
    uint32_t mab_len;
    portENTER_CRITICAL(&hardware->spinlock);
    mab_len = driver->tx.mab_len;
    portEXIT_CRITICAL(&hardware->spinlock);
    timer_set_alarm_value(driver->rst_seq_hw, driver->timer_idx, mab_len);
    ++driver->slot_idx;
  } else {
    // write data to tx FIFO
    dmx_hal_write_txfifo(&hardware->hal, driver->buffer, driver->tx.size,
                         &driver->slot_idx);

    // disable this interrupt
    timer_pause(driver->rst_seq_hw, driver->timer_idx);

    // enable tx interrupts
    portENTER_CRITICAL(&hardware->spinlock);
    dmx_hal_ena_intr_mask(&hardware->hal, DMX_INTR_TX_ALL);
    portEXIT_CRITICAL(&hardware->spinlock);
  }

  return task_awoken;
}

#ifdef __cplusplus
}
#endif
