#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_dmx.h"
#include "esp_system.h"
#include "impl/dmx_hal.h"
#include "impl/driver.h"

// Interrupt mask that triggers when the UART overflows.
#define DMX_INTR_RX_FIFO_OVERFLOW (UART_INTR_RXFIFO_OVF)

// Interrupt mask that is triggered when it is time to service the receive FIFO.
#define DMX_INTR_RX_DATA (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT)

// Interrupt mask that trigger when a DMX break is received.
#define DMX_INTR_RX_BREAK (UART_INTR_BRK_DET)

// Interrupt mask that represents a byte framing error.
#define DMX_INTR_RX_FRAMING_ERR                                             \
  (UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR | UART_INTR_FRAM_ERR | \
   UART_INTR_RS485_FRM_ERR)

// Interrupt mask that is called when a DMX clash occurs.
#define DMX_INTR_RX_CLASH (UART_INTR_RS485_CLASH)

// Interrupt mask that represents all receive conditions.
#define DMX_INTR_RX_ALL                                               \
  (DMX_INTR_RX_DATA | DMX_INTR_RX_BREAK | DMX_INTR_RX_FIFO_OVERFLOW | \
   DMX_INTR_RX_FRAMING_ERR | DMX_INTR_RX_CLASH)

// Interrupt mask that is triggered when the UART is ready to send data.
#define DMX_INTR_TX_DATA (UART_INTR_TXFIFO_EMPTY)

// Interrupt mask that is triggered when the UART has finished writing data.
#define DMX_INTR_TX_DONE (UART_INTR_TX_DONE)

// Interrupt mask that is triggered when the UART idle-after-packet is done.
#define DMX_INTR_TX_IDLE_DONE (UART_INTR_TX_BRK_IDLE)

// Interrupt mask that represents all send conditions.
#define DMX_INTR_TX_ALL \
  (DMX_INTR_TX_DATA | DMX_INTR_TX_DONE | DMX_INTR_TX_IDLE_DONE)

// Interrupt mask for all interrupts.
#define DMX_ALL_INTR_MASK (-1)

static void IRAM_ATTR dmx_intr_handler(void *arg) {
  // initialize pointer consts - may be optimized away by compiler
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const hardware = &dmx_context[driver->dmx_num];

  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_hal_get_intsts_mask(&hardware->hal);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_FIFO_OVERFLOW) {
      // handle a UART overflow
      if (driver->slot_idx > -1) {
        const uint32_t rxfifo_len = dmx_hal_get_rxfifo_len(&hardware->hal);
        dmx_event_t event = {
            .status = DMX_ERR_DATA_OVERFLOW,
            .size = driver->slot_idx + rxfifo_len,
            .timing = {
                .brk = driver->rx.break_len,
                .mab = driver->rx.mab_len
            }
          };
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.event_sent = true;
        driver->slot_idx = -1;  // set error state
      }
      dmx_hal_rxfifo_rst(&hardware->hal);

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_FIFO_OVERFLOW);
    } else if (intr_flags & DMX_INTR_RX_FRAMING_ERR) {
      // handle situation where a malformed slot is received
      if (driver->slot_idx > -1) {
        const uint32_t rxfifo_len = dmx_hal_get_rxfifo_len(&hardware->hal);
        dmx_event_t event = {
            .status = DMX_ERR_IMPROPER_SLOT,
            .size = driver->slot_idx + rxfifo_len,
            .timing = {
                .brk = driver->rx.break_len,
                .mab = driver->rx.mab_len
            }
        };
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.event_sent = true;
        driver->slot_idx = -1;  // set error state
      }
      dmx_hal_rxfifo_rst(&hardware->hal);

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_FRAMING_ERR);
    } else if (intr_flags & DMX_INTR_RX_BREAK) {
      // handle receiving the DMX break
      driver->rx.is_in_brk = true;  // notify sniffer
      if (!driver->rx.event_sent) {
        // haven't sent a queue event yet
        dmx_event_t event = {
            .status = DMX_OK,
            .size = driver->slot_idx,
            .timing = {
                .brk = driver->rx.break_len,
                .mab = driver->rx.mab_len
            },
            .is_late = true
        };
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        driver->rx.size_guess = driver->slot_idx;  // update guess
      } else if (driver->slot_idx > -1) {
        // check for errors that haven't been reported yet
        if (driver->slot_idx > DMX_MAX_PACKET_SIZE) {
          // packet length is longer than is allowed
          dmx_event_t event = {
              .status = DMX_ERR_PACKET_SIZE,
              .size = driver->slot_idx
          };
          xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        } else if (driver->slot_idx > driver->buf_size) {
          // packet length is longer than the driver buffer
          dmx_event_t event = {
              .status = DMX_ERR_BUFFER_SIZE,
              .size = driver->slot_idx
          };
          xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
        }
      }

      // ready driver to read data into the buffer
      driver->slot_idx = 0;
      driver->rx.event_sent = false;
      driver->rx.break_len = -1;
      driver->rx.mab_len = -1;
      dmx_hal_rxfifo_rst(&hardware->hal);

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_BREAK);
    } else if (intr_flags & DMX_INTR_RX_DATA) {
      // data was received or timed out waiting for new data

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

      // check if an event needs to be sent to the queue
      if (!driver->rx.event_sent) {
        if (driver->buffer[0] == DMX_SC &&
            driver->slot_idx == driver->rx.size_guess) {
          // send a DMX event to the queue
          dmx_event_t event = {
              .status = DMX_OK,
              .size = driver->slot_idx,
              .timing = {
                  .brk = driver->rx.break_len,
                  .mab = driver->rx.mab_len
              }
          };
          xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
          driver->rx.event_sent = true;
        }
        // TODO: handle RDM events
      }

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_DATA);
    } else if (intr_flags & DMX_INTR_RX_CLASH) {
      // there was a clash on the DMX bus (multiple devices talking at once)
      // TODO: this code should only run when using RDM

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_CLASH);
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // handle condition when UART is ready to send more data

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
        xSemaphoreGiveFromISR(driver->tx.sync_sem, &task_awoken);
      }

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_TX_DATA);
    } else if (intr_flags & DMX_INTR_TX_DONE) {
      // handle condition when packet has finished being sent

      // disable interrupt and unblock tasks
      portENTER_CRITICAL(&hardware->spinlock);
      dmx_hal_disable_intr_mask(&hardware->hal, DMX_INTR_TX_DONE);
      portEXIT_CRITICAL(&hardware->spinlock);
      xSemaphoreGiveFromISR(driver->tx.sent_sem, &task_awoken);

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_TX_DONE);
    } else if (intr_flags & DMX_INTR_TX_IDLE_DONE) {
      // handle RDM turnaround condition
      // TODO

      // disable interrupt
      portENTER_CRITICAL(&hardware->spinlock);
      dmx_hal_disable_intr_mask(&hardware->hal,DMX_INTR_TX_IDLE_DONE);
      portEXIT_CRITICAL(&hardware->spinlock);

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_TX_IDLE_DONE);
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
  bool task_awoken = false;

  if (driver->slot_idx == -1) {
    // end break, start mab
    dmx_hal_inverse_signal(&hardware->hal, 0);
    timer_set_alarm_value(driver->rst_seq_hw, driver->timer_idx,
                          driver->tx.mab_len);
    ++driver->slot_idx;
  } else {
    // block tasks until fifo is written
    // TODO: does this actually do anything having this here? (as opposed to 
    //  having it in the dmx_send_packet() function)
    xSemaphoreTakeFromISR(driver->tx.sync_sem, &task_awoken);

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
