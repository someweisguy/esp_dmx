#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "impl/driver.h"
#include "impl/dmx_hal.h"
#include "esp_dmx.h"
#include "esp_system.h"

// UART level interupt masks
#define UART_INTR_RXFIFO_FULL           (1 << 0) // Interrupt that triggers when the RX FIFO is full.
#define UART_INTR_TXFIFO_EMPTY          (1 << 1) // Interrupt that triggers when the TX FIFO is empty.
#define UART_INTR_PARITY_ERR            (1 << 2) // Interrupt that triggers when there is a parity bit error.
#define UART_INTR_FRAME_ERR             (1 << 3) // Interrupt that triggers when there is a data bit framing error.
#define UART_INTR_RXFIFO_OVF            (1 << 4) // Interrupt that triggers when the RX FIFO overflows.
#define UART_INTR_BRK_DET               (1 << 7) // Interrupt that triggers when a break is detected (break bit occurs for longer than a frame length).
#define UART_INTR_RXFIFO_TOUT           (1 << 8) // Interrupt that triggers when the RX FIFO times out waiting for a new frame (mark bit occurs longer than the RX timeout duration).
#define UART_INTR_TX_BRK_DONE           (1 << 12) // Interrupt that triggers when the TX break is finished transmitting.
#define UART_INTR_TX_BRK_IDLE           (1 << 13) // Interrupt that triggers when done TX'ing data, but before the break is finished transmitting.
#define UART_INTR_TX_DONE               (1 << 14) // Interrupt that triggers when finished transmitting data, usually used to indicate a break is ready to be transmitted.
#define UART_INTR_RS485_PARITY_ERR      (1 << 15) // Interrupt that triggers when a RS485 mode parity error occurs.
#define UART_INTR_RS485_FRM_ERR         (1 << 16) // Interrupt that triggers when a RS485 mode frame error occurs.
#define UART_INTR_RS485_CLASH           (1 << 17) // Interrupt that triggers when a RS485 bus smashing event occurs.

#define DMX_INTR_RX_BRK                 (UART_INTR_BRK_DET) // Interrupt mask that represents a DMX break. 
#define DMX_INTR_RX_FRAMING_ERR         (UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR | UART_INTR_FRAME_ERR | UART_INTR_RS485_FRM_ERR) // Interrupt mask that represents a byte framing error.
#define DMX_INTR_RX_ERR                 (UART_INTR_RXFIFO_OVF | DMX_INTR_RX_FRAMING_ERR) // Interrupt mask that represents an error condition.
#define DMX_INTR_RX_ALL                 (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | DMX_INTR_RX_BRK | DMX_INTR_RX_ERR) // Interrupt mask that represents all rx conditions.

#define DMX_INTR_TX_ALL                 (UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE | UART_INTR_TX_DONE | UART_INTR_TX_BRK_DONE | UART_INTR_RS485_CLASH) // Interrupt mask that represents all tx conditions.
#define DMX_INTR_TX_ALL_TIMER           (UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_DONE | UART_INTR_RS485_CLASH) // Interrupt mask that represents all tx conditions.

#define DMX_ENTER_CRITICAL_ISR(mux)     portENTER_CRITICAL_ISR(mux)
#define DMX_EXIT_CRITICAL_ISR(mux)      portEXIT_CRITICAL_ISR(mux)

static void IRAM_ATTR dmx_intr_handler(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  const dmx_port_t dmx_num = driver->dmx_num;
  dmx_context_t *const hardware = &dmx_context[dmx_num];
  portBASE_TYPE task_awoken = pdFALSE;

  while (true) {
    const uint32_t uart_intr_status = dmx_hal_get_intsts_mask(&hardware->hal);
    if (uart_intr_status == 0) break;

    // DMX Transmit #####################################################
    if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {
      // this interrupt is triggered when the tx FIFO is empty

      uint32_t bytes_written;
      const uint32_t num_slots_to_read = driver->tx.size - driver->slot_idx;
      const uint8_t *next_slot = driver->buffer + driver->slot_idx;
      dmx_hal_write_txfifo(&(dmx_context[dmx_num].hal), next_slot, 
                           num_slots_to_read, &bytes_written);
      driver->slot_idx += bytes_written;

      if (driver->slot_idx == driver->tx.size) {
        // allow tx FIFO to empty - break and idle will be written
        DMX_ENTER_CRITICAL_ISR(&hardware->spinlock);
        dmx_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), 
                                  UART_INTR_TXFIFO_EMPTY);
        DMX_EXIT_CRITICAL_ISR(&hardware->spinlock);

        /* Users can block a task until a DMX packet is sent by calling 
        dmx_wait_send_done(). However, it's not necessary to wait until the DMX
        packet is transmitted onto the DMX bus. Users need only wait until the 
        DMX packet is written to the UART hardware. This ensures synchronicity 
        because the data, once written to the UART, cannot be changed by the
        user. It can also give up to 5.6ms of task time back to the task! */
        
        xSemaphoreGiveFromISR(driver->tx.done_sem, &task_awoken);

      }

      dmx_hal_clr_intsts_mask(&hardware->hal, UART_INTR_TXFIFO_EMPTY);
    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      // this interrupt is triggered when the last byte in tx fifo is written

      // track breaks if using uart hardware for reset sequence
      if (driver->rst_seq_hw == DMX_USE_UART) driver->tx.last_break_ts = now;

      dmx_hal_clr_intsts_mask(&hardware->hal, UART_INTR_TX_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      // this interrupt is triggered when the break is done

      dmx_hal_clr_intsts_mask(&hardware->hal, UART_INTR_TX_BRK_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_IDLE) {
      // this interrupt is triggered when the mark after break is done

      dmx_hal_clr_intsts_mask(&hardware->hal, UART_INTR_TX_BRK_IDLE);
    } else if (uart_intr_status & UART_INTR_RS485_CLASH) {
      // this interrupt is triggered if there is a bus collision
      // this code should only run when using RDM
      // TODO: move this to the receive side

      dmx_hal_clr_intsts_mask(&hardware->hal, UART_INTR_RS485_CLASH);
    }

    // DMX Receive ####################################################
    else if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
      // the uart overflowed
      dmx_event_t event = {
          .status = DMX_ERR_DATA_OVERFLOW,
          .start_code = -1,
          .size = driver->slot_idx
      };
      //xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
      driver->slot_idx = -1;

      dmx_hal_clr_intsts_mask(&hardware->hal, UART_INTR_RXFIFO_OVF);

    } else if (uart_intr_status & DMX_INTR_RX_BRK) {
      // break detected

      driver->rx.is_in_brk = true; // notify sniffer

      // TODO: servicing the fifo here fixes the frame sync problems
      //  I have no idea why :(
      //  breaks are read into the fifo as a 0 byte so ignore it

      if (driver->slot_idx != -1) {
        dmx_event_t event = {
            .status = DMX_OK,
            .start_code = driver->buffer[0],
            .size = driver->slot_idx,
            .duration = 22760 // FIXME
        };
        xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);
      }

      driver->slot_idx = 0;
      dmx_hal_rxfifo_rst(&hardware->hal);

      dmx_hal_clr_intsts_mask(&hardware->hal, DMX_INTR_RX_BRK);

    } else if (uart_intr_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT)) {
      // service the rx fifo
      const uint32_t rxfifo_len = dmx_hal_get_rxfifo_len(&hardware->hal);
      if (rxfifo_len > 0) {
        const uint16_t slots_rem = driver->buf_size - driver->slot_idx;
        const int rd_len = slots_rem > rxfifo_len ? rxfifo_len : slots_rem;
        if (slots_rem > 0 && driver->slot_idx != -1) {
          // read data into dmx buffer
          uint8_t *slot_ptr = driver->buffer + driver->slot_idx;
          dmx_hal_read_rxfifo(&hardware->hal, slot_ptr, rd_len);
        } else {
          // not enough buffer space left - discard fifo
          dmx_hal_rxfifo_rst(&hardware->hal);
        }
        driver->slot_idx += rd_len;

        // check if we are ready to send a queue event
        if (driver->slot_idx == driver->buf_size) { // TODO: || driver->slot_idx == driver->guessed_pkt_size
          dmx_event_t event = {
              .status = DMX_OK,
              .start_code = driver->buffer[0],
              .size = driver->slot_idx,
              .duration = 22760 // FIXME
          };
          xQueueSendFromISR(driver->rx.queue, &event, &task_awoken);


          // indicates queue event has been published
          driver->slot_idx = -1;
        }
      }

      // disable or reenable the rx fifo timeout interrupt
      if (uart_intr_status & UART_INTR_RXFIFO_TOUT) {
        DMX_ENTER_CRITICAL_ISR(&hardware->spinlock);
        dmx_hal_disable_intr_mask(&hardware->hal, UART_INTR_RXFIFO_TOUT);
        DMX_EXIT_CRITICAL_ISR(&hardware->spinlock);
      } else {
        DMX_ENTER_CRITICAL_ISR(&hardware->spinlock);
        dmx_hal_ena_intr_mask(&hardware->hal, UART_INTR_RXFIFO_TOUT);
        DMX_EXIT_CRITICAL_ISR(&hardware->spinlock);
      }

      dmx_hal_clr_intsts_mask(&hardware->hal, (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT));
    } else if (uart_intr_status & DMX_INTR_RX_FRAMING_ERR) {
        // TODO 

    } else {
        // disable interrupts that shouldn't be handled
        DMX_ENTER_CRITICAL_ISR(&hardware->spinlock);
        dmx_hal_disable_intr_mask(&hardware->hal, uart_intr_status);
        DMX_EXIT_CRITICAL_ISR(&hardware->spinlock);
        dmx_hal_clr_intsts_mask(&hardware->hal, uart_intr_status);
    }

    
    
    /*

      

      if (uart_intr_status & (DMX_INTR_RX_BRK | DMX_INTR_RX_ERR)) {
        // handle end-of-frame conditions
        if (driver->rx.queue && !rx_frame_err) {
          // report end-of-frame to event queue
          dmx_event_t event = { .size = driver->slot_idx };
          if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
            // FIFO overflowed
            event.status = DMX_ERR_DATA_OVERFLOW;
            event.start_code = -1;
          } else if (uart_intr_status & DMX_INTR_RX_FRAMING_ERR) {
            // improperly framed slot
            event.status = DMX_ERR_IMPROPER_SLOT;
            event.start_code = -1;
          } else if (driver->slot_idx < 1 || driver->slot_idx > DMX_MAX_PACKET_SIZE) {
            // invalid packet length
            event.status = DMX_ERR_PACKET_SIZE;
            event.start_code = -1;
          } else if (driver->slot_idx > driver->buf_size) {
            // buffer overflowed
            event.status = DMX_ERR_BUFFER_SIZE;
            event.start_code = driver->buffer[0];
          } else {
            // dmx ok
            event.status = DMX_OK;
            event.start_code = driver->buffer[0];
          }

          // check if this is the first received packet
          const int64_t rx_brk_to_brk = now - driver->rx.last_break_ts;
          if (rx_brk_to_brk <= DMX_RX_MAX_BRK_TO_BRK_US) { 
            // only send event if received at least 1 full packet
            event.timing.brk = driver->rx.break_len;
            event.timing.mab = driver->rx.mab_len;
            event.duration = rx_brk_to_brk;
            xQueueSendFromISR(driver->rx.queue, (void *)&event, &task_awoken);
          }

          // reset expired data
          driver->rx.break_len = -1;
          driver->rx.mab_len = -1;
        }

        // do setup for next frame
        if (uart_intr_status & DMX_INTR_RX_BRK) {
          driver->rx.is_in_brk = true; // notify sniffer
          // set break timestamp, and reset slot counter
          driver->rx.last_break_ts = now;
          driver->slot_idx = 0;
        } else {
          // set frame error, don't switch buffers until break rx'd
          driver->slot_idx = (uint16_t)-1;
        }
      }

      dmx_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), DMX_INTR_RX_ALL);
    } else {
      // disable interrupts that shouldn't be handled
      DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      dmx_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), uart_intr_status);
      DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      dmx_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), uart_intr_status);
    }
    */
  }
  
  if (task_awoken == pdTRUE) portYIELD_FROM_ISR();
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
  const dmx_port_t dmx_num = driver->dmx_num;

  if (driver->tx.step == 0) {
    // start break
    dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), UART_SIGNAL_TXD_INV);
    timer_set_alarm_value(driver->rst_seq_hw, driver->tx.timer_idx,
                          driver->tx.break_len);
  } else if (driver->tx.step == 1) {
    // start mab
    dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), 0);
    timer_set_alarm_value(driver->rst_seq_hw, driver->tx.timer_idx,
                          driver->tx.mab_len);
  } else {
    // write data to tx FIFO
    uint32_t bytes_written;
    dmx_hal_write_txfifo(&(dmx_context[dmx_num].hal), driver->buffer, 
                         driver->tx.size, &bytes_written);
    driver->slot_idx = bytes_written;

    // disable this interrupt
    timer_pause(driver->rst_seq_hw, driver->tx.timer_idx);

    // enable tx interrupts
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_TX_ALL_TIMER);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  }
  
  ++(driver->tx.step);

  return false;
}

#ifdef __cplusplus
}
#endif
