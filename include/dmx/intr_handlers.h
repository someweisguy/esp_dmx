#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "dmx/driver.h"
#include "dmx/hal.h"
#include "esp_dmx.h"
#include "esp_system.h"

#define DMX_INTR_RX_BRK                 (UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR | UART_INTR_BRK_DET) // Interrupt mask that represents a DMX break. 
#define DMX_INTR_RX_PARITY_ERR          (UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR) // Interrupt mask that represents a parity error.
#define DMX_INTR_RX_ERR                 (UART_INTR_RXFIFO_OVF | DMX_INTR_RX_PARITY_ERR) // Interrupt mask that represents an error condition.
#define DMX_INTR_RX_ALL                 (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | DMX_INTR_RX_BRK | DMX_INTR_RX_ERR) // Interrupt mask that represents all rx conditions.

#define DMX_INTR_TX_ALL                 (UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE | UART_INTR_TX_DONE | UART_INTR_TX_BRK_DONE | UART_INTR_RS485_CLASH) // Interrupt mask that represents all tx conditions.

#define DMX_ENTER_CRITICAL_ISR(mux)     portENTER_CRITICAL_ISR(mux)
#define DMX_EXIT_CRITICAL_ISR(mux)      portEXIT_CRITICAL_ISR(mux)

static void IRAM_ATTR dmx_intr_handler(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_obj_t *const p_dmx = (dmx_obj_t *)arg;
  const dmx_port_t dmx_num = p_dmx->dmx_num;
  portBASE_TYPE task_awoken = pdFALSE;

  while (true) {
    const uint32_t uart_intr_status = uart_hal_get_intsts_mask(&(dmx_context[dmx_num].hal));
    if (uart_intr_status == 0) break;

    // DMX Transmit #####################################################
    if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {
      // this interrupt is triggered when the tx FIFO is empty

      uint32_t bytes_written;
      const uint32_t slots_rem = p_dmx->buf_size - p_dmx->slot_idx;
      const uint8_t *offset = p_dmx->buffer[0] + p_dmx->slot_idx;
      uart_hal_write_txfifo(&(dmx_context[dmx_num].hal), offset, slots_rem,
        &bytes_written);
      p_dmx->slot_idx += bytes_written;

      if (p_dmx->slot_idx == p_dmx->buf_size) {
        // allow tx FIFO to empty - break and idle will be written
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_TXFIFO_EMPTY);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      }

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TXFIFO_EMPTY);
    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      // this interrupt is triggered when the last byte in tx fifo is written

      // switch buffers, signal end of frame, and track breaks
      memcpy(p_dmx->buffer[1], p_dmx->buffer[0], p_dmx->buf_size);
      xSemaphoreGiveFromISR(p_dmx->tx_done_sem, &task_awoken);
      p_dmx->tx_last_brk_ts = now;

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      // this interrupt is triggered when the break is done

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_IDLE) {
      // this interrupt is triggered when the mark after break is done

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_IDLE);
    } else if (uart_intr_status & UART_INTR_RS485_CLASH) {
      // this interrupt is triggered if there is a bus collision
      // this code should only run when using RDM

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_RS485_CLASH);
    }

    // DMX Recieve ####################################################
    else if (uart_intr_status & DMX_INTR_RX_ALL) {
      // this interrupt is triggered when any rx event occurs
      
      const bool rx_frame_err = (p_dmx->slot_idx == (uint16_t)-1);

      /* Check if there is data in the rx FIFO and if there is, it either reads
      the data into the driver buffer, or if there is not enough space in the
      buffer, discards it. In either case, the slot counter is incremented by
      the number of bytes received. Breaks are received as null slots so in the
      event of a break the slot counter is decremented by one. If there is a 
      frame error, discard the data and do not increment the slot counter. */

      const uint32_t rxfifo_len = dmx_hal_get_rxfifo_len(&(dmx_context[dmx_num].hal));
      if (rxfifo_len) {
        if (p_dmx->slot_idx < p_dmx->buf_size) {
          // read data from rx FIFO into the buffer
          const uint16_t slots_rem = p_dmx->buf_size - p_dmx->slot_idx + 1;
          uint8_t *offset = p_dmx->buffer[p_dmx->buf_idx] + p_dmx->slot_idx;
          int slots_rd = dmx_hal_readn_rxfifo(&(dmx_context[dmx_num].hal), 
            offset, slots_rem);
          p_dmx->slot_idx += slots_rd;
          if (uart_intr_status & DMX_INTR_RX_BRK) 
            --p_dmx->slot_idx; // break is not a slot
        } else {
          // discard bytes that can't be read into the buffer
          uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
          if (!rx_frame_err) {
            p_dmx->slot_idx += rxfifo_len;
            if (uart_intr_status & DMX_INTR_RX_BRK)
              --p_dmx->slot_idx; // break is not a slot
          }
        }
      }
      
      // handle data received condition
      if (uart_intr_status & UART_INTR_RXFIFO_TOUT) {
        // disable the rxfifo tout interrupt
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      } else {
        // enable the rxfifo tout interrupt
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      }

      if (uart_intr_status & (DMX_INTR_RX_BRK | DMX_INTR_RX_ERR)) {
        // handle end-of-frame conditions
        if (p_dmx->queue && !rx_frame_err) {
          // report end-of-frame to event queue
          dmx_event_t event = { .size = p_dmx->slot_idx };
          if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
            // FIFO overflowed
            event.type = DMX_ERR_DATA_OVERFLOW;
            event.start_code = -1;
          } else if (uart_intr_status & DMX_INTR_RX_PARITY_ERR) {
            // parity error
            event.type = DMX_ERR_IMPROPER_SLOT;
            event.start_code = -1;
          } else if (p_dmx->slot_idx <= 0 || p_dmx->slot_idx > DMX_MAX_PACKET_SIZE) {
            // invalid packet length
            event.type = DMX_ERR_PACKET_SIZE;
            event.start_code = -1;
          } else if (p_dmx->slot_idx > p_dmx->buf_size) {
            // buffer overflowed
            event.type = DMX_ERR_BUFFER_SIZE;
            event.start_code = p_dmx->buffer[p_dmx->buf_idx][0];
          } else {
            // dmx ok
            event.type = DMX_OK;
            event.start_code = p_dmx->buffer[p_dmx->buf_idx][0];
          }

          // check if this is the first received packet
          const int64_t rx_brk_to_brk = now - p_dmx->rx_last_brk_ts;
          if (rx_brk_to_brk > DMX_RX_MAX_BRK_TO_BRK_US) { 
            // connection was just established or the packet timed out
            event.timing.brk = -1;
            event.timing.mab = -1;
            event.duration = -1;
          } else {
            // fill out event timing data
            event.timing.brk = p_dmx->rx_brk_len;
            event.timing.mab = p_dmx->rx_mab_len;
            event.duration = rx_brk_to_brk;
          }
          
          // send the event and reset expired data
          xQueueSendFromISR(p_dmx->queue, (void *)&event, &task_awoken);
          p_dmx->rx_brk_len = -1;
          p_dmx->rx_mab_len = -1;
        }

        // do setup for next frame
        if (uart_intr_status & DMX_INTR_RX_BRK) {
          p_dmx->rx_is_in_brk = true; // notify analyzer
          // switch buffers, set break timestamp, and reset slot counter
          p_dmx->buf_idx = !p_dmx->buf_idx;
          p_dmx->rx_last_brk_ts = now;
          p_dmx->slot_idx = 0;
        } else {
          // set frame error, don't switch buffers until break rx'd
          p_dmx->slot_idx = (uint16_t)-1;
        }
      }

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), DMX_INTR_RX_ALL);
    } else {
      // disable interrupts that shouldn't be handled
      DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), uart_intr_status);
      DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), uart_intr_status);
    }
  }
  
  if (task_awoken == pdTRUE) portYIELD_FROM_ISR();
}

static void IRAM_ATTR dmx_analyze_intr_handler(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_obj_t *const p_dmx = (dmx_obj_t *)arg;

  /* If this ISR is called on a positive edge and the current DMX frame is in a
  break and a negative edge condition has already occured, then the break has 
  just finished, so we can update the length of the break as well as unset the 
  rx_is_in_brk flag. If this ISR is called on a negative edge and the 
  mark-after-break has not been recorded while the break has been recorded,
  then we know that the mark-after-break has just completed so we should record
  its duration. */

  if (dmx_hal_get_rx_level(&(dmx_context[p_dmx->dmx_num].hal))) {
    if (p_dmx->rx_is_in_brk && p_dmx->rx_last_neg_edge_ts > -1) {
      p_dmx->rx_brk_len = now - p_dmx->rx_last_neg_edge_ts;
      p_dmx->rx_is_in_brk = false;
    }
    p_dmx->rx_last_pos_edge_ts = now;
  } else {
    if (p_dmx->rx_mab_len == -1 && p_dmx->rx_brk_len != -1)
      p_dmx->rx_mab_len = now - p_dmx->rx_last_pos_edge_ts;
    p_dmx->rx_last_neg_edge_ts = now;
  }
}

#ifdef __cplusplus
}
#endif
