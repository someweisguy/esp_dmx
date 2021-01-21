#pragma once

#include "driver/dmx_ctrl.h"
#include "esp_system.h"
#include "hal/dmx_hal.h"
#include "hal/uart_hal.h"

#include "driver/gpio.h" // TODO: for debugging

#ifdef CONFIG_UART_ISR_IN_IRAM
#define DMX_ISR_ATTR IRAM_ATTR
#else
#define DMX_ISR_ATTR
#endif

#define DMX_ENTER_CRITICAL_ISR(mux) portENTER_CRITICAL_ISR(mux)
#define DMX_EXIT_CRITICAL_ISR(mux)  portEXIT_CRITICAL_ISR(mux)

#define DMX_INTR_RX_BRK (UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR | UART_INTR_BRK_DET)
#define DMX_INTR_RX_ERR (UART_INTR_RXFIFO_OVF | UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR)


void DMX_ISR_ATTR dmx_default_intr_handler(void *arg) {
  gpio_set_level(33, 1);  // TODO: for debugging
  const int64_t now = esp_timer_get_time();
  dmx_obj_t *const p_dmx = (dmx_obj_t *)arg;
  const dmx_port_t dmx_num = p_dmx->dmx_num;
  portBASE_TYPE task_awoken = 0;

  while (true) {
    const uint32_t uart_intr_status = uart_hal_get_intsts_mask(&(dmx_context[dmx_num].hal));
    if (uart_intr_status == 0) break;

    // DMX Transmit #####################################################
    if (uart_intr_status & UART_INTR_TXFIFO_EMPTY) {
      // this interrupt is triggered when the tx FIFO is empty, or the mark after break is done

      uint32_t bytes_written;
      const uint32_t len = p_dmx->buf_size - p_dmx->slot_idx;
      const uint8_t *offset = p_dmx->buffer[p_dmx->buf_idx] + p_dmx->slot_idx;
      uart_hal_write_txfifo(&(dmx_context[dmx_num].hal), offset, len,
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
      // this interrupt is triggered when the UART break is done

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_IDLE) {
      // this interrupt is triggered when the UART mark after break is done

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_IDLE);
    } else if (uart_intr_status & UART_INTR_RS485_CLASH) {
      // this interrupt is triggered if there is a bus collision - should only occur with RDM

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_RS485_CLASH);
    }

    // DMX Recieve ####################################################
    else if (uart_intr_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | DMX_INTR_RX_BRK | DMX_INTR_RX_ERR)) {
      // this interrupt is triggered when any rx event occurs


      if (p_dmx->slot_idx < p_dmx->buf_size) {
        const int16_t slots_rem = p_dmx->buf_size - p_dmx->slot_idx;
        uint32_t rxfifo_rem = dmx_hal_get_rxfifo_len(&(dmx_context[dmx_num].hal));
        if (rxfifo_rem) {
          // read data into the active buffer
          dmx_hal_readn_rxfifo(&(dmx_context[dmx_num].hal), 
            p_dmx->buffer[p_dmx->buf_idx] + p_dmx->slot_idx, slots_rem);
          p_dmx->slot_idx += rxfifo_rem;
        }
      } else {
        // there are more bytes than there is space for in the buffer
        uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
      }
      
      // handle data received condition
      if (uart_intr_status & UART_INTR_RXFIFO_TOUT) {
        // disable the rxfifo tout interrupt
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      } else if (uart_intr_status & UART_INTR_RXFIFO_FULL) {
        // enable the rxfifo tout interrupt
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      }

      // handle end-of-frame conditions
      if (uart_intr_status & DMX_INTR_RX_BRK) {
        // handle dmx break (start of frame)

        if (!p_dmx->rx_frame_err && p_dmx->queue) {
          dmx_event_t event = {.size = p_dmx->slot_idx};
          // TODO: check for packets that are too long?
          event.type = DMX_OK;
          xQueueSendFromISR(p_dmx->queue, (void *)&event, &task_awoken);
        }

        // switch buffers, reset error, and reset the slot counter
        p_dmx->buf_idx = !p_dmx->buf_idx;
        p_dmx->rx_frame_err = false;
        p_dmx->slot_idx = 0;

      } else if (uart_intr_status & DMX_INTR_RX_ERR) {
        // handle rx FIFO overflow or parity error

        // if no error has been reported and the queue exists, report an error
        if (!p_dmx->rx_frame_err && p_dmx->queue) {
          dmx_event_t event = {.size = p_dmx->slot_idx};
          if (uart_intr_status & UART_INTR_RXFIFO_OVF)
            event.type = DMX_ERR_PACKET_OVERFLOW;
          else event.type = DMX_ERR_LOST_SIGNAL; // TODO: throw appropriate error for parity err
          xQueueSendFromISR(p_dmx->queue, (void *)&event, &task_awoken);
        }

        p_dmx->rx_frame_err = true;
      }

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | DMX_INTR_RX_BRK | DMX_INTR_RX_ERR));
    } else {
      uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), uart_intr_status);
      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), uart_intr_status);
    }
  }
  
  gpio_set_level(33, 0);  // TODO: for debugging
  if (task_awoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}