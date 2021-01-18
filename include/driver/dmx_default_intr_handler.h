#pragma once

#include "driver/dmx_ctrl.h"
#include "esp_system.h"
#include "hal/dmx_hal.h"
#include "hal/uart_hal.h"

#include "driver/gpio.h" // TODO: for debugging

#define DMX_ENTER_CRITICAL_ISR(mux) portENTER_CRITICAL_ISR(mux)
#define DMX_EXIT_CRITICAL_ISR(mux)  portEXIT_CRITICAL_ISR(mux)

#define DMX_INTR_RX_BRK (UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR | UART_INTR_BRK_DET)
#define DMX_INTR_RX_ERR (UART_INTR_RXFIFO_OVF | UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR)

void dmx_default_intr_handler(void *arg) {
  gpio_set_level(33, 1);  // TODO: for debugging
  int64_t now = esp_timer_get_time();
  dmx_obj_t *const p_dmx = (dmx_obj_t *)arg;
  const dmx_port_t dmx_num = p_dmx->dmx_num;
  portBASE_TYPE HPTaskAwoken = 0;

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

      xSemaphoreGiveFromISR(p_dmx->done_sem, &HPTaskAwoken);

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      // this interrupt is triggered when the UART break is done

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_IDLE) {
      // this interrupt is triggered when the UART mark after break is done

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_IDLE);
    }

    // DMX Recieve ####################################################
    else if (uart_intr_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | DMX_INTR_RX_BRK | DMX_INTR_RX_ERR)) {
      // this interrupt is triggered when any rx event occurs

      const int16_t slots_rem = p_dmx->buf_size - p_dmx->slot_idx;
      uint32_t rxfifo_rem = dmx_hal_get_rxfifo_len(&(dmx_context[dmx_num].hal));
      if (slots_rem > 0 && rxfifo_rem) {
        // read data into the active buffer
        int bytes_read = dmx_hal_readn_rxfifo(&(dmx_context[dmx_num].hal),
          p_dmx->buffer[p_dmx->buf_idx], slots_rem);
        p_dmx->slot_idx += rxfifo_rem;
      }

      if (rxfifo_rem > slots_rem) {
        // there are more bytes than there is space for in the buffer
        uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
      }
      
      if (uart_intr_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT)) {
        // handle data received

        if (uart_intr_status & UART_INTR_RXFIFO_TOUT) {
          // disable the rxfifo tout interrupt
          DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
          uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
          DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        }

      } else if (uart_intr_status & DMX_INTR_RX_BRK) {
        // handle dmx break

        // signal the end of the frame
        xSemaphoreGiveFromISR(p_dmx->done_sem, &HPTaskAwoken);

        // update valid frame length
        if (!p_dmx->rx_frame_err) p_dmx->rx_valid_len = p_dmx->slot_idx;
        p_dmx->rx_frame_err = false;

        // switch buffers and reset the slot counter
        p_dmx->buf_idx = !p_dmx->buf_idx;
        p_dmx->slot_idx = 0;

      } else if (uart_intr_status & DMX_INTR_RX_ERR) {
        // handle rx FIFO overflow or parity error

        p_dmx->rx_valid_len = p_dmx->slot_idx;
        p_dmx->rx_frame_err = true;
      }

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | DMX_INTR_RX_BRK | DMX_INTR_RX_ERR));
    }
  }
  
  gpio_set_level(33, 0);  // TODO: for debugging
  if (HPTaskAwoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}