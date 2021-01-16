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
  const int64_t now = esp_timer_get_time();
  dmx_obj_t *const p_dmx = (dmx_obj_t *)arg;
  const dmx_port_t dmx_num = p_dmx->dmx_num;
  portBASE_TYPE HPTaskAwoken = 0;

  while (true) {
    const uint32_t uart_intr_status = uart_hal_get_intsts_mask(&(dmx_context[dmx_num].hal));
    if (uart_intr_status == 0) break;

    // DMX Transmit #####################################################
    if (uart_intr_status & (UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE)) {
      // this interrupt is triggered when the tx FIFO is empty, or the mark after break is done

      uint32_t bytes_written;
      const uint8_t *buffer_offset = p_dmx->tx_buffer + p_dmx->tx_slot_idx;
      uart_hal_write_txfifo(&(dmx_context[dmx_num].hal), buffer_offset,
          p_dmx->tx_buffer_size - p_dmx->tx_slot_idx, &bytes_written);
      p_dmx->tx_slot_idx += bytes_written;

      // check if frame has been fully written
      if (p_dmx->tx_slot_idx == p_dmx->tx_buffer_size) {
        // TODO: release frame written mutex for frame synchronization

        // allow tx fifo to empty, break and idle will be written
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      }

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), (UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE));
    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      // this interrupt is triggered when the last byte in tx fifo is written

      p_dmx->tx_last_brk_ts = now;  // track break-to-break to ensure continuous data stream

      xSemaphoreGiveFromISR(p_dmx->tx_done_sem, &HPTaskAwoken);

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      // this interrupt is triggered when the UART break is done

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);
    }

    // DMX Recieve ####################################################
    else if (uart_intr_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | DMX_INTR_RX_BRK)) {
      // this interrupt is triggered when the rx FIFO is full or times out, or a break is detected

      // fetch data from rx FIFO
      if (p_dmx->rx_slot_idx < p_dmx->rx_buffer_size) {
        const uint16_t frame_rem = p_dmx->rx_buffer_size - p_dmx->rx_slot_idx;
        int bytes_read = dmx_hal_readn_rxfifo(&(dmx_context[dmx_num].hal), p_dmx->rx_buffer, frame_rem);
        p_dmx->rx_slot_idx += bytes_read;
      } else {
        // the dmx driver buffer size is smaller than the frame we received
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        // TODO: post frame overflow event
      }
      
      // handle break detection and frame timing
      if (uart_intr_status & DMX_INTR_RX_BRK) {
        // received data break on rx FIFO

        // check if break was received in time
        const int64_t brk_to_brk_len = now - p_dmx->rx_last_brk_ts;
        if (brk_to_brk_len < 1196 || brk_to_brk_len > 1250000) {
          // TODO: break was received either too quickly or not quick enough
        }
        p_dmx->rx_last_brk_ts = now;
        
        p_dmx->rx_slot_idx = 0;  // reset slot counter
        // TODO: release frame received mutex
      } else if (uart_intr_status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT)) {
        // received data on rx FIFO

        // figure out when the last byte was received
        int64_t last_byte_rxd;
        if (uart_intr_status & UART_INTR_RXFIFO_FULL) {
          last_byte_rxd = now;

          // reenable the rxfifo timeout interrupt if necessary
          const uint32_t uart_ena_status = uart_hal_get_intr_ena_status(&(dmx_context[dmx_num].hal));
          if (!(uart_ena_status & UART_INTR_RXFIFO_TOUT) && dmx_hal_get_rx_tout(&(dmx_context[dmx_num].hal))) {
            DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
            uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
            DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
          }
        } else {
          uint32_t baudrate;
          uart_hal_get_baudrate(&(dmx_context[dmx_num].hal), &baudrate);
          /* If the baudrate is >246912, it takes ~4.05us to send 1 bit, which
          rounds to 45us to send 11 bits. <252525 is ~3.96us, which rounds to
          43us for 11 bits. Anything in between should take ~44us. Most of the
          time the baudrate should be 250k, which takes EXACTLY 44us anyway. */
          const uint8_t word_speed = baudrate > 246912 ? 45 : baudrate < 252525 ? 43 : 44;
          last_byte_rxd = now - (dmx_hal_get_rx_tout(&(dmx_context[dmx_num].hal)) * word_speed);

          // disable the rxfifo tout interrupt while we're here
          DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
          uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
          DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        }

        const int64_t mrk_between_slots_len = last_byte_rxd - p_dmx->rx_last_byte_ts;
        if (mrk_between_slots_len > 999999) {
          // TODO: mark between slots was not quick enough
        }
      }

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | DMX_INTR_RX_BRK));
    } else if (uart_intr_status & DMX_INTR_RX_ERR) {
      // this interrupt is triggered when the rx FIFO overflows, or if there is a parity error
      
      // handle frame error state
      p_dmx->rx_valid_len = p_dmx->rx_slot_idx;  // track valid frame len
      p_dmx->rx_slot_idx = -1;                   // can't track the slot anymore
      // TODO: post data error event
      
      // flush the rx fifo
      DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
      DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), DMX_INTR_RX_ERR);
    }
  }
  
  gpio_set_level(33, 0);  // TODO: for debugging
  if (HPTaskAwoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}