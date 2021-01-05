#pragma once

#include "driver/dmx_ctrl.h"
#include "esp_system.h"
#include "hal/dmx_hal.h"
#include "hal/uart_hal.h"

#define DMX_ENTER_CRITICAL_ISR(mux) portENTER_CRITICAL_ISR(mux)
#define DMX_EXIT_CRITICAL_ISR(mux)  portEXIT_CRITICAL_ISR(mux)

/**
 * TXing: user inputs into user buffer. driver outputs data on work buffer.
 *   When it is time to sync, the driver will copy the contents of the user
 *   buffer into the work buffer
 * TX Interrrupts: UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE | UART_INTR_TX_BRK_DONE
 *  | UART_INTR_TX_DONE
 *
 * RXing: driver copies the contents of the uart fifo into one buffer. When
 *   it is time to sync, the driver will swap the frame pointer so that the
 *   user will always access the latest complete frame
 */
void dmx_default_intr_handler(void *arg) {
  dmx_obj_t *const p_dmx = (dmx_obj_t *)arg;
  const dmx_port_t dmx_num = p_dmx->dmx_num;
  portBASE_TYPE HPTaskAwoken = 0;

  while (true) {
    const uint32_t uart_intr_status = uart_hal_get_intsts_mask(&(dmx_context[dmx_num].hal));
    if (uart_intr_status == 0) break;

    /* DMX Transmit ################################################### */
    if (uart_intr_status & (UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE)) {
      // write data to tx fifo
      uint32_t bytes_written;
      const uint8_t *buffer_offset = p_dmx->buffer + p_dmx->slot_idx;
      uart_hal_write_txfifo(&(dmx_context[dmx_num].hal), buffer_offset,
          p_dmx->buffer_size - p_dmx->slot_idx, &bytes_written);
      p_dmx->slot_idx += bytes_written;

      // check if frame has been fully written
      if (p_dmx->slot_idx >= p_dmx->buffer_size) {
        // TODO: release frame written mutex for frame synchronization

        // allow tx fifo to empty, break and idle will be written
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        
        p_dmx->slot_idx = 0;  // reset slot counter
      }

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), (UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE));
    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      // this interrupt is triggered when the last byte in tx fifo is written

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_DONE);
    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      // this interrupt is triggered when the UART break is done

      // enable tx fifo empty interrupt to write the next frame
      DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE);
      DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);
    }

    // DMX Recieve ####################################################
    else if (uart_intr_status & (UART_INTR_RXFIFO_FULL | UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR | UART_INTR_BRK_DET | UART_INTR_RXFIFO_TOUT)) {
      // received data on rx fifo

      // TODO: check if data was received in time

      // fetch data from uart fifo
      if (p_dmx->slot_idx < p_dmx->buffer_size) {
        const uint16_t frame_rem = p_dmx->buffer_size - p_dmx->slot_idx;
        int read = dmx_hal_readn_rxfifo(
            &(dmx_context[dmx_num].hal), p_dmx->buffer, frame_rem);
        p_dmx->slot_idx += read;
      } else {
        // the dmx driver buffer size is smaller than the frame we received
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        // TODO: post frame overflow event
      }

      if (uart_intr_status & (UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR | UART_INTR_BRK_DET)) {
        // received data break on rx fifo

        // TODO: check if break was received in time

        p_dmx->slot_idx = 0;  // reset slot counter
        // TODO: mutex here?
      }

      if (uart_intr_status & UART_INTR_RXFIFO_TOUT) {
        // rx timed out waiting for data
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      } else {
        // TODO: only call this if rx_timeout is enabled!!!
        // rx didn't time out AND the rxfifo_tout interrupt is disabled - reenable it
        DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_RXFIFO_TOUT);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      }
      

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), (UART_INTR_RXFIFO_FULL | UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR | UART_INTR_BRK_DET | UART_INTR_RXFIFO_TOUT));
    } else if (uart_intr_status & (UART_INTR_RXFIFO_OVF | UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR)) {
      // uart rx fifo overflow or parity error

      if (uart_intr_status & (UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR)) {
        // TODO: set the valid frame len to the current slot_idx
        // TODO: post data error event
      }

      p_dmx->slot_idx = UINT16_MAX; // can't track the slot anymore
      
      // flush the rx fifo
      DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
      DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));

      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), (UART_INTR_RXFIFO_OVF | UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR));
    }
  }
  
  if (HPTaskAwoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}