#pragma once

#include "dmx_ctrl.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "hal/uart_hal.h"
#include "hal/uart_ll.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"

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
        uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal),
            UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE);
        DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
        
        p_dmx->slot_idx = 0;  // reset slot counter
      }

      // clear interrupts
      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal),
          (UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE));

    } else if (uart_intr_status & UART_INTR_TX_DONE) {
      // this interrupt is triggered when the last byte in tx fifo is written
      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_DONE);

      // TODO: break gets written here

    } else if (uart_intr_status & UART_INTR_TX_BRK_DONE) {
      // this interrupt is triggered when the UART break is done
      uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);

      // enable tx fifo empty interrupt to write the next frame
      DMX_ENTER_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
      uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal),
          UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_BRK_IDLE);
      DMX_EXIT_CRITICAL_ISR(&(dmx_context[dmx_num].spinlock));
    }



      //uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);
      //uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);
      //uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_TX_BRK_DONE);
    /*
        // DMX Recieve ####################################################
        else if (uart_intr_status & (UART_INTR_RXFIFO_FULL | DMX_RX_BRK_FLAG)) {
          // got data on uart

          // check if data was received in time
          const int64_t data_elapsed = now - driver->last_data;
          if (data_elapsed > DMX_RX_MAX_MARK_TIME_BETWEEN_SLOTS)
            driver->frame_ok[driver->buf_idx] = false;
          driver->last_data = now;

          // fetch data from uart fifo
          uint32_t rx_fifo_rem = UART->status.rxfifo_cnt;
          while (rx_fifo_rem && driver->chn_idx < driver->buf_len) {
            // read data from fifo
            driver->buf[driver->buf_idx][driver->chn_idx++] =
       UART->fifo.rw_byte;
            --rx_fifo_rem;
          }
          if (rx_fifo_rem) {
            // driver buffer overflow
            do {
              UART->fifo.rw_byte;  // discard remaining data
            } while (--rx_fifo_rem);
          }

          if (uart_intr_status & DMX_RX_BRK_FLAG) {
            // got break on uart

            // check if break was received in time
            const int64_t brk_elapsed = now - driver->last_brk;
            if (brk_elapsed > DMX_RX_MAX_BREAK_TO_BREAK_TIME)
              driver->frame_ok[driver->buf_idx] = false;

            driver->last_brk = now;

            driver->chn_idx = 0;  // reset channel counter

            // sync data
            // TODO: mutex here?
            driver->buf_idx = !driver->buf_idx;
            driver->frame_ok[driver->buf_idx] = true;
          }
          uart_clear_intr_status(dmx_num, UART_INTR_RXFIFO_FULL |
       DMX_RX_BRK_FLAG);

        } else if (uart_intr_status & UART_INTR_RXFIFO_OVF) {
          // uart fifo overflow

          // invalidate frame and stop fetching data from uart fifo
          driver->frame_ok[driver->buf_idx] = false;
          driver->chn_idx = UINT16_MAX;

          uart_clear_intr_status(dmx_num, UART_INTR_RXFIFO_OVF);
        }
        */
  }
  if (HPTaskAwoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}