#pragma once

#include "driver/dmx.h"
#include "hal/uart_hal.h"
#include "soc/uart_caps.h"
#include "freertos/semphr.h"

#define DMX_CONTEX_INIT_DEF(dmx_num)                               \
  {                                                                \
    .hal.dev = UART_LL_GET_HW(dmx_num),                            \
    .spinlock = portMUX_INITIALIZER_UNLOCKED, .hw_enabled = false, \
  }

/* This is the DMX driver object used to handle tx'ing and rx'ing DMX data on
the UART port. It stores all the information needed to run and analyze DMX
including the double-buffer used as an intermediary to store reads/writes on
the UART bus. */
typedef struct {
  dmx_port_t dmx_num;             // The driver's DMX port.
  QueueHandle_t queue;            // The queue to report DMX received events.
  intr_handle_t intr_handle;      // The handle to the DMX rx/tx ISR.

  uint16_t buf_size;              // Size of the DMX buffer in bytes.
  uint8_t *buffer[2];             // Used for reading or writing DMX data (double-buffered).
  uint16_t slot_idx;              // Index of the current slot that is being rx'd or tx'd.
  uint8_t buf_idx;                // Index of the currently active buffer that is being rx'd into.
  dmx_mode_t mode;                // The mode the driver is in - either RX or TX.

  int64_t rx_last_brk_ts;         // Timestamp of the last rx'd break.

  SemaphoreHandle_t tx_done_sem;  // Signals the frame has finished being tx'd.
  int64_t tx_last_brk_ts;         // Timestamp of the last tx'd break.
} dmx_obj_t;

dmx_obj_t *p_dmx_obj[DMX_NUM_MAX] = {0};


/* This is the DMX hardware context. It is used to track which hardware has
been initialized as well as to store spinlocks and pointers to UART hardware
registers used by the HAL. */
typedef struct {
  uart_hal_context_t hal;
  portMUX_TYPE spinlock;
  bool hw_enabled;
} dmx_context_t;

dmx_context_t dmx_context[DMX_NUM_MAX] = {
    DMX_CONTEX_INIT_DEF(DMX_NUM_0),
    DMX_CONTEX_INIT_DEF(DMX_NUM_1),
#if DMX_NUM_MAX > 2
    DMX_CONTEX_INIT_DEF(DMX_NUM_2),
#endif
};

#undef DMX_CONTEX_INIT_DEF
