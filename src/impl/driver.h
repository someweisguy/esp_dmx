#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_dmx.h"
#include "esp_intr_alloc.h"
#include "freertos/semphr.h"
#include "hal/uart_hal.h"
#include "soc/uart_struct.h"

#define DMX_CONTEX_INIT_DEF(uart_num) {\
    .hal.dev = UART_LL_GET_HW(uart_num),\
    .spinlock = portMUX_INITIALIZER_UNLOCKED,\
    .hw_enabled = false,\
}

/* This is the DMX driver object used to handle tx'ing and rx'ing DMX data on
the UART port. It stores all the information needed to run and analyze DMX
including the double-buffer used as an intermediary to store reads/writes on
the UART bus. */
typedef struct {
  dmx_port_t dmx_num;             // The driver's DMX port.
  dmx_mode_t mode;                // The mode the driver is in - either RX or TX.
  timer_group_t timer_group;      // The timer group being used for the reset sequence. Is -1 when using reset-sequence-first mode.

  uint16_t buf_size;              // Size of the DMX buffer in bytes.
  uint8_t *buffer[2];             // Used for reading or writing DMX data (double-buffered).
  uint8_t buf_idx;                // Index of the currently active buffer that is being rx'd or tx'd.
  uint16_t slot_idx;              // Index of the current slot that is being rx'd or tx'd.
  
  intr_handle_t intr_handle;      // The handle to the DMX rx/tx ISR.
  
  /* These variables are used when transmitting DMX. */
  struct {
    SemaphoreHandle_t done_sem;   // Signals the frame has finished being tx'd.
    uint16_t size;                // The size of the number of slots to send.
    union {
      /* This struct is used when sending DMX in reset-sequence-first mode. */
      struct {
        timer_idx_t timer_idx;    // The timer index being used for the reset sequence.
        uint32_t break_len;       // Length in microseconds of the last transmitted break.
        uint32_t mab_len;         // Length in microseconds of the last transmitted mark-after-break;
        uint8_t step;             // The current step in the DMX reset sequence. 
      };

      /* This struct is used when sending DMX in reset-sequence-last mode.*/
      struct {
        int64_t last_break_ts;    // Timestamp of the last transmitted break.
      };
    };
  } tx;

  /* These variables are used when receiving DMX. */
  struct {
    QueueHandle_t queue;          // The queue to report DMX received events.
    gpio_num_t intr_io_num;       // The GPIO number of the DMX sniffer interrupt pin.
    int64_t last_break_ts;        // The timestamp of the last received break.
    int32_t break_len;            // Length in microseconds of the last received break. Is always -1 unless the DMX sniffer is enabled.
    int32_t mab_len;              // Length in microseconds of the last received mark-after-break. Is always -1 unless the DMX sniffer is enabled.
    
    /* These variables are only used if the DMX sniffer is enabled. They
    are uninitialized until dmx_sniffer_enable() is called. */
    bool is_in_brk;               // True if the received DMX packet is currently in a break.
    int64_t last_pos_edge_ts;     // Timestamp of the last positive edge on the sniffer pin.
    int64_t last_neg_edge_ts;     // Timestamp of the last negative edge on the sniffer pin.
  } rx;
} dmx_driver_t;

static IRAM_ATTR dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {0};

typedef struct {
    uart_hal_context_t hal;
    portMUX_TYPE spinlock;
    bool hw_enabled;
} dmx_context_t;

static dmx_context_t dmx_context[DMX_NUM_MAX] = {
    DMX_CONTEX_INIT_DEF(DMX_NUM_0),
    DMX_CONTEX_INIT_DEF(DMX_NUM_1),
#if DMX_NUM_MAX > 2
    DMX_CONTEX_INIT_DEF(DMX_NUM_2),
#endif
};

#ifdef __cplusplus
}
#endif
