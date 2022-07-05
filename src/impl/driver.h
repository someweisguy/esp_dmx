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

#define DMX_CONTEX_INIT_DEF(uart_num)                              \
  {                                                                \
    .hal.dev = UART_LL_GET_HW(uart_num),                           \
    .spinlock = portMUX_INITIALIZER_UNLOCKED, .hw_enabled = false, \
  }

/* This is the DMX driver object used to handle tx'ing and rx'ing DMX data on
the UART port. It stores all the information needed to run and analyze DMX
including the buffer used as an intermediary to store reads/writes on the UART
bus. */
typedef struct {
  dmx_port_t dmx_num;             // The driver's DMX port number.
  rst_seq_hw_t rst_seq_hw;        // The hardware being used to transmit the DMX reset sequence. Can be either the UART or an ESP32 timer group.
  timer_idx_t timer_idx;          // The timer index being used for the reset sequence.
  intr_handle_t uart_isr_handle;  // The handle to the DMX UART ISR.

  uint16_t buf_size;              // Size of the DMX buffer in bytes.
  uint8_t *buffer;                // Used for reading or writing DMX data.
  int16_t slot_idx;               // Index of the current slot that is being rx'd or tx'd.

  // TODO: replace variables with single variable with flags
  dmx_mode_t mode;                // The mode the driver is in - either READ or WRITE.
  bool awaiting_response;
  
  /* These variables are used when transmitting DMX. */
  struct {
    SemaphoreHandle_t sent_sem;     // Signals that the packet has been fully sent. The DMX driver is ready to send another.
    StaticSemaphore_t sent_sem_buf;

    uint32_t break_len;       // Length in microseconds of the transmitted break.
    uint32_t mab_len;         // Length in microseconds of the transmitted mark-after-break;
    int64_t last_data;

    // TODO: every tx variable below this comment can be unionized with rx variables

    uint16_t size;            // The size of the number of slots to send.
  } tx;

  /* These variables are used when receiving DMX. */
  struct {
    QueueHandle_t queue;          // The queue to report DMX received events.
    int16_t size_guess;           // The guess of the size of the packet. Can reduce latency in reporting new data.

    // TODO: every rx variable below this comment can be unionized with tx variables

    gpio_num_t intr_io_num;       // The GPIO number of the DMX sniffer interrupt pin.
    bool event_sent;              // True if a queue event has been sent.
    
    /* The remaining variables are only used if the DMX sniffer is enabled.
    They are uninitialized until dmx_sniffer_enable is called. */
    bool is_in_brk;               // True if the received DMX packet is currently in a break.
    int32_t break_len;            // Length in microseconds of the last received break. Is always -1 unless the DMX sniffer is enabled.
    int32_t mab_len;              // Length in microseconds of the last received mark-after-break. Is always -1 unless the DMX sniffer is enabled.
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
