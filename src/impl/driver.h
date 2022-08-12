#pragma once

#include "esp_dmx.h"
#include "esp_intr_alloc.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "hal/uart_hal.h"
#include "soc/uart_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMX_CONTEXT_INIT(uart_num)                                 \
  {                                                                \
    .hal.dev = UART_LL_GET_HW(uart_num),                           \
    .spinlock = portMUX_INITIALIZER_UNLOCKED, .hw_enabled = false, \
  }

/**
 * @brief The DMX driver object used to handle reading and writing DMX data on 
 * the UART port. It storese all the information needed to run and analyze DMX
 * and RDM.
 */
typedef __attribute__((aligned(4))) struct {
  dmx_port_t dmx_num;  // The driver's DMX port number.
  int timer_group;     // The timer group being used fro the DMX reset sequence. Can be set to -1 to use busy-waits instead of a hardware timer.
  int timer_num;       // The timer number being used for the DMX reset sequence.

  intr_handle_t uart_isr_handle;  // The handle to the DMX UART ISR.

  uint32_t break_len;  // Length in microseconds of the transmitted break.
  uint32_t mab_len;    // Length in microseconds of the transmitted mark-after-break;

  struct {
    size_t head;      // The index of the current slot being either transmitted or received.
    uint8_t *buffer;  // The buffer that stores the DMX packet.
    size_t tx_size;   // The size of the outgoing data packet.
    size_t rx_size;   // The expected size of the incoming data packet
 
    int previous_type;     // The type of the previous data packet. If the previous packet was an RDM packet, this is equal to its command class.
    int64_t previous_uid;  // The destination UID of the previous packet. Is -1 if the previous packet was not RDM.
    int64_t previous_ts;   // The timestamp (in microseconds since boot) of the last slot of the previous data packet.
    int sent_previous;     // Is true if this device sent the previous data packet.

    dmx_err_t err;  // The error state of the received DMX data.
  } data;

  int is_in_break;      // True if the driver is sending or receiving a DMX break.
  int received_packet;  // True if the driver is receiving data.
  int is_sending;       // True if the driver is sending data.
  int timer_running;    // True if the hardware timer is running.

  TaskHandle_t task_waiting;  // The handle to a task that is waiting for data to be sent or received.
  SemaphoreHandle_t mux;      // The handle to the driver mutex which allows multi-threaded driver function calls.

  struct {
    QueueHandle_t queue;       // The handle to the DMX sniffer queue.
    int intr_io_num;           // The GPIO number of the DMX sniffer interrupt pin.
    int32_t break_len;         // Length in microseconds of the last received break. Is always -1 unless the DMX sniffer is enabled.
    int32_t mab_len;           // Length in microseconds of the last received mark-after-break. Is always -1 unless the DMX sniffer is enabled.
    int64_t last_pos_edge_ts;  // Timestamp of the last positive edge on the sniffer pin.
    int64_t last_neg_edge_ts;  // Timestamp of the last negative edge on the sniffer pin.
  } sniffer;
} dmx_driver_t;

typedef struct {
    uart_hal_context_t hal;
    spinlock_t spinlock;
    int hw_enabled;
} dmx_context_t;

static DRAM_ATTR dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {0};

static DRAM_ATTR dmx_context_t dmx_context[DMX_NUM_MAX] = {
    DMX_CONTEXT_INIT(DMX_NUM_0),
    DMX_CONTEXT_INIT(DMX_NUM_1),
#if DMX_NUM_MAX > 2
    DMX_CONTEXT_INIT(DMX_NUM_2),
#endif
};

#ifdef __cplusplus
}
#endif
