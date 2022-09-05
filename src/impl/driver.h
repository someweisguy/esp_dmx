#pragma once

#include <stdint.h>

#include "esp_intr_alloc.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The DMX driver object used to handle reading and writing DMX data on 
 * the UART port. It storese all the information needed to run and analyze DMX
 * and RDM.
 */
typedef __attribute__((aligned(4))) struct dmx_driver {
  dmx_port_t dmx_num;  // The driver's DMX port number.
  int timer_group;     // The timer group being used fro the DMX reset sequence. Can be set to -1 to use busy-waits instead of a hardware timer.
  int timer_num;       // The timer number being used for the DMX reset sequence.

  intr_handle_t uart_isr_handle;  // The handle to the DMX UART ISR.

  uint32_t break_len;  // Length in microseconds of the transmitted break.
  uint32_t mab_len;    // Length in microseconds of the transmitted mark-after-break;

  struct dmx_data {
    size_t head;      // The index of the current slot being either transmitted or received.
    uint8_t *buffer;  // The buffer that stores the DMX packet.
    size_t tx_size;   // The size of the outgoing data packet.
    size_t rx_size;   // The expected size of the incoming data packet.
 
    int previous_type;      // The type of the previous data packet. If the previous packet was an RDM packet, this is equal to its command class.
    int64_t previous_uid;  // The destination UID of the previous packet. Is 0 if the previous packet was not RDM.
    int64_t previous_ts;    // The timestamp (in microseconds since boot) of the last slot of the previous data packet.
    int sent_previous;      // Is true if this device sent the previous data packet.

    dmx_err_t err;  // The error state of the received DMX data.
  } data;

  int is_in_break;      // True if the driver is sending or receiving a DMX break.
  int received_packet;  // True if the driver is receiving data.
  int is_sending;       // True if the driver is sending data.
  int timer_running;    // True if the hardware timer is running.

  uint32_t rdm_tn;   // RDM Transaction number. Increments with every RDM packet sent.
  int rdm_is_muted;  // True if RDM discovery is muted.

  TaskHandle_t task_waiting;  // The handle to a task that is waiting for data to be sent or received.
  SemaphoreHandle_t mux;      // The handle to the driver mutex which allows multi-threaded driver function calls.

  struct dmx_sniffer {
    QueueHandle_t queue;       // The queue handle used to receive sniffer data.
    dmx_sniffer_data_t data;   // The data received by the DMX sniffer.

    int intr_pin;              // The GPIO number of the DMX sniffer interrupt pin.
    int is_in_mab;             // True if the sniffer is receiving a DMX mark-after-break.
    int64_t last_pos_edge_ts;  // Timestamp of the last positive edge on the sniffer pin.
    int64_t last_neg_edge_ts;  // Timestamp of the last negative edge on the sniffer pin.
  } sniffer;
} dmx_driver_t;

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];

#ifdef __cplusplus
}
#endif
