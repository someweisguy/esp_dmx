/**
 * @file driver.h
 * @author Mitch Weisbrod
 * @brief This file contains the definition for the DMX driver. It is intended
 * to be obfuscated from end-users, but may be included when forking this
 * library or adding new features.
 */
#pragma once

#include <stdint.h>

#include "dmx/types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/uart_hal.h"
#include "rdm/types.h"
#include "rdm/utils.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/gptimer.h"
#else
#include "driver/timer.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_RDM_RESPONDER_MAX_PARAMETERS
#define CONFIG_RDM_RESPONDER_MAX_PARAMETERS 16
#endif
/** @brief The maximum number of parameters that the RDM responder can
 * support. This value is editable in the Kconfig.*/
#define RDM_RESPONDER_MAX_PIDS (8 + CONFIG_RDM_RESPONDER_MAX_PARAMETERS)

/** @brief Used for argument checking at the beginning of each function.*/
#define DMX_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

enum dmx_interrupt_mask_t {
  DMX_INTR_RX_FIFO_OVERFLOW = UART_INTR_RXFIFO_OVF,
  DMX_INTR_RX_FRAMING_ERR = UART_INTR_PARITY_ERR | UART_INTR_FRAM_ERR,
  DMX_INTR_RX_ERR = DMX_INTR_RX_FIFO_OVERFLOW | DMX_INTR_RX_FRAMING_ERR,

  DMX_INTR_RX_BREAK = UART_INTR_BRK_DET,
  DMX_INTR_RX_DATA = UART_INTR_RXFIFO_FULL,
  DMX_INTR_RX_ALL = DMX_INTR_RX_DATA | DMX_INTR_RX_BREAK | DMX_INTR_RX_ERR,

  DMX_INTR_TX_DATA = UART_INTR_TXFIFO_EMPTY,
  DMX_INTR_TX_DONE = UART_INTR_TX_DONE,
  DMX_INTR_TX_ALL = DMX_INTR_TX_DATA | DMX_INTR_TX_DONE,

  DMX_ALL_INTR_MASK = -1
};

enum rdm_packet_timing_t {
  RDM_DISCOVERY_NO_RESPONSE_PACKET_SPACING = 5800,
  RDM_REQUEST_NO_RESPONSE_PACKET_SPACING = 3000,
  RDM_BROADCAST_PACKET_SPACING = 176,
  RDM_RESPOND_TO_REQUEST_PACKET_SPACING = 176,

  RDM_CONTROLLER_RESPONSE_LOST_TIMEOUT = 2800,
  RDM_RESPONDER_RESPONSE_LOST_TIMEOUT = 2000
};

enum dmx_flags_t {
  DMX_FLAGS_DRIVER_IS_ENABLED = BIT0,   // The driver is enabled.
  DMX_FLAGS_DRIVER_IS_IDLE = BIT1,      // The driver is not sending data.
  DMX_FLAGS_DRIVER_IS_SENDING = BIT2,   // The driver is sending.
  DMX_FLAGS_DRIVER_SENT_LAST = BIT3,    // The driver sent the last packet.
  DMX_FLAGS_DRIVER_IS_IN_BREAK = BIT4,  // The driver is in a DMX break.
  DMX_FLAGS_DRIVER_IS_IN_MAB = BIT5,    // The driver is in a DMX MAB.
  DMX_FLAGS_DRIVER_HAS_DATA = BIT6,     // The driver has an unhandled packet.
  DMX_FLAGS_DRIVER_DISC_MUTED = BIT7,   // The driver discovery is muted.
  DMX_FLAGS_DRIVER_BOOT_LOADER = BIT8,  // An error occurred with the driver.
  DMX_FLAGS_TIMER_IS_RUNNING = BIT9,    // The driver hardware timer is running.

  DMX_FLAGS_RDM_IS_VALID = BIT0,
  DMX_FLAGS_RDM_IS_REQUEST = BIT1,
  DMX_FLAGS_RDM_IS_BROADCAST = BIT2,
  DMX_FLAGS_RDM_IS_RECIPIENT = BIT3,  // is addressed to me
  DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH = BIT4,
};

/** @brief The DMX driver object used to handle reading and writing DMX data on
 * the UART port. It storese all the information needed to run and analyze DMX
 * and RDM.*/
typedef struct dmx_driver_t {
  dmx_port_t dmx_num;  // The driver's DMX port number.
  uart_dev_t *uart;               // A pointer to the UART port.
  intr_handle_t uart_isr_handle;  // The handle to the DMX UART ISR.
  SemaphoreHandle_t mux;      // The handle to the driver mutex which allows multi-threaded driver function calls.
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_handle_t gptimer_handle;  // The general purpose timer to use for DMX functions.
#else
  timer_group_t timer_group;  // The timer group to use for DMX functions.
  timer_idx_t timer_idx;      // The timer index to use for DMX functions.
#endif
  TaskHandle_t task_waiting;  // The handle to a task that is waiting for data to be sent or received.

  uint32_t break_len;  // Length in microseconds of the transmitted break.
  uint32_t mab_len;    // Length in microseconds of the transmitted mark-after-break;

  struct dmx_data_t {
    int head;                  // The index of the current slot being either transmitted or received.
    uint8_t *restrict buffer;  // The buffer that stores the DMX packet.
    int tx_size;            // The size of the outgoing data packet.
    int rx_size;            // The expected size of the incoming data packet.

    int64_t timestamp;  // The timestamp (in microseconds since boot) of the last slot of the previous data packet.
  } data;

  uint8_t rdm_type;
  uint16_t flags;

  struct dmx_personality_t {
    uint16_t footprint;
    const char *description;
  } personalities[CONFIG_DMX_MAX_PERSONALITIES];


  struct rdm_info_t {
    uint32_t tn;             // The current RDM transaction number. Is incremented with every RDM packet sent.
    rdm_device_info_t device_info;    // The RDM device info of this device.

    uint32_t num_cbs;
    struct rdm_cb_table_t {
      rdm_pid_description_t desc;
      rdm_response_cb_t cb;
      void *param;
      size_t len;  // TODO: can we remove this?
      void *context;
    } cbs[RDM_RESPONDER_MAX_PIDS];
  } rdm;

  struct dmx_sniffer_t {
    QueueHandle_t queue;       // The queue handle used to receive sniffer data.
    dmx_metadata_t data;       // The metadata received by the DMX sniffer.

    int intr_pin;              // The GPIO number of the DMX sniffer interrupt pin.
    int64_t last_pos_edge_ts;  // Timestamp of the last positive edge on the sniffer pin.
    int64_t last_neg_edge_ts;  // Timestamp of the last negative edge on the sniffer pin.
  } sniffer;
} dmx_driver_t;

#ifdef __cplusplus
}
#endif
