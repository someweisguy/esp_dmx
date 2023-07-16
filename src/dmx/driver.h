/**
 * @file driver.h
 * @author Mitch Weisbrod
 * @brief This file contains the definition for the DMX driver. It is intended
 * to be obfuscated from end-users, but may be included when forking this
 * library or adding new features.
 */
#pragma once

#include <stdint.h>

#include "dmx/caps.h"
#include "dmx/types.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "hal/uart_hal.h"
#include "rdm/types.h"
#include "rdm/utils.h"
#include "rdm/responder.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/gptimer.h"
#else
#include "driver/timer.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Used for argument checking at the beginning of each function.*/
#define DMX_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)


#define DMX_SPINLOCK(n) (&dmx_driver[(n)]->spinlock)  // TODO: conditionally compile

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
  DMX_FLAGS_DRIVER_BOOT_LOADER = BIT7,  // An error occurred with the driver.

  DMX_FLAGS_RDM_IS_VALID = BIT0,      // The RDM packet is valid.
  DMX_FLAGS_RDM_IS_REQUEST = BIT1,    // The RDM packet is a request.
  DMX_FLAGS_RDM_IS_BROADCAST = BIT2,  // The RDM packet is a broadcast.
  DMX_FLAGS_RDM_IS_RECIPIENT =BIT3,   // The RDM packet is addressed to this device.
  DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH = BIT4,  // The RDM packet is a DISC_UNIQUE_BRANCH.
};

/**
 * @brief Stores the DMX personality information of the DMX driver when RDM is 
 * not enabled.*/
typedef struct dmx_driver_personality_t {
  uint16_t dmx_start_address;   // The driver's DMX start address.
  uint8_t current_personality;  // The current personality of the DMX driver.
  uint8_t personality_count;    // The number of personalities supported.
} dmx_driver_personality_t;

/** @brief The DMX driver object used to handle reading and writing DMX data on
 * the UART port. It storese all the information needed to run and analyze DMX
 * and RDM.*/
typedef struct dmx_driver_t {
  // UART configuration
  dmx_port_t dmx_num;  // The driver's DMX port number.

  // Synchronization state
  SemaphoreHandle_t mux;      // The handle to the driver mutex which allows multi-threaded driver function calls.
  spinlock_t spinlock;  // TODO: conditionally compile
  TaskHandle_t task_waiting;  // The handle to a task that is waiting for data to be sent or received.

  // Data buffer
  int16_t head;     // The index of the slot being transmitted or received.
  uint8_t *data;    // The buffer that stores the DMX packet.
  int16_t tx_size;  // The size of the outgoing packet.
  int16_t rx_size;  // The expected size of the incoming packet.

  // Driver state
  uint8_t flags;
  uint8_t rdm_type;
  uint8_t tn;            // The current RDM transaction number. Is incremented with every RDM packet sent.
  int64_t last_slot_ts;  // The timestamp (in microseconds since boot) of the last slot of the previous data packet.

  // DMX configuration
  struct dmx_personality_t {
    uint16_t footprint;       // The DMX footprint of the personality.
    const char *description;  // A description of the personality.
  } personalities[DMX_PERSONALITIES_MAX];
  uint32_t break_len;  // Length in microseconds of the transmitted break.
  uint32_t mab_len;    // Length in microseconds of the transmitted mark-after-break.

  uint8_t *alloc_data;  // Allocated memory for DMX/RDM parameter data.
  size_t alloc_size;    // The size of the allocated memory.
  size_t alloc_head;    // The amount of memory currently used for parameters.

  // RDM responder configuration
  uint16_t num_rdm_cbs;
  struct rdm_cb_table_t {
    rdm_pid_description_t desc;
    void *param;
    const char *param_str;
    rdm_driver_cb_t driver_cb;
    rdm_responder_cb_t user_cb;
    void *context;
  } rdm_cbs[RDM_RESPONDER_PIDS_MAX];

  // DMX sniffer configuration
  dmx_metadata_t metadata;       // The metadata received by the DMX sniffer.
  QueueHandle_t metadata_queue;  // The queue handle used to receive sniffer data.
  int sniffer_pin;               // The GPIO number of the DMX sniffer interrupt pin.
  int64_t last_pos_edge_ts;      // Timestamp of the last positive edge on the sniffer pin.
  int64_t last_neg_edge_ts;      // Timestamp of the last negative edge on the sniffer pin.
} dmx_driver_t;

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];

#ifdef __cplusplus
}
#endif
