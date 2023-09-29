/**
 * @file struct.h
 * @author Mitch Weisbrod
 * @brief This file contains the definition for the DMX driver. This file is not
 * considered part of the API and should not be included by the user.
 */
#pragma once

#include <stdint.h>

#include "dmx/config.h"
#include "dmx_types.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "rdm/responder.h"
#include "rdm_types.h"
#include "rdm_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DMX_USE_SPINLOCK
#define DMX_SPINLOCK(n) (&dmx_driver[(n)]->spinlock)
#else
#define DMX_SPINLOCK(n)
#endif

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
  DMX_FLAGS_RDM_IS_RECIPIENT = BIT3,  // The RDM packet is addressed to this device.
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
  dmx_port_t dmx_num;  // The driver's DMX port number.

  // Synchronization state
  SemaphoreHandle_t mux;      // The handle to the driver mutex which allows multi-threaded driver function calls.
  TaskHandle_t task_waiting;  // The handle to a task that is waiting for data to be sent or received.

#ifdef DMX_USE_SPINLOCK
  dmx_spinlock_t spinlock;  // The spinlock used for critical sections.
#endif

  // Data buffer
  int16_t head;     // The index of the slot being transmitted or received.
  uint8_t *data;    // The buffer that stores the DMX packet.
  int16_t tx_size;  // The size of the outgoing packet.
  int16_t rx_size;  // The expected size of the incoming packet.

  // Driver state
  uint8_t flags;     // Flags which indicate the current state of the driver.
  uint8_t rdm_type;  // Flags which indicate the RDM type of the most recent packet.
  uint8_t tn;  // The current RDM transaction number. Is incremented with every RDM packet sent.
  int64_t last_slot_ts;  // The timestamp (in microseconds since boot) of the last slot of the previous data packet.

  // DMX configuration
  struct dmx_personality_t {
    uint16_t footprint;       // The DMX footprint of the personality.
    const char *description;  // A description of the personality.
  } personalities[DMX_PERSONALITY_COUNT_MAX];
  uint32_t break_len;  // Length in microseconds of the transmitted break.
  uint32_t mab_len;  // Length in microseconds of the transmitted mark-after-break.

  // Parameter data
  void *pd;        // Allocated memory for DMX/RDM parameter data.
  size_t pd_size;  // The size of the allocated memory.
  size_t pd_head;  // The amount of memory currently used for parameters.

  // RDM responder configuration
  size_t num_rdm_cbs;            // The number of RDM callbacks registered.
  struct rdm_cb_table_t {
    rdm_pid_description_t desc;  // The parameter description.
    void *param;                 // A pointer to the parameter data.
    const char *param_str;       // A parameter string describing the data.
    rdm_driver_cb_t driver_cb;   // The driver-side callback function.
    rdm_responder_cb_t user_cb;  // The user-side callback function.
    void *context;               // The contexted for the user-side callback.
    bool nvs;                    // True if the parameter is non-volatile.
  } rdm_cbs[RDM_RESPONDER_PIDS_MAX];  // A table containing information on RDM callbacks.

  uint16_t rdm_queue_last_sent;  // The PID of the last sent queued message.
  uint16_t rdm_queue_size;  // The index of the RDM message queue list.
  uint16_t rdm_queue[RDM_RESPONDER_MAX_QUEUE_SIZE];  // The RDM queued message list.

  // DMX sniffer configuration
  dmx_metadata_t metadata;  // The metadata received by the DMX sniffer.
  QueueHandle_t metadata_queue;  // The queue handle used to receive sniffer data.
  int64_t last_pos_edge_ts;  // Timestamp of the last positive edge on the sniffer pin.
  int64_t last_neg_edge_ts;  // Timestamp of the last negative edge on the sniffer pin.
} dmx_driver_t;

extern dmx_port_t rdm_binding_port;
extern rdm_uid_t rdm_device_uid;
extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];

#ifdef __cplusplus
}
#endif
