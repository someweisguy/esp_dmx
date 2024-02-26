/**
 * @file dmx/include/service.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file contains the definition for the DMX driver. This file is not
 * considered part of the API and should not be included by the user.
 */
#pragma once

#include <stdint.h>

#include "dmx/include/parameter.h"
#include "dmx/include/types.h"
#include "esp_check.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "rdm/responder/include/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Used for argument checking at the beginning of each function.*/
#define DMX_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

/** @brief Logs a warning message on the terminal if the condition is not met.*/
#define DMX_ERR(format, ...)              \
  do {                                    \
    ESP_LOGE(TAG, format, ##__VA_ARGS__); \
  } while (0);

/** @brief Logs a warning message on the terminal if the condition is not met.*/
#define DMX_WARN(format, ...)             \
  do {                                    \
    ESP_LOGW(TAG, format, ##__VA_ARGS__); \
  } while (0);

#if defined(CONFIG_DMX_ISR_IN_IRAM) || ESP_IDF_VERSION_MAJOR < 5
/** @brief This macro sets certain functions used within DMX interrupt handlers
 * to be placed within IRAM. The current hardware configuration of this device
 * places the DMX driver functions within IRAM. */
#define DMX_ISR_ATTR IRAM_ATTR
/** @brief This macro is used to conditionally compile certain parts of code
 * depending on whether or not the DMX driver is within IRAM.*/
#define DMX_ISR_IN_IRAM
#else
/** @brief This macro sets certain functions used within DMX interrupt handlers
 * to be placed within IRAM. Due to the current hardware configuration of this
 * device, the DMX driver is not currently placed within IRAM. */
#define DMX_ISR_ATTR
#endif

/** @brief Directs the DMX driver to use spinlocks in critical sections. This is
 * needed for devices which have multiple cores.*/
#define DMX_USE_SPINLOCK
#define DMX_SPINLOCK(n) (&dmx_driver[(n)]->spinlock)
typedef spinlock_t dmx_spinlock_t;
#define DMX_SPINLOCK_INIT portMUX_INITIALIZER_UNLOCKED

extern const char *TAG;  // The log tagline for the library.

enum dmx_parameter_type_t {
  DMX_PARAMETER_TYPE_NULL,
  DMX_PARAMETER_TYPE_DYNAMIC,
  DMX_PARAMETER_TYPE_NON_VOLATILE,
  DMX_PARAMETER_TYPE_STATIC,
  DMX_PARAMETER_TYPE_NON_VOLATILE_STAGED,
};

enum {
  DMX_HEAD_WAITING_FOR_BREAK = -1,  // The driver is awaiting a DMX break.

  DMX_PROGRESS_STALE = 0,  // The packet has already been received.
  DMX_PROGRESS_IN_BREAK,   // The packet is in the DMX break.
  DMX_PROGRESS_IN_MAB,     // The packet is in the DMX mark-after-break.
  DMX_PROGRESS_IN_DATA,    // Packet slot data is being sent or received.
  DMX_PROGRESS_COMPLETE,   // The packet is complete.

  DMX_STATUS_IDLE = 0,   // The DMX driver is idle.
  DMX_STATUS_RECEIVING,  // The DMX driver is receiving data.
  DMX_STATUS_SENDING,    // The DMX driver is sending data.
};

/**
 * @brief The DMX parameter type. Contains information necessary for maintaining
 * parameter information as well as RDM response information if necessary.
 */
typedef struct dmx_parameter_t {
  rdm_pid_t pid;  // The parameter ID of the parameter.
  size_t size;    // The size of the parameter in bytes.
  void *data;     // A pointer to the data pertaining to the parameter.
  uint8_t type;  // The storage type of the parameter data. Determines if the parameter is non-volatile or not.
  const rdm_parameter_definition_t *definition;  // The RDM definition of the parameter. Is only needed for RDM responders.
  rdm_callback_t callback;  // A user callback for the parameter. Is only needed for RDM responders.
  void *context;            // Context for the user callback.
} dmx_parameter_t;

/**
 * @brief The DMX device type. Holds an array of parameters associated with the
 * device as well as a linked-list style pointer to the next DMX device.
 */
typedef struct dmx_device_t {
  dmx_device_num_t num;  // The device number.
  struct dmx_device_t *next;  // A pointer to the next device.
  dmx_parameter_t parameters[];  // An array of parameters associated with this device.
} dmx_device_t;

/** @brief The DMX driver object used to handle reading and writing DMX data on
 * the UART port. It stores all the information needed to run and analyze DMX
 * and RDM.*/
typedef struct dmx_driver_t {
  // Driver configuration
  dmx_port_t dmx_num;  // The driver's DMX port number.
  rdm_uid_t uid;       // The driver's UID.
  uint32_t break_len;  // Length in microseconds of the transmitted break.
  uint32_t mab_len;  // Length in microseconds of the transmitted mark-after-break.

  bool is_enabled;  // True if the DMX driver is enabled.
  bool is_controller;  // True if the DMX driver is the controller on the DMX bus.

  // Synchronization state
  SemaphoreHandle_t mux;      // The handle to the driver mutex which allows multi-threaded driver function calls.
  TaskHandle_t task_waiting;  // The handle to a task that is waiting for data to be sent or received.
#ifdef DMX_USE_SPINLOCK
  dmx_spinlock_t spinlock;  // The spinlock used for critical sections.
#endif

  // Data buffer
  struct dmx_driver_dmx_t {
    int head;  // The index of the slot being transmitted or received.
    uint8_t data[DMX_PACKET_SIZE_MAX];  // The buffer that stores the DMX packet.
    int size;  // The expected size of the incoming/outgoing packet.
    int status;  // The status of the DMX port.
    int progress;  // The progress of the current packet.
    rdm_pid_t last_controller_pid;  // The PID of the last controller-generated packet.
    int64_t controller_eop_timestamp;  // The timestamp (in microseconds since boot) of the end-of-packet of the last controller-generated packet.
    rdm_pid_t last_responder_pid;  // The PID of the last responder-generated packet.
    bool responder_sent_last;  // True if the last packet was a responder-generated packet.
    union {
      struct {
        rdm_pid_t last_request_pid;  // The PID of the last packet which targeted this device. Is only used when this device is a DMX responder.
        uint8_t last_request_pid_repeats;  // The number of times the last request targeting this device repeated its PID. Used for PIDs which can generate ACK overflow responses. Is only used when this device is a DMX responder.
      }; 
      bool last_request_was_broadcast;  // True if the last request was a broadcast. Is only used when this device is a DMX controller.
    };
  } dmx;

  // RDM driver information
  struct dmx_driver_rdm_t {
    union {
      uint8_t tn;  // The current RDM transaction number. Is incremented with every RDM request sent.
      bool boot_loader;  // The RDM responder boot-loader flag. True when when the device is incapable of normal operation until receiving a firmware upload.
    };
  } rdm;
  
  // DMX sniffer configuration
  struct dmx_driver_sniffer_t {
    bool is_enabled;
    int buffer_index;
    dmx_metadata_t metadata[2];  // The metadata received by the DMX sniffer.
    int64_t last_pos_edge_ts;  // Timestamp of the last positive edge on the sniffer pin.
    int64_t last_neg_edge_ts;  // Timestamp of the last negative edge on the sniffer pin.
  } sniffer;

  // DMX device information
  struct dmx_driver_device_t {
    struct dmx_driver_parameter_count_t {
      unsigned int root;  // The number of parameters supported by the root device.
      unsigned int sub_devices;  // The number of parameters supported by sub-devices.
      unsigned int staged;  // The number of non-volatile parameters waiting to be committed to non-volatile storage.
    } parameter_count;  // Parameter counts for various purposes.
    dmx_device_t root;  // The root device of the RDM driver.
  } device;
} dmx_driver_t;

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];

// TODO: implement dmx_device_add()
// dmx_device_add(dmx_num, device_num);

/**
 * @brief Gets a pointer to the desired device, if it exists.
 * 
 * @param dmx_num The DMX port number.
 * @param device_num The sub-device number.
 * @return A pointer to the device, or NULL on failure.
 */
dmx_device_t *dmx_device_get(dmx_port_t dmx_num, dmx_device_num_t device_num);

/**
 * @brief Adds a parameter to the DMX driver, if there is space available.
 *
 * @param dmx_num The DMX port number.
 * @param device_num The sub-device number.
 * @param pid The parameter ID.
 * @param type The type of the parameter, one of enum dmx_parameter_type_t.
 * @param data A pointer to the initial value of the data, or the data itself,
 * in the case of DMX_PARAMETER_TYPE_STATIC.
 * @param size The size of the data.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_parameter_add(dmx_port_t dmx_num, dmx_device_num_t device_num,
                       rdm_pid_t pid, int type, void *data, size_t size);

/**
 * @brief Gets a pointer to the desired parameter, if it exists.
 * 
 * @param dmx_num The DMX port number.
 * @param device_num The sub-device number.
 * @param pid The parameter ID.
 * @return A pointer to the parameter, or NULL on failure.
 */
dmx_parameter_t *dmx_parameter_get_entry(dmx_port_t dmx_num,
                                         dmx_device_num_t device_num,
                                         rdm_pid_t pid);

#ifdef __cplusplus
}
#endif
