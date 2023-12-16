/**
 * @file dmx/struct.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file contains the definition for the DMX driver. This file is not
 * considered part of the API and should not be included by the user.
 */
#pragma once

#include <stdint.h>

#include "dmx/hal/include/gpio.h"
#include "dmx/hal/include/timer.h"
#include "dmx/hal/include/uart.h"
#include "dmx/types.h"
#include "esp_check.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "rdm/responder.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief DMX port max. Used for error checking.*/
#define DMX_NUM_MAX SOC_UART_NUM

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

#ifdef CONFIG_RDM_DEVICE_UID_MAN_ID
/** @brief This is the RDM Manufacturer ID used with this library. It may be set
 * using the Kconfig file. The default value is 0x05e0.*/
#define RDM_UID_MANUFACTURER_ID (CONFIG_RDM_DEVICE_UID_MAN_ID)
#else
/** @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID (as
 * long as it is used responsibly) or may choose to register their own
 * manufacturer ID.*/
#define RDM_UID_MANUFACTURER_ID (0x05e0)
#endif

#ifdef CONFIG_RDM_DEVICE_UID_DEV_ID
/** @brief This is the RDM Device ID used with this library. It may be set
 * using the Kconfig file. The default value is a function of this device's MAC
 * address.*/
#define RDM_UID_DEVICE_ID (CONFIG_RDM_DEVICE_UID_DEV_ID)
#else
/** @brief This is the RDM Device ID used with this library. The default value
 * is a function of this device's MAC address.*/
#define RDM_UID_DEVICE_ID (0xffffffff)
#endif

// TODO: docs
#define RDM_RESPONDER_NUM_PIDS_REQUIRED 9

#ifdef CONFIG_RDM_RESPONDER_MAX_OPTIONAL_PARAMETERS
/** @brief The maximum number of optional parameters that the RDM responder can
 * support. This value is editable in the Kconfig.*/
#define RDM_RESPONDER_NUM_PIDS_OPTIONAL \
  (CONFIG_RDM_RESPONDER_MAX_OPTIONAL_PARAMETERS)
#else
/** @brief The maximum number of optional parameters that the RDM responder can
 * support.*/
#define RDM_RESPONDER_NUM_PIDS_OPTIONAL 25
#endif

/** @brief The maximum number of parameters that the RDM responder can
 * support.*/
#define RDM_RESPONDER_NUM_PIDS_MAX \
  (RDM_RESPONDER_NUM_PIDS_REQUIRED + RDM_RESPONDER_NUM_PIDS_OPTIONAL)

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

enum dmx_flags_t {
  DMX_FLAGS_DRIVER_IS_ENABLED = BIT0,   // The driver is enabled.
  DMX_FLAGS_DRIVER_IS_IDLE = BIT1,      // The driver is not sending data.
  DMX_FLAGS_DRIVER_IS_SENDING = BIT2,   // The driver is sending.
  DMX_FLAGS_DRIVER_SENT_LAST = BIT3,    // The driver sent the last packet.
  DMX_FLAGS_DRIVER_IS_IN_BREAK = BIT4,  // The driver is in a DMX break.
  DMX_FLAGS_DRIVER_IS_IN_MAB = BIT5,    // The driver is in a DMX MAB.
  DMX_FLAGS_DRIVER_HAS_DATA = BIT6,     // The driver has an unhandled packet.
  DMX_FLAGS_DRIVER_BOOT_LOADER = BIT7,  // An error occurred with the driver.
};

// TODO: docs
typedef struct rdm_parameter_s {
  rdm_pid_t pid;
  size_t size;
  void *data;
  uint8_t is_heap_allocated;
  uint8_t storage_type;
  bool is_queued;
} rdm_parameter_t;

typedef struct rdm_device_s {
  rdm_sub_device_t device_num;
  struct rdm_device_s *next;
  
  // Device information
  uint16_t model_id;
  uint16_t product_category;
  uint32_t software_version_id;

  rdm_parameter_t parameters[];
} rdm_device_t;

/** @brief The DMX driver object used to handle reading and writing DMX data on
 * the UART port. It storese all the information needed to run and analyze DMX
 * and RDM.*/
typedef struct dmx_driver_t {
  // Driver configuration
  dmx_port_t dmx_num;  // The driver's DMX port number.
  rdm_uid_t uid;       // The driver's UID.
  uint32_t break_len;  // Length in microseconds of the transmitted break.
  uint32_t mab_len;  // Length in microseconds of the transmitted mark-after-break.
  uint8_t flags;     // Flags which indicate the current state of the driver.

  // Driver hardware handles
  dmx_uart_handle_t uart;    // The handle to the UART HAL.
  dmx_timer_handle_t timer;  // The handle to the hardware timer HAL.
  dmx_gpio_handle_t gpio;    // The handle to the GPIO HAL.

  // Synchronization state
  SemaphoreHandle_t mux;      // The handle to the driver mutex which allows multi-threaded driver function calls.
  TaskHandle_t task_waiting;  // The handle to a task that is waiting for data to be sent or received.
#ifdef DMX_USE_SPINLOCK
  dmx_spinlock_t spinlock;  // The spinlock used for critical sections.
#endif

  // Data buffer
  int16_t head;     // The index of the slot being transmitted or received.
  uint8_t data[DMX_PACKET_SIZE_MAX];  // The buffer that stores the DMX packet.
  int16_t tx_size;  // The size of the outgoing packet.
  int16_t rx_size;  // The expected size of the incoming packet.
  int64_t last_slot_ts;  // The timestamp (in microseconds since boot) of the last slot of the previous data packet.
  
  // DMX sniffer configuration
  dmx_metadata_t metadata;  // The metadata received by the DMX sniffer.
  QueueHandle_t metadata_queue;  // The queue handle used to receive sniffer data.
  int64_t last_pos_edge_ts;  // Timestamp of the last positive edge on the sniffer pin.
  int64_t last_neg_edge_ts;  // Timestamp of the last negative edge on the sniffer pin.

  struct rdm_driver_t {
    uint32_t staged_count;
    uint32_t queue_count;       // The index of the RDM message queue list.
    rdm_pid_t previous_popped;  // The PID of the last sent queued message.

    uint8_t tn;  // The current RDM transaction number. Is incremented with every RDM packet sent.

    uint32_t root_device_parameter_max;
    uint32_t sub_device_parameter_max;

    rdm_device_t root_device;
  } rdm;
} dmx_driver_t;

// Parameter data
// TODO: implement status using space in pd
// uint8_t rdm_status_threshold;
// struct rdm_status_queue_t {
//   uint16_t head;  // The next element to pop
//   uint16_t tail;  // The next open space for an element
//   rdm_status_message_t queue[RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX];
// } rdm_status[3];

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];

#ifdef __cplusplus
}
#endif
