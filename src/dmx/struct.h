/**
 * @file dmx/struct.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file contains the definition for the DMX driver. This file is not
 * considered part of the API and should not be included by the user.
 */
#pragma once

#include <stdint.h>

#include "dmx/hal/gpio.h"
#include "dmx/hal/timer.h"
#include "dmx/hal/uart.h"
#include "dmx/types.h"
#include "esp_check.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "rdm/responder.h"
#include "rdm/types.h"
#include "rdm/utils/pd.h"

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

/** @brief Macro used to convert milliseconds to FreeRTOS ticks. Evaluates to
 * the minimum number of ticks needed for the specified number of milliseconds
 * to elapse.*/
#define pdDMX_MS_TO_TICKS(ms)                               \
  (pdMS_TO_TICKS(ms) +                                      \
   (((TickType_t)(ms) * (TickType_t)(configTICK_RATE_HZ)) % \
        (TickType_t)1000U >                                 \
    0))

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

#define RDM_RESPONDER_NUM_PIDS_REQUIRED 9

#ifdef CONFIG_RDM_RESPONDER_MAX_OPTIONAL_PARAMETERS
/** @brief The maximum number of optional parameters that the RDM responder can
 * support. This value is editable in the Kconfig.*/
#define RDM_RESPONDER_NUM_PIDS_OPTIONAL \
  (CONFIG_RDM_RESPONDER_MAX_OPTIONAL_PARAMETERS)
#else
#define RDM_RESPONDER_NUM_PIDS_OPTIONAL 25
#endif

/** @brief The maximum number of parameters that the RDM responder can
 * support.*/
#define RDM_RESPONDER_NUM_PIDS_MAX \
  (RDM_RESPONDER_NUM_PIDS_REQUIRED + RDM_RESPONDER_NUM_PIDS_OPTIONAL)

#ifdef CONFIG_RDM_RESPONDER_QUEUE_SIZE_MAX
/** @brief The maximum number of queued messages that the RDM responder can
 * support. It may be set using the Kconfig file.
 */
#define RDM_RESPONDER_QUEUE_SIZE_MAX CONFIG_RDM_RESPONDER_QUEUE_SIZE_MAX
#else
/** @brief The maximum number of queued messages that the RDM responder can
 * support.
 */
#define RDM_RESPONDER_QUEUE_SIZE_MAX 64
#endif

#ifdef CONFIG_RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX
/** @brief The maximum number of queued status messages that the RDM responder can
 * support. It may be set using the Kconfig file.
 */
#define RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX \
  CONFIG_RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX
#else
/** @brief The maximum number of queued status messages that the RDM responder
 * can support.
 */
#define RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX 64
#endif

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

  dmx_uart_handle_t uart;    // The handle to the UART HAL.
  dmx_timer_handle_t timer;  // The handle to the hardware timer HAL.
  dmx_gpio_handle_t gpio;    // The handle to the GPIO HAL.

  // Synchronization state
  SemaphoreHandle_t mux;      // The handle to the driver mutex which allows
                              // multi-threaded driver function calls.
  TaskHandle_t task_waiting;  // The handle to a task that is waiting for data
                              // to be sent or received.

#ifdef DMX_USE_SPINLOCK
  dmx_spinlock_t spinlock;  // The spinlock used for critical sections.
#endif

  // Data buffer
  int16_t head;     // The index of the slot being transmitted or received.
  uint8_t *data;    // The buffer that stores the DMX packet.
  int16_t tx_size;  // The size of the outgoing packet.
  int16_t rx_size;  // The expected size of the incoming packet.
  int64_t last_slot_ts;  // The timestamp (in microseconds since boot) of the last slot of the previous data packet.

  // Driver state
  uint8_t flags;  // Flags which indicate the current state of the driver.
  uint8_t rdm_type;  // Flags which indicate the RDM type of the most recent packet.
  uint8_t tn;  // The current RDM transaction number. Is incremented with every RDM packet sent.

  // DMX configuration
  struct dmx_personality_t {
    uint16_t footprint;       // The DMX footprint of the personality.
    const char *description;  // A description of the personality.
  } personalities[DMX_PERSONALITY_COUNT_MAX];
  uint32_t break_len;  // Length in microseconds of the transmitted break.
  uint32_t mab_len;  // Length in microseconds of the transmitted mark-after-break.


  struct rdm_driver_t {
    void *pd;  // Allocated memory for DMX/RDM parameter data.
    size_t pd_available;

    // RDM responder configuration
    // uint32_t num_parameters;  // The number of RDM parameters registered.
    // struct rdm_parameter_table_t {
    //   rdm_pid_t pid;               // The PID of this parameter.
    //   void *data;                  // A pointer to the parameter data.
    //   uint8_t flags;
    //   rdm_pd_definition_t definition;  // The definition of the parameter.
    //   rdm_callback_t callback;     // The parameter callback function.
    //   void *context;               // Context for the callback function.
    // } params[RDM_RESPONDER_NUM_PIDS_MAX];  // A table containing RDM parameter information.

    uint32_t param_count;
    struct rdm_pd_vector_s {
      const rdm_pd_definition_t *definition;
      enum rdm_pd_storage_type_e storage_type;
      union rdm_pd_u {
        void *value;
        rdm_pd_getter_t getter;
      } data;
      size_t size;
      uint8_t flags;
      rdm_callback_t callback;
      void *context;
    } params[RDM_RESPONDER_NUM_PIDS_MAX];

  } rdm;

  // Parameter data


  /*



  typedef struct rdm_pd_definition_s {
    rdm_pid_t pid;
    rdm_pd_storage_type_t storage_type;
    rdm_ds_t ds;
    rdm_pid_cc_t pid_cc;
    struct rdm_pd_format_s {
      struct rdm_pd_format_cc_s {
        const char *request;
        const char *response;
      } get, set;
    } format;
    rdm_response_handler_t response_handler;
    rdm_units_t units;
    rdm_prefix_t prefix;
    uint32_t default_value;
    const char *description;
  } rdm_pd_definition_t;





  typedef struct rdm_pd_format_t {
    struct get {
      const char *request;
      const char *response;
    };
    struct set {
      const char *request;
      const char *response;
    };
  } rdm_pd_format_t;

  typedef struct rdm_pd_limits_t {
    uint32_t min;
    uint32_t max;
  } rdm_pd_limits_t;

  static int rdm_rhd(dmx_port_t dmx_num, const rdm_limits_t *limits, 
                     rdm_header_t *header, void *pd, uint8_t *pdl_out);

  typedef struct rdm_pd_definition_t {
    uint32_t default_value;
    rdm_units_t units;
    rdm_prefix_t prefix
    const char *description;
  } rdm_pd_definition_t

  uint32_t parameter_count;
  struct rdm_param_vector_t {
    rdm_pid_t pid;
    uint8_t flags;
    uint8_t pid_cc;
    union {
      struct {
        void *data;
        size_t alloc_size;
      };
      rdm_pd_getter_t getter;
    };
    rdm_pd_format_t format;
    rdm_pd_limits_t limits;
    rdm_response_handler_t response_handler;
    const rdm_pd_definition_t *definition;
  } paramvs[RDM_RESPONDER_NUM_PIDS_MAX];

  const void *rdm_pd_add_variable(dmx_port_t dmx_num, 
                                  rdm_sub_device_t sub_device, rdm_pid_t pid,
                                  const rdm_param_vector_t *v, 
                                  const void *init_value);

  done:
    rdm_ds_t data_type;  // rdm_pd_get_ds(*format);
    rdm_pid_cc_t cc;
    size_t pdl_size;  // rdm_pd_get_size(*format);
    uint32_t min_value;
    uint32_t max_value;
    size_t alloc_size;
    const char *format;

    rdm_response_handler_t response_handler;


  required:


  manufacturer specific only:
    uint32_t default_value;
    rdm_units_t units;
    rdm_prefix_t prefix;
    const char *description;

  */

  uint16_t rdm_queue_last_sent;  // The PID of the last sent queued message.
  uint16_t rdm_queue_size;       // The index of the RDM message queue list.
  uint16_t rdm_queue[RDM_RESPONDER_QUEUE_SIZE_MAX];  // The RDM queued message list.

  // TODO: implement status using space in pd
  // uint8_t rdm_status_threshold;
  // struct rdm_status_queue_t {
  //   uint16_t head;  // The next element to pop
  //   uint16_t tail;  // The next open space for an element
  //   rdm_status_message_t queue[RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX];
  // } rdm_status[3];

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
