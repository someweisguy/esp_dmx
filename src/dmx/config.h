/**
 * @file config.h
 * @author Mitch Weisbrod
 * @brief This file contains configuration information for the DMX library. It
 * is not considered part of the API and should not be included by the user.
 */
#pragma once

#include "esp_check.h"

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
#define RDM_RESPONDER_NUM_PIDS_OPTIONAL (CONFIG_RDM_RESPONDER_MAX_OPTIONAL_PARAMETERS)
#else
#define RDM_RESPONDER_NUM_PIDS_OPTIONAL 25
#endif

/** @brief The maximum number of parameters that the RDM responder can
 * support.*/
#define RDM_RESPONDER_PIDS_MAX (RDM_RESPONDER_NUM_PIDS_REQUIRED + RDM_RESPONDER_NUM_PIDS_OPTIONAL)

#ifdef CONFIG_RDM_RESPONDER_MAX_QUEUE_SIZE
/** @brief The maximum number of queued messages that ther RDM responder can 
 * support. It may be set using the Kconfig file.
 */
#define RDM_RESPONDER_QUEUE_SIZE_MAX CONFIG_RDM_RESPONDER_MAX_QUEUE_SIZE
#else
/** @brief The maximum number of queued messages that ther RDM responder can 
 * support.
 */
#define RDM_RESPONDER_MAX_QUEUE_SIZE 64
#endif

/** @brief Directs the DMX driver to use spinlocks in critical sections. This is
 * needed for devices which have multiple cores.*/
#define DMX_USE_SPINLOCK
typedef spinlock_t dmx_spinlock_t;
#define DMX_SPINLOCK_INIT portMUX_INITIALIZER_UNLOCKED

extern const char *TAG;  // The log tagline for the library.

#ifdef __cplusplus
}
#endif
