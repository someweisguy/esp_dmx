/**
 * @file esp_dmx.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This is the main header file for esp_dmx. It includes the required
 * header files for the basic operation of this library and defines macros to
 * allow for users to quickly create functional DMX devices.
 */
#pragma once

#include "dmx/include/device.h"
#include "dmx/include/driver.h"
#include "dmx/include/types.h"
#include "rdm/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief The major version number of this library. (X.x.x)*/
#define ESP_DMX_VERSION_MAJOR 4

/** @brief The minor version number of this library. (x.X.x)*/
#define ESP_DMX_VERSION_MINOR 1

/** @brief The patch version number of this library. (x.x.X)*/
#define ESP_DMX_VERSION_PATCH 0

/** @brief The version of this library expressed as a 32-bit integer value.*/
#define ESP_DMX_VERSION_ID                                        \
  ((ESP_DMX_VERSION_MAJOR << 16) | (ESP_DMX_VERSION_MINOR << 8) | \
   ESP_DMX_VERSION_PATCH)

/** @brief The version of this library expressed as a string value.*/
#define ESP_DMX_VERSION_LABEL                         \
  __XSTRING(ESP_DMX_VERSION_MAJOR)                    \
  "." __XSTRING(ESP_DMX_VERSION_MINOR) "." __XSTRING( \
      ESP_DMX_VERSION_PATCH) " " __DATE__

#if defined(CONFIG_DMX_ISR_IN_IRAM) || ESP_IDF_VERSION_MAJOR < 5
/** @brief The default interrupt flags for the DMX driver. Places the
 * interrupts in IRAM.*/
#define DMX_INTR_FLAGS_DEFAULT (ESP_INTR_FLAG_IRAM)
#else
/** @brief The default interrupt flags for the DMX driver.*/
#define DMX_INTR_FLAGS_DEFAULT (0)
#endif

/** @brief The default configuration for the DMX driver.*/
#define DMX_CONFIG_DEFAULT                                            \
  (dmx_config_t) {                                                    \
    DMX_INTR_FLAGS_DEFAULT,           /*interrupt_flags*/             \
        32,                           /*root_device_parameter_count*/ \
        0,                            /*sub_device_parameter_count*/  \
        0,                            /*model_id*/                    \
        RDM_PRODUCT_CATEGORY_FIXTURE, /*product_category*/            \
        ESP_DMX_VERSION_ID,           /*software_version_id*/         \
        ESP_DMX_VERSION_LABEL,        /*software_version_label*/      \
        32,                           /*queue_size_max*/              \
  }

#ifdef __cplusplus
}
#endif
