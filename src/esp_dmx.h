/**
 * @file esp_dmx.h
 * @author Mitch Weisbrod
 * @brief This is the main header file for esp_dmx. It contains required API
 * functions for the user. Notably, the functions in this header are not
 * hardware-dependent. The functions in this file do not interface with any of
 * the functions described in uart.h, timer.h, or nvs.h. This header includes
 * dmx/hal.h which contains functions that interface with hardware.
 */
#pragma once

#include "dmx/device.h"
#include "dmx/driver.h"
#include "dmx/io.h"
#include "dmx/rw.h"
#include "dmx_types.h"
#include "rdm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief The major version number of this library. (X.x.x)*/
#define ESP_DMX_VERSION_MAJOR 3

/** @brief The minor version number of this library. (x.X.x)*/
#define ESP_DMX_VERSION_MINOR 1

/** @brief The patch version number of this library. (x.x.X)*/
#define ESP_DMX_VERSION_PATCH 0

/** @brief The version of this library expressed as a 32-bit integer value.*/
#define ESP_DMX_VERSION_ID                                        \
  ((ESP_DMX_VERSION_MAJOR << 16) | (ESP_DMX_VERSION_MINOR << 8) | \
   ESP_DMX_VERSION_PATCH)

/** @brief The version of this library expressed as a string value.*/
#define ESP_DMX_VERSION_LABEL                                 \
  "esp_dmx v" __XSTRING(ESP_DMX_VERSION_MAJOR) "." __XSTRING( \
      ESP_DMX_VERSION_MINOR) "." __XSTRING(ESP_DMX_VERSION_PATCH)

/** @brief The default configuration for the DMX driver. Passing this
 * configuration to dmx_driver_install() installs the driver with one DMX
 * personality which has a footprint of one DMX address. The DMX address will
 * automatically be searched for in NVS and set to 1 if not found or if NVS is
 * disabled. */
#define DMX_CONFIG_DEFAULT                                     \
  ((dmx_config_t){                                             \
      255,                          /*pd_size*/                \
      0,                            /*model_id*/               \
      RDM_PRODUCT_CATEGORY_FIXTURE, /*product_category*/       \
      ESP_DMX_VERSION_ID,           /*software_version_id*/    \
      ESP_DMX_VERSION_LABEL,        /*software_version_label*/ \
      "Default Device",             /*device_label*/           \
      1,                            /*current_personality*/    \
      {{1, "Default Personality"}}, /*personalities*/          \
      1,                            /*personality_count*/      \
      0,                            /*dmx_start_address*/      \
  })

#ifdef DMX_ISR_IN_IRAM
/** @brief The default interrupt flags for the DMX sniffer. Places the
 * interrupts in IRAM.*/
#define DMX_SNIFFER_INTR_FLAGS_DEFAULT (ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM)
/** @brief The default interrupt flags for the DMX driver. Places the
 * interrupts in IRAM.*/
#define DMX_INTR_FLAGS_DEFAULT (ESP_INTR_FLAG_IRAM)
#else
/** @brief The default interrupt flags for the DMX sniffer.*/
#define DMX_SNIFFER_INTR_FLAGS_DEFAULT (ESP_INTR_FLAG_EDGE)
/** @brief The default interrupt flags for the DMX driver.*/
#define DMX_INTR_FLAGS_DEFAULT (0)
#endif

#ifdef __cplusplus
}
#endif
