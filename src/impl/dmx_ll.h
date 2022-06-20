#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "hal/uart_types.h"
#include "soc/uart_struct.h"

/**
 * The primary purpose of the LL Layer is to abstract away register field access
 * into more easily understandable functions. LL functions essentially translate
 * various in/out arguments into the register fields of a peripheral in the form
 * of get/set functions. All the necessary bit-shifting, masking, ofsetting, and
 * endianness of the register fields should be handled by the LL functions.
 *
 * All LL Layer functions should be tagged with FORCE_INLINE_ATTR to ensure
 * minimal overhead. The functions defined here do not have an equivalent LL
 * Layer function in the ESP-IDF LL Layer.
 * */

FORCE_INLINE_ATTR uint32_t dmx_ll_get_rx_level(uart_dev_t *hw) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return hw->status.rxd;
// #elif defined(CONFIG_IDF_TARGET_ESP32C2)
// TODO: Not yet supported by ESP-IDF.
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hw->status.rxd;
// #elif defined(CONFIG_IDF_TARGET_ESP32H2)
// TODO: Not yet supported by ESP-IDF.
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  return hw->status.rxd;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  return hw->uart_status_reg_t.rxd;
#else
#define DMX_GET_RX_LEVEL_NOT_IMPLEMENTED
#endif
  return 0;  // default rx low
}

#ifdef __cplusplus
}
#endif