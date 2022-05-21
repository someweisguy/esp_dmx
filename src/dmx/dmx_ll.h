#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "hal/uart_types.h"

FORCE_INLINE_ATTR uint16_t dmx_ll_get_idle_num(uart_dev_t *hw) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  return hw->idle_conf.tx_idle_num;
// #elif defined(CONFIG_IDF_TARGET_ESP32C2)
  // FIXME
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hw->idle_conf.tx_idle_num;
// #elif defined(CONFIG_IDF_TARGET_ESP32H2)
  // FIXME
// #elif defined(CONFIG_IDF_TARGET_ESP32S2)
  // FIXME
// #elif defined(CONFIG_IDF_TARGET_ESP32S3)
  // FIXME
#else
#define DMX_GET_IDLE_NUM_NOT_SUPPORTED
#endif
  return 5; // default 20 microseconds (assuming 250k baud)
}

FORCE_INLINE_ATTR uint8_t dmx_ll_get_break_num(uart_dev_t *hw) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  return hw->idle_conf.tx_brk_num;
// #elif defined(CONFIG_IDF_TARGET_ESP32C2)
  // FIXME
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hw->txbrk_conf.tx_brk_num;
#else
// #elif defined(CONFIG_IDF_TARGET_ESP32H2)
  // FIXME
// #elif defined(CONFIG_IDF_TARGET_ESP32S2)
  // FIXME
// #elif defined(CONFIG_IDF_TARGET_ESP32S3)
  // FIXME
#define DMX_GET_BREAK_NUM_NOT_SUPPORTED
#endif
  return 45; // default 180 microseconds (assuming 250k baud)
}

FORCE_INLINE_ATTR uint32_t dmx_ll_get_rx_level(uart_dev_t *hw) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  return hw->status.rxd;
// #elif defined(CONFIG_IDF_TARGET_ESP32C2)
  // FIXME
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hw->status.rxd;
// #elif defined(CONFIG_IDF_TARGET_ESP32H2)
  // FIXME
// #elif defined(CONFIG_IDF_TARGET_ESP32S2)
  // FIXME
// #elif defined(CONFIG_IDF_TARGET_ESP32S3)
  // FIXME
#else
#define DMX_GET_RX_LEVEL_NOT_SUPPORTED
#endif
  return 0; // default rx low
}

#ifdef __cplusplus
}
#endif