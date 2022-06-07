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
 * 
 * */

FORCE_INLINE_ATTR uint16_t dmx_ll_get_idle_num(uart_dev_t *hw) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return hw->idle_conf.tx_idle_num;
// #elif defined(CONFIG_IDF_TARGET_ESP32C2)
  // TODO: Not supported yet
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hw->idle_conf.tx_idle_num;
// #elif defined(CONFIG_IDF_TARGET_ESP32H2)
  // TODO: Not supported yet
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  return hw->idle_conf.tx_idle_num;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  return hw->uart_idle_conf_reg_t.tx_idle_num;
#else
#define DMX_GET_IDLE_NUM_NOT_SUPPORTED
#endif
  return 5; // default 20 microseconds (assuming 250k baud)
}

FORCE_INLINE_ATTR uint8_t dmx_ll_get_break_num(uart_dev_t *hw) {
#if defined(CONFIG_IDF_TARGET_ESP32)
  return hw->idle_conf.tx_brk_num;
// #elif defined(CONFIG_IDF_TARGET_ESP32C2)
  // TODO: Not supported yet
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hw->txbrk_conf.tx_brk_num;
// #elif defined(CONFIG_IDF_TARGET_ESP32H2)
  // TODO: Not supported yet
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  return hw->idle_conf.tx_brk_num;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  return hw->uart_txbrk_conf_reg_t.tx_brk_num;
#else
#define DMX_GET_BREAK_NUM_NOT_SUPPORTED
#endif
  return 45; // default 180 microseconds (assuming 250k baud)
}

FORCE_INLINE_ATTR uint32_t dmx_ll_get_rx_level(uart_dev_t *hw) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  return hw->status.rxd;
// #elif defined(CONFIG_IDF_TARGET_ESP32C2)
  // TODO: Not supported yet
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  return hw->status.rxd;
// #elif defined(CONFIG_IDF_TARGET_ESP32H2)
  // TODO: Not supported yet
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  return hw->status.rxd;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  return hw->uart_status_reg_t.rxd;
#else
#define DMX_GET_RX_LEVEL_NOT_SUPPORTED
#endif
  return 0; // default rx low
}

#ifdef __cplusplus
}
#endif