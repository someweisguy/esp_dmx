#pragma once

#include "esp_system.h"
#include "hal/uart_types.h"

#define DMX_DEFAULT_CONFIG \
  {                        \
      .baudrate = 250000,  \
      .break_num = 44,     \
      .idle_num = 3,       \
  }

/**
 * @brief DMX configuration parameters for dmx_param_config function
 */
typedef struct {
  int baudrate;           // DMX baud rate.
  uint8_t break_num;      // DMX break length (unit: number of bits).
  uint16_t idle_num;      // DMX mark after break length (unit: number of bits).
  uart_sclk_t source_clk; // DMX source clock selection.
} dmx_config_t;