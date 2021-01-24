#pragma once

#include "esp_system.h"
#include "hal/uart_types.h"

#define DMX_DEFAULT_CONFIG \
  {                        \
      .baudrate = 250000,  \
      .break_num = 44,     \
      .idle_num = 3,       \
  }

typedef int dmx_port_t;             // DMX port type.

/**
 * @brief DMX configuration parameters for dmx_param_config() function
 */
typedef struct {
  int baudrate;           // DMX baud rate.
  uint8_t break_num;      // DMX break length (unit: number of bits).
  uint16_t idle_num;      // DMX mark after break length (unit: number of bits).
  uart_sclk_t source_clk; // DMX source clock selection.
} dmx_config_t;

/**
 * @brief DMX modes of operation.
 */
typedef enum {
  DMX_MODE_RX,            // DMX receive mode.
  DMX_MODE_TX,            // DMX transmit mode.
  DMX_MODE_MAX            // Maximum DMX mode value - used for error checking.
} dmx_mode_t;

/**
 * @brief DMX packet types reported to the event queue when a packet is received.
 */
typedef enum {
  DMX_OK = 0,             // The DMX packet is valid.
  DMX_ERR_IMPROPER_SLOT,  // A slot is improperly framed (missing stop bits).
  DMX_ERR_PACKET_SIZE,    // The packet size is 0 or longer than the DMX standard allows.
  DMX_ERR_BUFFER_SIZE,    // The user defined buffer is too small for the received packet.
  DMX_ERR_DATA_OVERFLOW,  // The hardware FIFO overflowed, causing loss of data.
  DMX_ERR_MAX
} dmx_event_type_t;

/**
 * @brief DMX data events reported to the event queue when a packet is received.
 */
typedef struct {
  dmx_event_type_t type;            // The type of DMX packet received.
  int16_t start_code;               // The start code (slot 0) of the DMX packet, or -1 on error (except for DMX_ERR_BUFFER_SIZE).
  size_t size;                      // The length of the received DMX packet.
  uint32_t packet_len;
  uint32_t brk_len;
  uint32_t mab_len;
} dmx_event_t;

/**
 * @brief Interrupt configuration used to configure the DMX hardware ISR.
 */
typedef struct {
  uint8_t rx_timeout_thresh;        // DMX timeout interrupt threshold (unit: time of sending one byte).
  uint8_t txfifo_empty_intr_thresh; // DMX tx empty interrupt threshold.
  uint8_t rxfifo_full_thresh;       // DMX rx full interrupt threshold.
} dmx_intr_config_t;