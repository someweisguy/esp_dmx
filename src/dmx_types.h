#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_system.h"
#include "driver/uart.h"

typedef int dmx_port_t;             // DMX port type.

/**
 * @brief DMX configuration parameters for dmx_param_config() function
 */
typedef struct {
  int baud_rate;                    // DMX baud rate.
  uint8_t break_num;                // DMX break length (unit: number of bits).
  uint16_t idle_num;                // DMX mark after break length (unit: number of bits).
  uart_sclk_t source_clk;           // DMX source clock selection.
} dmx_config_t;

/**
 * @brief DMX modes of operation.
 */
typedef enum {
  DMX_MODE_READ,                    // DMX receive mode.
  DMX_MODE_WRITE,                   // DMX transmit mode.
  DMX_MODE_MAX                      // Maximum DMX mode value - used for error checking.
} dmx_mode_t;

/**
 * @brief DMX packet status types reported to the event queue when a packet is received.
 */
typedef enum {
  DMX_OK = 0,                       // The DMX packet is valid.
  DMX_ERR_BUFFER_SIZE,              // The user defined buffer is too small for the received packet.
  DMX_ERR_IMPROPER_SLOT,            // A slot is improperly framed (missing stop bits).
  DMX_ERR_PACKET_SIZE,              // The packet size is 0 or longer than the DMX standard allows.
  DMX_ERR_DATA_OVERFLOW             // The hardware FIFO overflowed, causing loss of data.
} dmx_event_status_t;

/**
 * @brief DMX data events reported to the event queue when a packet is received.
 */
typedef struct {
  dmx_event_status_t status;        // The status of DMX packet.
  int start_code;                   // The start code (slot 0) of the DMX packet, or -1 on error (except for DMX_ERR_BUFFER_SIZE).
  size_t size;                      // The size of the received DMX packet in bytes.
  int32_t duration;                 // The duration of the received DMX packet in microseconds.
  struct {
    int32_t brk;                    // Duration of the break in microseconds.
    int32_t mab;                    // Duration of the mark-after-break in microseconds.
  } timing;                         // Timing values received from the DMX sniffer.
} dmx_event_t;

/**
 * @brief Interrupt configuration used to configure the DMX hardware ISR.
 */
typedef struct {
  uint8_t rx_timeout_thresh;        // DMX timeout interrupt threshold (unit: time of sending one byte).
  uint8_t txfifo_empty_intr_thresh; // DMX tx empty interrupt threshold.
  uint8_t rxfifo_full_thresh;       // DMX rx full interrupt threshold.
} dmx_intr_config_t;

#ifdef __cplusplus
}
#endif
