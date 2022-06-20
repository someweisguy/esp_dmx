#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_system.h"
#include "driver/uart.h"
#include "driver/timer.h"

typedef int dmx_port_t;             // DMX port type.



/**
 * @brief DMX modes of operation.
 */
typedef enum {
  DMX_MODE_READ,                    // DMX receive mode.
  DMX_MODE_WRITE,                   // DMX transmit mode.
  DMX_MODE_MAX                      // Maximum DMX mode value - used for error checking.
} dmx_mode_t;

// TODO: Documentation
typedef enum {
  DMX_USE_UART = -1,
  DMX_USE_TIMER_GROUP_0 = TIMER_GROUP_0,
#if SOC_TIMER_GROUPS > 1
  DMX_USE_TIMER_GROUP_1 = TIMER_GROUP_1,
#endif
  DMX_RESET_SEQUENCE_MAX
} rst_seq_hw_t;

// TODO: documentation! This struct contains driver config information that
//  cannot be changed without first deleting the driver
typedef struct {
  uint16_t buffer_size;             // The buffer size of the DMX driver.
  int8_t rst_seq_hw;          // The timer group to use to generate the reset sequence. Can be set to -1 to use reset-sequence-last mode.
  uint8_t timer_idx;                // The timer index to use to generate the reset sequence. Can be set to -1 to use reset-sequence-last mode.
  int intr_alloc_flags;             // Interrupt allocation flags as specified in esp_intr_alloc.h.
} dmx_config_t;

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
  size_t size;                      // The size of the received DMX packet in bytes.
  struct {
    int32_t brk;                    // Duration of the break in microseconds.
    int32_t mab;                    // Duration of the mark-after-break in microseconds.
  } timing;                         // Timing values received from the DMX sniffer.
  bool is_late;                     // True if the event was sent to the event queue during the next DMX packet's reset sequence.
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
