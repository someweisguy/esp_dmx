#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/timer.h"
#include "driver/uart.h"
#include "esp_system.h"

/**
 * @brief DMX port type.
 */
typedef int dmx_port_t;

/**
 * @brief DMX modes of operation.
 */
typedef enum {
  /**
   * @brief DMX receive mode.
   */
  DMX_MODE_READ,
  /**
   * @brief DMX transmit mode.
   */
  DMX_MODE_WRITE,
  /**
   * @brief Maximum DMX mode value. Used for error checking.
   */
  DMX_MODE_MAX
} dmx_mode_t;

/**
 * @brief DMX reset sequence hardware choice. Determines which hardware is used
 * to send the DMX reset sequence.
 */
typedef enum {
  /**
   * @brief Use busy-waits to send the DMX reset sequence. Can be significantly
   * less precise than using a hardware timer if there are multiple tasks to
   * execute.
   */
  DMX_USE_BUSY_WAIT = -1,
  /**
   * @brief Use hardware timer group 0 to send the DMX reset sequence.
   */
  DMX_USE_TIMER_GROUP_0 = TIMER_GROUP_0,
#if SOC_TIMER_GROUPS > 1
  /**
   * @brief Use hardware timer group 1 to send the DMX reset sequence.
   */
  DMX_USE_TIMER_GROUP_1 = TIMER_GROUP_1,
#endif
  /**
   * @brief Maximum DMX reset sequence hardware value. Used for error checking.
   */
  DMX_RESET_SEQUENCE_MAX
} rst_seq_hw_t;

/**
 * @brief Struct that contains DMX driver constants that cannot be changed
 * without first deleting the driver.
 */
typedef struct {
  /**
   * @brief The data buffer size of the DMX driver.
   */
  uint16_t buffer_size;
  /**
   * @brief The hardware to use to generate the DMX reset sequence. Can be set
   * to -1 to use busy-wait mode.
   */
  int8_t rst_seq_hw;
  /**
   * @brief The timer index to use to generate the DMX reset sequence.
   */
  uint8_t timer_idx;
  /**
   * @brief Interrupt allocation flags as specified in esp_intr_alloc.h
   */
  int intr_alloc_flags;
} dmx_config_t;

/**
 * @brief DMX packet status types reported to the event queue when a packet is
 * received.
 */
typedef enum {
  /**
   * @brief The DMX packet is valid.
   */
  DMX_OK = 0,
  /**
   * @brief The user defined buffer is too small for the received packet.
   */
  DMX_ERR_BUFFER_SIZE,
  /**
   * @brief A slot in the packet was improperly framed (missing stop bits).
   */
  DMX_ERR_IMPROPER_SLOT,
  /**
   * @brief The packet size is 0 or longer than the DMX standard allows.
   */
  DMX_ERR_PACKET_SIZE,
  /**
   * @brief The UART overflowed causing loss of data.
   */
  DMX_ERR_DATA_OVERFLOW,
  /**
   * @brief Timed out waiting for a DMX or RDM packet.
   */
  DMX_ERR_TIMEOUT
} dmx_event_status_t;

/**
 * @brief DMX data events reported to the event queue when a packet is received.
 */
typedef struct {
  /**
   * @brief The status of the received DMX packet.
   */
  dmx_event_status_t status;
  /**
   * @brief Is true if the packet is an RDM packet.
   */
  bool is_rdm;
  /**
   * @brief The size of the received DMX packet in bytes.
   */
  size_t size;

  /**
   * @brief Timing values received from the DMX sniffer.
   */
  struct {
    /**
     * @brief Duration of the DMX break in microseconds.
     */
    int32_t brk;
    /**
     * @brief Duration of the DMX mark-after-break in microseconds.
     *
     */
    int32_t mab;
  } timing;
  struct {
    uint64_t source_uid;
    uint64_t destination_uid;
  } rdm;
  /**
   * @brief True if the event was sent to the event queue during the next DMX
   * packet's reset sequence.
   */
  bool is_late;
} dmx_event_t;

/**
 * @brief Interrupt configuration used to configure the DMX hardware ISR.
 */
typedef struct {
  /**
   * @brief DMX timeout interrupt threshold. This sets the amount of time after
   * receiving data that it takes for the "RX FIFO timeout" interrupt to fire.
   * Unit: time of sending one byte.
   */
  uint8_t rx_timeout_threshold;
  /**
   * @brief DMX TX empty interrupt threshold. This the maximum number of bytes
   * that are needed in the UART TX FIFO for the "FIFO empty" interrupt to fire.
   */
  uint8_t txfifo_empty_threshold;
  /**
   * @brief DMX RX full interrupt threshold. This is the minimum number of bytes
   * that are needed in the UART RX FIFO for the "FIFO full" interrupt to fire.
   */
  uint8_t rxfifo_full_threshold;
} dmx_intr_config_t;

#ifdef __cplusplus
}
#endif
