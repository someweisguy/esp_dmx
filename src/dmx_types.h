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
  DMX_MODE_READ,   // DMX receive mode.
  DMX_MODE_WRITE,  // DMX transmit mode.
  DMX_MODE_MAX     // Maximum DMX mode value. Used for error checking.
} dmx_mode_t;

/**
 * @brief DMX reset sequence hardware choice. Determines which hardware is used
 * to send the DMX reset sequence.
 */
typedef enum {
  DMX_USE_BUSY_WAIT = -1,                 // Use busy-waits to send the DMX reset sequence. Can be significantly less precise than using a hardware timer if there are multiple tasks to execute.
  DMX_USE_TIMER_GROUP_0 = TIMER_GROUP_0,  // Use hardware timer group 0 to send the DMX reset sequence.
#if SOC_TIMER_GROUPS > 1
  DMX_USE_TIMER_GROUP_1 = TIMER_GROUP_1,  // Use hardware timer group 1 to send the DMX reset sequence.
#endif
  DMX_RESET_SEQUENCE_MAX                  // Maximum DMX reset sequence hardware value. Used for error checking.
} rst_seq_hw_t;

/**
 * @brief Struct that contains DMX driver constants that cannot be changed
 * without first deleting the driver.
 */
typedef struct {
  uint16_t buffer_size;  // The data buffer size of the DMX driver.
  int8_t rst_seq_hw;     // The hardware to use to generate the DMX reset sequence. Can be set to -1 to use busy-wait mode.
  uint8_t timer_idx;     // The timer index to use to generate the DMX reset sequence.
  int intr_alloc_flags;  // Interrupt allocation flags as specified in esp_intr_alloc.h
} dmx_config_t;

/*
TODO
new dmx_event_status_t types
ok == ESP_OK
packet_size/buffer_size == ESP_ERR_INVALID_SIZE
  but the user should know the size of the buffer, so they can figure it out 
  based on the event.size that is returned

timeout == ESP_ERR_TIMEOUT
overflow/improper_slot == ESP_FAIL
  on overflow, set the event.size to -1 so user can figure out which happened
*/
/**
 * @brief DMX packet status types reported to the event queue when a packet is
 * received.
 */
typedef enum {
  DMX_OK = 0,               // The DMX packet is valid.
  DMX_ERR_BUFFER_SIZE,      // The user defined buffer is too small for the received packet.
  DMX_ERR_IMPROPER_SLOT,    // A slot in the packet was improperly framed (missing stop bits).
  DMX_ERR_PACKET_SIZE,      // The packet size is 0 or longer than the DMX standard allows.
  DMX_ERR_DATA_OVERFLOW,    // The UART overflowed causing loss of data.
  DMX_ERR_TIMEOUT,          // Timed out waiting for a DMX or RDM packet.
} dmx_event_status_t;


typedef enum {
  DMX_DIMMER_PACKET = 0x00,
  RDM_DISCOVERY_COMMAND = 0x10,
  RDM_DISCOVERY_COMMAND_RESPONSE = 0x11,
  RDM_GET_COMMAND = 0x20,
  RDM_GET_COMMAND_RESPONSE = 0x21,
  RDM_SET_COMMAND = 0x30,
  RDM_SET_COMMAND_RESPONSE = 0x31,
} rdm_command_class_t;

typedef enum {
  RESPONSE_TYPE_ACK = 0x00,
  RESPONSE_TYPE_ACK_TIMER = 0x01,
  RESPONSE_TYPE_NACK_REASON = 0x02,
  RESPONSE_TYPE_ACK_OVERFLOW = 0x03
} rdm_response_type_t;

enum {
  DMX_DATA_CLASS = DMX_SC,  // DMX class data. A packet is a DMX packet if it begins with a null start code.
  RDM_DATA_CLASS = RDM_SC   // RDM class data. A packet is an RDM packet if it begins with an RDM start code and an RDM sub-start code.
};

typedef struct {
    uint8_t sc;
    uint8_t sub_sc;
    uint8_t size;
    uint8_t destination_uid[6];
    uint8_t source_uid[6];
    uint8_t transaction_num;
    union {
      uint8_t port_id;
      uint8_t response_type;
    };
    uint8_t message_count;
    uint16_t sub_device;
    uint8_t command_class;
    uint16_t parameter_id; 
    uint8_t parameter_data_len;
    void *parameter_data;
} rdm_packet_t;

/**
 * @brief DMX data events reported to the event queue when a packet is received.
 */
typedef struct {
  dmx_event_status_t status;   // The status of the received DMX packet.
  int16_t size;                // The size of the received DMX packet in bytes.
  uint8_t data_class;          // The type of packet received. 
  struct {
    int32_t break_len;         // Duration of the DMX break in microseconds.
    int32_t mab_len;           // Duration of the DMX mark-after-break in microseconds.
  } timing;                    // Timing values received from the DMX sniffer.
  struct {
    uint64_t destination_uid;  // TODO
    uint64_t source_uid;       // TODO
    uint8_t transaction_num;
    union {
      uint8_t port_id;
      uint8_t response_type;
    };
    uint8_t message_count;
    uint16_t sub_device;
    uint8_t command_class;
    uint16_t parameter_id;  // TODO: replace with enum?
    uint8_t parameter_data_len;
    void *parameter_data;
    bool checksum_is_valid;
  } rdm;
} dmx_event_t;

/**
 * @brief Interrupt configuration used to configure the DMX hardware ISR.
 */
typedef struct {
  uint8_t rx_timeout_threshold;    // DMX timeout interrupt threshold. This sets the amount of time after receiving data that it takes for the "RX FIFO timeout" interrupt to fire. Unit: time of sending one byte.
  uint8_t txfifo_empty_threshold;  // DMX TX empty interrupt threshold. This the maximum number of bytes that are needed in the UART TX FIFO for the "FIFO empty" interrupt to fire.
  uint8_t rxfifo_full_threshold;   // DMX RX full interrupt threshold. This is the minimum number of bytes that are needed in the UART RX FIFO for the "FIFO full" interrupt to fire.
} dmx_intr_config_t;

#ifdef __cplusplus
}
#endif
