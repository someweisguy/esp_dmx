#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DMX port type.
 */
typedef int dmx_port_t;

/**
 * @brief Struct that contains DMX driver constants that cannot be changed
 * without first deleting the driver.
 */
typedef struct {
  int8_t rst_seq_hw;     // The hardware to use to generate the DMX reset sequence. Can be set to -1 to use busy-wait mode.
  uint8_t timer_idx;     // The timer index to use to generate the DMX reset sequence.
  int intr_alloc_flags;  // Interrupt allocation flags as specified in esp_intr_alloc.h
} dmx_config_t;

typedef enum {
  DMX_NON_RDM_PACKET = 0x00,

  RDM_DISCOVERY_COMMAND = 0x10,
  RDM_DISCOVERY_COMMAND_RESPONSE = 0x11,
  RDM_GET_COMMAND = 0x20,
  RDM_GET_COMMAND_RESPONSE = 0x21,
  RDM_SET_COMMAND = 0x30,
  RDM_SET_COMMAND_RESPONSE = 0x31,
};

typedef enum {
  RESPONSE_TYPE_ACK = 0x00,
  RESPONSE_TYPE_ACK_TIMER = 0x01,
  RESPONSE_TYPE_NACK_REASON = 0x02,
  RESPONSE_TYPE_ACK_OVERFLOW = 0x03
};

static const uint64_t RDM_BROADCAST_UID = 0xffffffffffff;

typedef enum {
  DMX_OK = 0,
  DMX_IMPROPERLY_FRAMED_SLOT,
  DMX_DATA_COLLISION,
  DMX_HARDWARE_OVERFLOW,
} dmx_err_t;

typedef struct __attribute__((__packed__)) {
    uint8_t sc;
    uint8_t sub_sc;
    uint8_t message_len;
    uint8_t destination_uid[6];
    uint8_t source_uid[6];
    uint8_t transaction_num;
    union {
      uint8_t port_id;
      uint8_t response_type;
    };
    uint8_t message_count;
    uint16_t sub_device;
    union {
      uint8_t command_class;
      uint8_t cc;
    };
    uint16_t parameter_id; 
    uint8_t parameter_data_len;
    void *parameter_data;
} rdm_packet_t;

/**
 * @brief DMX data events reported to the event queue when a packet is received.
 */
typedef struct {
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
    int checksum_is_valid;
  } rdm;
} dmx_event_t;

#ifdef __cplusplus
}
#endif
