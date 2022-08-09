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
  int timer_group;       // The hardware to use to generate the DMX reset sequence. Can be set to -1 to use busy-wait mode.
  int timer_num;         // The timer index to use to generate the DMX reset sequence.
  int intr_alloc_flags;  // Interrupt allocation flags as specified in esp_intr_alloc.h
} dmx_config_t;

/**
 * @brief UID which indicates an RDM packet is being broadcast. Responders shall
 * not respond to RDM broadcast messages.
 */
static const uint64_t RDM_BROADCAST_UID = 0xffffffffffff;

/**
 * @brief DMX error codes. These values indicate problem in receiving DMX data 
 * or processing RDM packets.
 */
typedef enum {
  DMX_OK = 0,  // Indicates data was read successfully.
  DMX_IMPROPERLY_FRAMED_SLOT,  // The receiver detected missing stop bits. If a missing stop bit is detected, the receiver shall discard the improperly framed slot data and all following slots in the packet.
  DMX_DATA_COLLISION,  // A data collision was detected. This typically occurs during RDM discovery.
  DMX_HARDWARE_OVERFLOW,  // The ESP32 hardware overflowed, resulting in loss of data.
} dmx_err_t;

/**
 * @brief The RDM command class (CC) type. The command class specifies the 
 * action of the message. Responders shall always generate a response to 
 * GET_COMMAND and SET_COMMAND messages except when the destination UID of the
 * message is a broadcast address. Responders shall not respond to commands sent 
 * using broadcast addressing, in order to prevent collisions.
 */
typedef enum {
  DMX_NON_RDM_PACKET = 0x00,  // The packet is a non-RDM packet.
  RDM_DISCOVERY_COMMAND = 0x10,  // The packet is an RDM discovery command.
  RDM_DISCOVERY_COMMAND_RESPONSE = 0x11,  // The packet is a response to an RDM discovery command.
  RDM_GET_COMMAND = 0x20,  // The packet is an RDM get request.
  RDM_GET_COMMAND_RESPONSE = 0x21,  // The packet is a response to an RDM get request.
  RDM_SET_COMMAND = 0x30,  // The packet is an RDM set request.
  RDM_SET_COMMAND_RESPONSE = 0x31,  // The packet is a response to an RDM set request.
} rdm_cc_t;

typedef enum {
  RESPONSE_TYPE_ACK = 0x00,
  RESPONSE_TYPE_ACK_TIMER = 0x01,
  RESPONSE_TYPE_NACK_REASON = 0x02,
  RESPONSE_TYPE_ACK_OVERFLOW = 0x03
} rdm_response_type_t;

typedef enum {
  DISC_UNIQUE_BRANCH = 0x0001,
  DISC_MUTE = 0x0002,
  DISC_UN_MUTE = 0x0003,
  PROXIED_DEVICES = 0x0010,
  PROXIED_DEVICE_COUNT = 0x0011,
  COMMS_STATUS = 0x0015,
  // TODO: Add the rest of the PIDs
} rdm_pid_t;

/**
 * @brief DMX data events reported to the user when a packet is received.
 */
typedef struct {
  dmx_err_t err;  // Evaluates to true if an error occurred reading DMX data. Refer to dmx_err_t to evaluate the type of error.
  size_t size;  // The size of the received DMX packet in bytes.
  bool is_rdm;  // True if the received packet is RDM.
  struct {
    uint64_t destination_uid;  // The UID of the target device(s).
    uint64_t source_uid;  // The UID of the device originating this packet.
    size_t tn;  // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
    union {
      int port_id;  // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
      rdm_response_type_t response_type;  // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
    };
    size_t message_count;  // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
    int sub_device;  // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
    rdm_cc_t cc;  // The command class (CC) specifies the action of the message. 
    rdm_pid_t pid;  // The parameter ID (PID) identifies a specific type of parameter data.
    size_t pdl;  // The parameter data length (PDL) is the number of slots included in the parameter data area that it precedes.
    
    bool checksum_is_valid;  // Is true if the RDM checksum is valid.
  } rdm;  // A struct containing information about the received RDM packet.
} dmx_event_t;

/**
 * @brief A struct which can be used to help process raw RDM packets instead of 
 * reading slots by index alone. RDM sends data in most-significant byte first,
 * so endianness must be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) {
    uint8_t sc;  // This field shall contain the defined RDM start code. Controllers and responders shall always send RDM_SC in this slot.
    uint8_t sub_sc;  // This field shall contain the sub-start code within RDM that defines this packet structure. Unless specified in future version, the sub-start code shall be equal to RDM_SUB_SC.
    uint8_t message_len;  // The message length value is defined as the number of slots int he RDM packet including the start code and excluding the checksum.
    uint8_t destination_uid[6];  // The UID of the target device(s).
    uint8_t source_uid[6];  // The UID of the device originating this packet.
    uint8_t tn;  // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
    union {
      uint8_t port_id;  // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
      uint8_t response_type;  // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
    };
    uint8_t message_count;  // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
    uint16_t sub_device;  // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
    uint8_t cc;  // The command class (CC) specifies the action of the message. 
    uint16_t pid;  // The parameter ID (PID) identifies a specific type of parameter data.
    uint8_t pdl;  // The parameter data length (PDL) is the number of slots included in the parameter data area that it precedes.
    void *pd;  // The parameter data (PD) is of variable length. The content format is PID dependent.
} rdm_data_t;

#ifdef __cplusplus
}
#endif
