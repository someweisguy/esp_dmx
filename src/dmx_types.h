/**
 * @file dmx_types.h
 * @author Mitch Weisbrod
 * @brief This file contains all the types used for processing both DMX and RDM 
 * data. Anonymous enums and other constants should not be put in this file but
 * should be placed in dmx_constants.h instead.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DMX port type.
 */
typedef unsigned int dmx_port_t;

typedef struct dmx_sniffer_data {
  uint32_t break_len;  // Length in microseconds of the last received DMX break.
  uint32_t mab_len;    // Length in microseconds of the last received DMX mark-after-break.
} dmx_sniffer_data_t ;

/**
 * @brief DMX error codes. These values indicate problem in receiving DMX data 
 * or processing RDM packets.
 */
typedef enum dmx_err {
  DMX_OK = 0,                      // Indicates data was read successfully.
  DMX_ERR_IMPROPERLY_FRAMED_SLOT,  // The receiver detected missing stop bits. If a missing stop bit is detected, the receiver shall discard the improperly framed slot data and all following slots in the packet.
  DMX_ERR_DATA_COLLISION,          // A data collision was detected. This typically occurs during RDM discovery.
  DMX_ERR_HARDWARE_OVERFLOW,       // The ESP32 hardware overflowed, resulting in loss of data.
} dmx_err_t;

/**
 * @brief The RDM command class (CC) type. The command class specifies the 
 * action of the message. Responders shall always generate a response to 
 * GET_COMMAND and SET_COMMAND messages except when the destination UID of the
 * message is a broadcast address. Responders shall not respond to commands sent 
 * using broadcast addressing, in order to prevent collisions.
 */
typedef enum rdm_command_class {
  RDM_NON_RDM_PACKET = 0x00,              // The packet is a non-RDM packet.
  RDM_DISCOVERY_COMMAND = 0x10,           // The packet is an RDM discovery command.
  RDM_DISCOVERY_COMMAND_RESPONSE = 0x11,  // The packet is a response to an RDM discovery command.
  RDM_GET_COMMAND = 0x20,                 // The packet is an RDM get request.
  RDM_GET_COMMAND_RESPONSE = 0x21,        // The packet is a response to an RDM get request.
  RDM_SET_COMMAND = 0x30,                 // The packet is an RDM set request.
  RDM_SET_COMMAND_RESPONSE = 0x31,        // The packet is a response to an RDM set request.
} rdm_cc_t;

/**
 * @brief The response type field is used in messages from Responders to 
 * indicate the acknowledgement type of the response.
 */
typedef enum rdm_response_type {
  RDM_RESPONSE_TYPE_ACK = 0x00,          // Indicates that the responder has correctly received the controller message and is acting upon the message.
  RDM_RESPONSE_TYPE_ACK_TIMER = 0x01,    // Indicates that the responder is unable to supply the requested GET information or SET confirmation within the required response time.
  RDM_RESPONSE_TYPE_NACK_REASON = 0x02,  // Indicates that the responder is unable to reply with the requested GET information or unable to process the specified SET command.
  RDM_RESPONSE_TYPE_ACK_OVERFLOW = 0x03, // Indicates that the responder has correctly received the controller message and is acting upon the message, but there is more response data available than will fit in a single response message.
} rdm_response_type_t;

/**
 * @brief The parameter ID (PID) is a 16-bit number that identifies a specific 
 * type of parameter data. The PID may represent either a well known parameter 
 * such as those defined in the RDM standard document, or a 
 * manufacturer-specific parameter whose details are either published by the 
 * manufacturer for third-party support or proprietary for the manufacturer's 
 * own use.
 */
typedef enum rdm_pid {
  // Category: Network Management
  RDM_PID_DISC_UNIQUE_BRANCH = 0x0001,  // TODO: required
  RDM_PID_DISC_MUTE = 0x0002,  // TODO: required
  RDM_PID_DISC_UN_MUTE = 0x0003,  // TODO: required
  RDM_PID_PROXIED_DEVICES = 0x0010,
  RDM_PID_PROXIED_DEVICE_COUNT = 0x0011,
  RDM_PID_COMMS_STATUS = 0x0015,

  // Category: Status Collection
  RDM_PID_QUEUED_MESSAGE = 0x0020,  // TODO: See rdm_status_t
  RDM_PID_STATUS_MESSAGE = 0x0030,  // TODO: See rdm_status_t
  RDM_PID_STATUS_ID_DESCRIPTION = 0x0031,
  RDM_PID_CLEAR_STATUS_ID = 0x0032,
  RDM_PID_SUB_DEVICE_STATUS_REPORT_THRESHOLD = 0x0033,  // TODO: See rdm_status_t

  // Category: RDM Information
  RDM_PID_SUPPORTED_PARAMETERS = 0x0050,  // TODO: req'd if using more than minimum PIDs
  RDM_PID_PARAMETER_DESCRIPTION = 0x0051,  // TODO: req'd if using manufacturer specific PIDs
  
  // Category: Product Information
  RDM_PID_DEVICE_INFO = 0x0060,  // TODO: required
  RDM_PID_PRODUCT_DETAIL_ID_LIST = 0x0070,
  RDM_PID_DEVICE_MODEL_DESCRIPTION = 0x0080,
  RDM_PID_MANUFACTURER_LABEL = 0x0081,
  RDM_PID_DEVICE_LABEL = 0x0082,
  RDM_PID_FACTORY_DEFAULTS = 0x0090,
  RDM_PID_LANGUAGE_CAPABILITIES = 0x00a0,
  RDM_PID_LANGUAGE = 0x00b0,
  RDM_PID_SOFTWARE_VERSION_LABEL = 0x00c0,  // TODO: required
  RDM_PID_BOOT_SOFTWARE_VERSION_ID = 0x00c1,
  RDM_PID_BOOT_SOFTWARE_VERSION_LABEL = 0x00c2,

  // Category: DMX512 Setup
  RDM_PID_DMX_PERSONALITY = 0x00e0,
  RDM_PID_DMX_PERSONALITY_DESCRIPTION = 0x00e1,
  RDM_PID_DMX_START_ADDRESS = 0x00f0,  // TODO: required
  RDM_PID_SLOT_INFO = 0x0120,
  RDM_PID_SLOT_DESCRIPTION = 0x0121,
  RDM_PID_DEFAULT_SLOT_VALUE = 0x0122,

  // Category: Sensors (0x02xx)
  RDM_PID_SENSOR_DEFINITION = 0x0200,
  RDM_PID_SENSOR_VALUE = 0x0201,
  RDM_PID_RECORD_SENSORS = 0x0202,

  // Category: Dimmer Settings (0x03xx)
  // Not yet defined by ANSI/ESTA e1.20

  // Category: Power/Lamp Settings (0x04xx)
  RDM_PID_DEVICE_HOURS = 0x0400,
  RDM_PID_LAMP_HOURS = 0x0401,
  RDM_PID_LAMP_STRIKES = 0x0402,
  RDM_PID_LAMP_STATE = 0x0403,  // TODO: See rdm_lamp_state_t
  RDM_PID_LAMP_ON_MODE = 0x0404,  // TODO: See rdm_lamp_on_mode_t
  RDM_PID_DEVICE_POWER_CYCLES = 0x0405,
  
  // Category: Display Settings (0x05xx)
  RDM_PID_DISPLAY_INVERT = 0x0500,
  RDM_PID_DISPLAY_LEVEL = 0x0501,

  // Category: Configuration (0x06xx)
  RDM_PID_PAN_INVERT = 0x0600,
  RDM_PID_TILT_INVERT = 0x0601,
  RDM_PID_PAN_TILT_SWAP = 0x0602,
  RDM_PID_REAL_TIME_CLOCK = 0x0603,

  // Category: Control (0x10xx)
  RDM_PID_IDENTIFY_DEVICE = 0x1000,  // TODO: required
  RDM_PID_RESET_DEVICE = 0x1001, 
  RDM_PID_POWER_STATE = 0x1010,  // TODO: See rdm_power_state_t
  RDM_PID_PERFORM_SELF_TEST = 0x1020,  // TODO: See rdm_self_test_t
  RDM_PID_SELF_TEST_DESCRIPTION = 0x1021,
  RDM_PID_CAPTURE_PRESET = 0x1030,
  RDM_PID_PRESET_PLAYBACK = 0x1031,  // TODO: See rdm_preset_playback_t

  // Reserved for Future RDM Development: 0x7fe0-0x7fff
  // Manufacturer Specific PIDs:          0x8000-0xffdf
  // Reserved for Future RDM Development: 0xffe0-0xffff
} rdm_pid_t;

/**
 * @brief Provides a synopsis of the received RDM packet so that users may
 * quickly and easily process and respond to RDM data.
 */
typedef struct rdm_event {
  uint64_t destination_uid;             // The UID of the target device(s).
  uint64_t source_uid;                  // The UID of the device originating this packet.
  size_t tn;                            // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
  union {
    int port_id;                        // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
    rdm_response_type_t response_type;  // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
  };
  size_t message_count;                 // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
  int sub_device;                       // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
  rdm_cc_t cc;                          // The command class (CC) specifies the action of the message.
  rdm_pid_t pid;                        // The parameter ID (PID) identifies a specific type of parameter data.
  size_t pdl;                           // The parameter data length (PDL) is the number of slots included in the parameter data area that it precedes.
  bool checksum_is_valid;               // True if the RDM checksum is valid.
} rdm_event_t;

/**
 * @brief Provides a synopsis of the received DMX packet so that users may 
 * quickly and easily process and respond to DMX data.
 */
typedef struct dmx_event {
  dmx_err_t err;    // Evaluates to true if an error occurred reading DMX data. Refer to dmx_err_t to evaluate the type of error.
  uint8_t sc;       // Start code of the DMX packet.
  size_t size;      // The size of the received DMX packet in bytes.
  bool is_rdm;      // True if the received packet is RDM.
  rdm_event_t rdm;  // An RDM event struct. Is garbage data if is_rdm is false.
} dmx_event_t;

/**
 * @brief A type for evaluating RDM UIDs. Allows for easy string formatting UIDs
 * to RDM specification.
 */
typedef struct __attribute__((__packed__)) rdm_uid {
  uint32_t device_id;        // The device ID of the RDM device.
  uint16_t manufacturer_id;  // The manufacturer ID of the RDM device.
} rdm_uid_t;

/**
 * @brief A struct which can be used to help process raw RDM packets instead of 
 * reading slots by index alone. RDM sends data in most-significant byte first,
 * so endianness must be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_data {
  uint8_t sc;                  // This field shall contain the defined RDM start code. Controllers and responders shall always send RDM_SC in this slot.
  uint8_t sub_sc;              // This field shall contain the sub-start code within RDM that defines this packet structure. Unless specified in future version, the sub-start code shall be equal to RDM_SUB_SC.
  uint8_t message_len;         // The message length value is defined as the number of slots int he RDM packet including the start code and excluding the checksum.
  uint8_t destination_uid[6];  // The UID of the target device(s).
  uint8_t source_uid[6];       // The UID of the device originating this packet.
  uint8_t tn;                  // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
  union {
    uint8_t port_id;           // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
    uint8_t response_type;     // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
  };
  uint8_t message_count;       // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
  uint16_t sub_device;         // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
  uint8_t cc;                  // The command class (CC) specifies the action of the message. 
  uint16_t pid;                // The parameter ID (PID) identifies a specific type of parameter data.
  uint8_t pdl;                 // The parameter data length (PDL) is the number of slots included in the parameter data area that it precedes.
  void *pd;                    // The parameter data (PD) is of variable length. The content format is PID dependent.
} rdm_data_t;

#ifdef __cplusplus
}
#endif
