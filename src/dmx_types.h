/**
 * @file dmx_types.h
 * @author Mitch Weisbrod
 * @brief This file contains the types used for processing DMX and RDM as needed
 * for the base DMX driver. Types that are only used in rdm_tools.h should be
 * defined in rdm_types.h instead. Anonymous enums and constants defined in the
 * DMX or RDM standard should be defined in dmx_constants.h or rdm_constants.h
 * instead.
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
} dmx_sniffer_data_t;

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
 * @brief Provides a synopsis of the received RDM packet so that users may
 * quickly and easily process and respond to RDM data.
 */
typedef struct rdm_event {
  uint64_t destination_uid;             // The UID of the target device(s).
  uint64_t source_uid;                  // The UID of the device originating this packet.
  size_t tn;                            // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
  union {
    int port_id;                        // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
    int response_type;  // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
  };
  size_t message_count;                 // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
  int sub_device;                       // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
  int cc;                          // The command class (CC) specifies the action of the message.
  int pid;                        // The parameter ID (PID) identifies a specific type of parameter data.
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
