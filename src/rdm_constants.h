/**
 * @file rdm_constants.h
 * @author Mitch Weisbrod
 * @brief This file contains constants used in RDM.
 */
#pragma once

#include "dmx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID (as 
 * long as it is used responsibly) or may choose to register their own 
 * manufacturer ID.
 */
#define RDM_DEFAULT_MAN_ID (0x05e0)

/**
 * @brief This macro creates a broadcast UID for a specific manufacturer. This
 * can be used to send commands to all devices made by one manufacturer. For
 * example, commands sent to RDM_MANUFACTURER_BROADCAST_UID(0x05e0) will be
 * processed by devices that use the default manufacturer ID of this library.
 */
#define RDM_BROADCAST_MAN_UID(man_id) \
  ((((rdm_uid_t)man_id & 0xffff) << 32) | (rdm_uid_t)0xffffffff)

/**
 * @brief Returns true if a UID is a broadcast UID.
 */
#define RDM_UID_IS_BROADCAST(uid) \
  (((rdm_uid_t)uid & (rdm_uid_t)0xffffffff) == (rdm_uid_t)0xffffffff)

/**
 * @brief UID which indicates an RDM packet is being broadcast to all devices
 * regardless of manufacturer. Responders shall not respond to RDM broadcast
 * messages.
 */
static const rdm_uid_t RDM_BROADCAST_ALL_UID = RDM_BROADCAST_MAN_UID(0xffff);

/**
 * @brief The maximum RDM UID possible. Any UID above this value (except
 * RDM_BROADCAST_ALL_UID) is considered invalid.
 */
static const rdm_uid_t RDM_MAX_UID = 0xfffffffffffe;

/**
 * @brief The RDM command class (CC) type. The command class specifies the 
 * action of the message. Responders shall always generate a response to 
 * GET_COMMAND and SET_COMMAND messages except when the destination UID of the
 * message is a broadcast address. Responders shall not respond to commands sent 
 * using broadcast addressing, in order to prevent collisions.
 */
typedef enum rdm_cc_t {
  RDM_CC_NON_RDM_PACKET = 0x00,         // The packet is a non-RDM packet.
  RDM_CC_DISC_COMMAND = 0x10,           // The packet is an RDM discovery command.
  RDM_CC_DISC_COMMAND_RESPONSE = 0x11,  // The packet is a response to an RDM discovery command.
  RDM_CC_GET_COMMAND = 0x20,            // The packet is an RDM get request.
  RDM_CC_GET_COMMAND_RESPONSE = 0x21,   // The packet is a response to an RDM get request.
  RDM_CC_SET_COMMAND = 0x30,            // The packet is an RDM set request.
  RDM_CC_SET_COMMAND_RESPONSE = 0x31,   // The packet is a response to an RDM set request.
} rdm_cc_t;

/**
 * @brief The response type field is used in messages from Responders to 
 * indicate the acknowledgement type of the response.
 */
typedef enum rdm_response_type_t {
  RDM_RESPONSE_TYPE_NONE = -1, // TODO: docs

  RDM_RESPONSE_TYPE_ACK = 0x00,          // Indicates that the responder has correctly received the controller message and is acting upon the message.
  RDM_RESPONSE_TYPE_ACK_TIMER = 0x01,    // Indicates that the responder is unable to supply the requested GET information or SET confirmation within the required response time.
  RDM_RESPONSE_TYPE_NACK_REASON = 0x02,  // Indicates that the responder is unable to reply with the requested GET information or unable to process the specified SET command.
  RDM_RESPONSE_TYPE_ACK_OVERFLOW = 0x03, // Indicates that the responder has correctly received the controller message and is acting upon the message, but there is more response data available than will fit in a single response message.
} rdm_response_type_t;

/**
 * @brief The NACK reason defines the reason that the responder is unable to
 * comply with the request.
 */
typedef enum rdm_nr_t {
  RDM_NR_UNKNOWN_PID = 0x0000,                // The responder cannot comply with the request because the message is not implemented in the responder.
  RDM_NR_FORMAT_ERROR = 0x0001,               // The responder cannot interpret the request as the controller data was not formatted correctly.
  RDM_NR_HARDWARE_FAULT = 0x0002,             // The responder cannot comply due to an internal hardware fault.
  RDM_NR_PROXY_REJECT = 0x0003,               // Proxy is not the RDM line master and cannot comply with the message.
  RDM_NR_WRITE_PROTECT = 0x0004,              // Set command normally allowed but being blocked currently.
  RDM_NR_UNSUPPORTED_COMMAND_CLASS = 0x0005,  // Not valid for command class attempted. May be used where get allowed but set is not supported.
  RDM_NR_DATA_OUT_OF_RANGE = 0x0006,          // Value for given parameter out of allowable range or not supported.
  RDM_NR_BUFFER_FULL = 0x0007,                // Buffer or queue space currently has no free space to store data.
  RDM_NR_PACKET_SIZE_UNSUPPORTED = 0x0008,    // Incoming message exceeds buffer capacity.
  RDM_NR_SUB_DEVICE_OUT_OF_RANGE = 0x0009,    // Sub-device is out of range or unknown.
  RDM_NR_PROXY_BUFFER_FULL = 0x000a           // The proxy buffer is full and cannot store any more queued message or status message responses.
} rdm_nr_t;

/**
 * @brief The parameter ID (PID) is a 16-bit number that identifies a specific 
 * type of parameter data. The PID may represent either a well known parameter 
 * such as those defined in the RDM standard document, or a 
 * manufacturer-specific parameter whose details are either published by the 
 * manufacturer for third-party support or proprietary for the manufacturer's 
 * own use.
 */
typedef enum rdm_pid_t {
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

#ifdef __cplusplus
}
#endif
