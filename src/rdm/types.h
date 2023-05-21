/**
 * @file types.h
 * @author Mitch Weisbrod
 * @brief This file contains constants used in RDM.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RDM unique ID type.
 */
typedef uint64_t rdm_uid_t;

typedef uint16_t rdm_sub_device_t;

/**
 * @brief Macro for creating a manufacturer broadcast UID based on the desired
 * manufacturer ID.
 */
#define RDM_UID_BROADCAST_MAN(man_id) (((rdm_uid_t)man_id << 32) | 0xffffffff)

#if ESP_IDF_VERSION_MAJOR >= 5
/**
 * @brief The recommended method for representing the UID in text by separating
 * the manufacturer ID and the device ID. For use with printf-like functions.
 */
#define UIDSTR "%04x:%08lx"
#else
// TODO: can we remove this one?
/**
 * @brief The recommended method for representing the UID in text by separating
 * the manufacturer ID and the device ID. For use with printf-like functions.
 */
#define UIDSTR "%04x:%08x"
#endif

/**
 * @brief Used to generate arguments for the UIDSTR macro for representing the
 * UID in text by separating the manufacturer ID and device ID. For use with
 * printf-like functions.
 */
#define UID2STR(uid) ((uint16_t)(uid >> 32)), ((uint32_t)(uid))

/**
 * @brief UID which indicates an RDM packet is being broadcast to all devices
 * regardless of manufacturer. Responders shall not respond to RDM broadcast
 * messages.
 */
static const rdm_uid_t RDM_UID_BROADCAST_ALL = 0xffffffffffff;

/**
 * @brief The maximum RDM UID possible. Any UID above this value (except for a 
 * broadcast UID) is considered invalid.
 */
static const rdm_uid_t RDM_UID_MAX = 0xfffffffffffe;

/**
 * @brief Sub-device which respresents the root of a RDM device.
 */
static const rdm_sub_device_t RDM_SUB_DEVICE_ROOT = 0;

/**
 * @brief The RDM sub-device number which can be used to address all sub-devices
 * of an RDM device in a request.
 */
static const rdm_sub_device_t RDM_SUB_DEVICE_ALL = 0xffff;

/**
 * @brief RDM controllers may send GET and SET requests for the
 * RDM_PID_DMX_START_ADDRESS of this device. When an RDM responder has a DMX
 * footprint of 0, the DMX start address of this device must be set to this
 * value.
 */
static const uint16_t DMX_START_ADDRESS_NONE = 0xffff;

/**
 * @brief The RDM command class (CC) type. The command class specifies the
 * action of the message. Responders shall always generate a response to
 * GET_COMMAND and SET_COMMAND messages except when the destination UID of the
 * message is a broadcast address. Responders shall not respond to commands sent
 * using broadcast addressing, in order to prevent collisions.
 */
typedef enum rdm_cc_t {
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
  RDM_RESPONSE_TYPE_NONE = -1,           // Indicates that a response was not received.
  RDM_RESPONSE_TYPE_INVALID = -2,        // Indicates that a response was received, but it was invalid.

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
  RDM_PID_DISC_UNIQUE_BRANCH = 0x0001,  // Discovery Unique Branch. This parameter is used for the device discovery process. @note Does not support GET or SET. Must only be sent to RDM_BROADCAST_ALL_UIDS. Must only be sent to root devices.
  RDM_PID_DISC_MUTE = 0x0002,  // A responder port shall set its Mute flag when it receives this message containing its UID, or a broadcast address. @note Does not support GET or SET. Must only be sent to root devices.
  RDM_PID_DISC_UN_MUTE = 0x0003,  // A responder port shall clear its Mute flag when it receives this message containing its UID, or a broadcast address. @note Does not support GET or SET. Must only be sent to root devices.

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
  RDM_PID_SUPPORTED_PARAMETERS = 0x0050,  // This parameter is used to retrieve a list of supported PIDs. @note Supports GET.
  RDM_PID_PARAMETER_DESCRIPTION = 0x0051,  // TODO: req'd if using manufacturer specific PIDs
  
  // Category: Product Information
  RDM_PID_DEVICE_INFO = 0x0060,  // This parameter is used to retrieve a variety of information about the device that is normally required by a controller. @note Supports GET.
  RDM_PID_PRODUCT_DETAIL_ID_LIST = 0x0070,
  RDM_PID_DEVICE_MODEL_DESCRIPTION = 0x0080,
  RDM_PID_MANUFACTURER_LABEL = 0x0081,
  RDM_PID_DEVICE_LABEL = 0x0082,
  RDM_PID_FACTORY_DEFAULTS = 0x0090,
  RDM_PID_LANGUAGE_CAPABILITIES = 0x00a0,
  RDM_PID_LANGUAGE = 0x00b0,
  RDM_PID_SOFTWARE_VERSION_LABEL = 0x00c0,  // This parameter is used to get a descriptive ASCII text label for the device‘s operating software version. The descriptive text returned by this parameter is intended for display to the user. @note Supports GET.
  RDM_PID_BOOT_SOFTWARE_VERSION_ID = 0x00c1,
  RDM_PID_BOOT_SOFTWARE_VERSION_LABEL = 0x00c2,

  // Category: DMX512 Setup
  RDM_PID_DMX_PERSONALITY = 0x00e0,
  RDM_PID_DMX_PERSONALITY_DESCRIPTION = 0x00e1,
  RDM_PID_DMX_START_ADDRESS = 0x00f0,  // This parameter is used to set or get the DMX512 start address. @note Supports GET and SET.
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
  RDM_PID_IDENTIFY_DEVICE = 0x1000,  // This parameter is used for the user to physically identify the device represented by the UID. @note Supports GET and SET.
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
 * @brief Provides information about RDM responses.
 */
typedef struct rdm_ack_t {  
  esp_err_t err;  // Evaluates to true if an error occurred reading RDM data.
  rdm_response_type_t type;  // The type of the RDM response received.
  union {
    int num;               // The number of parameters received. This field should be read when the response type received is RDM_RESPONSE_TYPE_ACK or RDM_RESPONSE_TYPE_ACK_OVERFLOW.
    TickType_t timer;      // The amount of time in FreeRTOS ticks until the responder device will be ready to respond to the request. This field should be read when the response type received is RDM_RESPONSE_TYPE_ACK_TIMER.
    rdm_nr_t nack_reason;  // The reason that the request was unable to be fulfilled. This field should be read when the response type received is RDM_RESPONSE_TYPE_NACK_REASON.
  };
} rdm_ack_t;

/**
 * @brief Parameters for use in RDM discovery requests. Discovery requests are
 * broadcast messages with a lower bound and upper bound. If a responding
 * device's UID falls within the lower bound and upper bound, it will respond to
 * the discovery request.
 */
typedef struct rdm_disc_unique_branch_t {
  rdm_uid_t lower_bound;  // The lower bound of the RDM discovery request.
  rdm_uid_t upper_bound;  // The upper bound of the RDM discovery request.
} rdm_disc_unique_branch_t;

/**
 * @brief Parameters for use with RDM discovery mute and un-mute requests. When
 * a responder device is successfully muted or un-muted, it responds with these
 * parameters.
 */
typedef struct rdm_disc_mute_t {
  bool managed_proxy;     // The manged proxy flag shall be set to 1 when the responder is a proxy device.
  bool sub_device;        // The sub-device flag shall be set to 1 when the responder supports sub-devices.
  bool boot_loader;       // The boot-loader flag shall only be set to 1 when the device is incapable of normal operation until receiving a firmware upload.
  bool proxied_device;    // The proxied device flag shall only be set to 1 when a proxy is responding to discovery on behalf of another device. This flag indicates that the response has come from a proxy rather than the actual device.
  rdm_uid_t binding_uid;  // The binding UID field shall only be included when the responding device contains multiple responder ports. If the device does contain multiple ports then the binding UID field shall contain the UID for the primary port on the device. If the device does not contain multiple responder ports, this field is set to 0.
} rdm_disc_mute_t;

/**
 * @brief Parameter for use with RDM device info requests.
 */
typedef struct rdm_device_info_t {
  int model_id;                  // This field identifies the device model ID of the root device or sub-device. The manufacturer shall not use the same ID to represent more than one unique model type.
  int product_category;          // Devices shall report a product category based on the product's primary function.
  uint32_t software_version_id;  // This field indicates the software version ID for the device. The software version ID is a 32-bit value determined by the manufacturer.
  size_t footprint;              // This field species the DMX footprint - the number of consecutive DMX slots required.
  int current_personality;       // The current selected DMX personality of the device. The personality is the configured arrangement of DMX slots used by the device. Many devices may have multiple personalities from which to choose.
  int personality_count;         // The number of personalities supported by the device. The personality is the configured arrangement of DMX slots used by the device. Many devices may have multiple personalities from which to choose.
  int start_address;             // The DMX start address of the device. If the device or sub-device that the request is directed to has a DMX footprint of 0, then this field shall be set to -1.
  int sub_device_count;          // This parameter is used to retrieve the number of sub-devices respresented by the root device. The response for this field shall always be the same regardless of whether this message is directed to the root device or a sub-device.
  int sensor_count;              // This field indicates the number of available sensors in a root device or sub-device. When this parameter is directed to a sub-device, the reply shall be identical for any sub-device owned by a specific root device.
} rdm_device_info_t;

/**
 * @brief A struct which stores RDM packet header information. Header
 * information contains metadata about the packet message data block. 
 */
typedef struct rdm_header_t {
  rdm_uid_t dest_uid;  // The UID of the target device(s).
  rdm_uid_t src_uid;   // The UID of the device originating this packet.
  int tn;  // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
  union {
    int port_id;  // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
    rdm_response_type_t response_type;  // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
  };
  int message_count;  // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
  rdm_sub_device_t sub_device;  // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
  rdm_cc_t cc;  // The command class (CC) specifies the action of the message.
  rdm_pid_t pid;  // The parameter ID (PID) identifies a specific type of parameter data.
} rdm_header_t;

/**
 * @brief A struct which stores RDM message data block information. This is the
 * main payload of an RDM packet. Information in the message data block is
 * typically encoded and decoded with functions found in `mdb.h`.
 *
 */
typedef struct rdm_mdb_t {
  size_t pdl;  // The parameter data length (PDL) is the number of slots included in the parameter data area that it precedes.
  union {
    size_t preamble_len;  // The preamble length is the number of preamble bytes (excluding the delimiter) in a DISC_UNIQUE_BRANCH response packet.
    uint8_t pd[231];  // The parameter data (PD) is the data section of the packet. Its length is included in its PDL.
  };
} rdm_mdb_t;

/**
 * @brief A struct which contains instructions on encoding data into an RDM 
 * message data block.
 */
typedef struct rdm_encode_t {
  size_t (*function)(rdm_mdb_t *, const void *, int);  // The encode function.
  const void *params;  // A pointer to a parameter or parameters to encode.
  int num;  // The number of parameters to encode.
} rdm_encode_t;

/**
 * @brief A struct which contains instructions on decoded data from an RDM
 * message data block.
 */
typedef struct rdm_decode_t {
  int (*function)(const rdm_mdb_t *, void *, int);  // The decode function.
  void *params;  // A pointer to a parameter or parameters to decode.
  int num;  // The number of parameters to decode.
} rdm_decode_t;

/**
 * @brief A function type for RDM responder callbacks. This is the type of
 * function that is called when responding to RDM requests.
 */
typedef rdm_response_type_t (*rdm_response_cb_t)(dmx_port_t dmx_num,
                                                 const rdm_header_t *header,
                                                 rdm_mdb_t *mdb, void *context);

typedef enum rdm_product_category_t {
  RDM_PRODUCT_CATEGORY_NOT_DECLARED = 0x0000,  // The product category is not declared.

  RDM_PRODUCT_CATEGORY_FIXTURE = 0x0100,  // The product is a fixture intended to create illumination.

  RDM_PRODUCT_CATEGORY_FIXTURE_ACCESSORY = 0x0200,  // The product is an add-on to a fixture or projector.

  RDM_PRODUCT_CATEGORY_PROJECTOR = 0x0300,  // The product is a light source capable of producing realistic images from another media.

  RDM_PRODUCT_CATEGORY_ATMOSPHERIC = 0x0400,  // The product creates atmospheric effects such as haze, fog, or pyrotechnics.

  RDM_PRODUCT_CATEGORY_DIMMER = 0x0500,  // The product is for intensity control, specifically dimming equipment.

  RDM_PRODUCT_CATEGORY_POWER = 0x0600,  // The product is for power control, other than dimming equipment.

  RDM_PRODUCT_CATEGORY_SCENIC = 0x0700,  // The product is a scenic device unrelated to lighting equipment.

  RDM_PRODUCT_CATEGORY_DATA = 0x0800, // The product is a DMX converter, interface, or otherwise part of DMX infrastructure.

  RDM_PRODUCT_CATEGORY_AV = 0x0900,  // The product is audio-visual equipment.

  RDM_PRODUCT_CATEGORY_MONITOR = 0x0a00,  // The product is monitoring equipment.

  RDM_PRODUCT_CATEGORY_CONTROL = 0x7000,  // The product is a controller or backup device.

  RDM_PRODUCT_CATEGORY_TEST = 0x7100,  // The product is test equipment.

  RDM_PRODUCT_CATEGORY_OTHER = 0x7fff  // The product isn't described by any of the other product categories.

  // Manufacturer Specific Categories: 0x8000-0xdfff
} rdm_product_category_t;

#ifdef __cplusplus
}
#endif
