/**
 * @file rdm/types.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file contains constants and types used in RDM.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dmx/types.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Macro for creating a manufacturer broadcast UID based on the desired
 * manufacturer ID.*/
#define rdm_uid_broadcast_man(man_id) ((rdm_uid_t){(man_id), 0xffffffff})

/** @brief The recommended method for representing the UID in text by separating
 * the manufacturer ID and the device ID. For use with printf-like functions.*/
#define UIDSTR "%04x:%08lx"

/** @brief Used to generate arguments for the UIDSTR macro for representing the
 * UID in text by separating the manufacturer ID and device ID. For use with
 * printf-like functions.*/
#define UID2STR(uid) (uid).man_id, (uid).dev_id

/** @brief RDM sub-device type.*/
typedef enum rdm_sub_device_t {
  /** @brief Sub-device which respresents the root of a RDM device.*/
  RDM_SUB_DEVICE_ROOT = 0,
  /** @brief The RDM sub-device number which can be used to address all
     sub-devices of an RDM device in a request.*/
  RDM_SUB_DEVICE_ALL = 0xffff
} rdm_sub_device_t;

/** @brief The RDM command class (CC) type. The command class specifies the
 * action of the message. Responders shall always generate a response to
 * GET_COMMAND and SET_COMMAND messages except when the destination UID of the
 * message is a broadcast address. Responders shall not respond to commands sent
 * using broadcast addressing, in order to prevent collisions.*/
typedef enum rdm_cc_t {
  /** @brief The packet is an RDM discovery command.*/
  RDM_CC_DISC_COMMAND = 0x10,
  /** @brief The packet is a response to an RDM discovery command.*/
  RDM_CC_DISC_COMMAND_RESPONSE = 0x11,
  /** @brief The packet is an RDM GET request.*/
  RDM_CC_GET_COMMAND = 0x20,
  /** @brief The packet is a response to an RDM GET request.*/
  RDM_CC_GET_COMMAND_RESPONSE = 0x21,
  /** @brief The packet is an RDM SET request.*/
  RDM_CC_SET_COMMAND = 0x30,
  /** @brief The packet is a response to an RDM SET request.*/
  RDM_CC_SET_COMMAND_RESPONSE = 0x31,
} rdm_cc_t;

/** @brief The parameter ID (PID) is a 16-bit number that identifies a specific
 * type of parameter data. The PID may represent either a well known parameter
 * such as those defined in the RDM standard document, or a
 * manufacturer-specific parameter whose details are either published by the
 * manufacturer for third-party support or proprietary for the manufacturer's
 * own use.*/
typedef enum rdm_pid_t {
  // Category: Network Management

  /** @brief Discovery Unique Branch. This parameter is used for the device
     discovery process. @note Does not support GET nor SET. Must only be sent to
     RDM_UID_BROADCAST_ALL. Must only be sent to RDM_SUB_DEVICE_ROOT. This
     parameter is required.*/
  RDM_PID_DISC_UNIQUE_BRANCH = 0x0001,
  /** @brief A responder port shall set its Mute flag when it receives this
     message containing its UID, or a broadcast address. @note Does not support
     GET nor SET. Must only be sent to RDM_SUB_DEVICE_ROOT. This parameter is
     required.*/
  RDM_PID_DISC_MUTE = 0x0002,
  /** @brief A responder port shall clear its Mute flag when it receives this
     message containing its UID, or a broadcast address. @note Does not support
     GET nor SET. Must only be sent to RDM_SUB_DEVICE_ROOT. This parameter is
     required.*/
  RDM_PID_DISC_UN_MUTE = 0x0003,

  /** @brief This parameter is used to retrieve the UIDs from a device
     identified as a proxy during discovery. The response to this parameter
     contains a packed list of 48-bit UIDs for all devices represented by the
     proxy. @note Supports GET. Must only be sent to RDM_SUB_DEVICE_ROOT.*/
  RDM_PID_PROXIED_DEVICES = 0x0010,
  /** @brief This parameter is used to identify the number of devices being
     represented by a proxy and whether the list of represented device UIDs has
     changed. If the list change flag is set then the controller should GET
     RDM_PID_PROXIED_DEVICES. The device shall automatically clear the list
     change flag after all the proxied UID's have been retrieved using the GET
     RDM_PID_PROXIED_DEVICES message. @note Supports GET. Must only be sent to
     RDM_SUB_DEVICE_ROOT.*/
  RDM_PID_PROXIED_DEVICE_COUNT = 0x0011,
  /** @brief The RDM_PID_COMMS_STATUS parameter is used to collect information
     that may be useful in analyzing the integrity of the communication
     system. @note Supports GET and SET. Must only be sent to
     RDM_SUB_DEVICE_ROOT.*/
  RDM_PID_COMMS_STATUS = 0x0015,

  // Category: Status Collection
  
  /** @brief The RDM_PID_QUEUED_MESSAGE parameter shall be used to retrieve a
     message from the responder's message queue. The message count field of all
     response messages defines the number of messages that are queued in the
     responder. Each RDM_PID_QUEUED_MESSAGE response shall be composed of a
     single message response.*/
  RDM_PID_QUEUED_MESSAGE = 0x0020,
  RDM_PID_STATUS_MESSAGE = 0x0030,
  RDM_PID_STATUS_ID_DESCRIPTION = 0x0031,
  RDM_PID_CLEAR_STATUS_ID = 0x0032,
  RDM_PID_SUB_DEVICE_STATUS_REPORT_THRESHOLD = 0x0033,

  // Category: RDM Information

  /** @brief This parameter is used to retrieve a list of supported PIDs. @note
     Supports GET. This parameter is required if supporting parameters beyond
     the minimum required set.*/
  RDM_PID_SUPPORTED_PARAMETERS = 0x0050,
  /** @brief This parameter is used to retrieve the definition of some
    manufacturer-specific PIDs. The purpose of this parameter is to allow a
    controller to retrieve enough information about the manufacturerspecific PID
    to generate GET and SET commands. @note Supports GET. This parameter is
    required if using manufacturer-specific PIDs.*/
  RDM_PID_PARAMETER_DESCRIPTION = 0x0051,

  // Category: Product Information

  /** @brief This parameter is used to retrieve a variety of information about
     the device that is normally required by a controller. @note Supports GET.
     This parameter is required.*/
  RDM_PID_DEVICE_INFO = 0x0060,
  RDM_PID_PRODUCT_DETAIL_ID_LIST = 0x0070,
  RDM_PID_DEVICE_MODEL_DESCRIPTION = 0x0080,
  RDM_PID_MANUFACTURER_LABEL = 0x0081,
  RDM_PID_DEVICE_LABEL = 0x0082,
  RDM_PID_FACTORY_DEFAULTS = 0x0090,
  RDM_PID_LANGUAGE_CAPABILITIES = 0x00a0,
  RDM_PID_LANGUAGE = 0x00b0,
  /** @brief This parameter is used to get a descriptive ASCII text label for
     the device's operating software version. The descriptive text returned by
     this parameter is intended for display to the user. @note Supports GET.
     This parameter is required.*/
  RDM_PID_SOFTWARE_VERSION_LABEL = 0x00c0,
  RDM_PID_BOOT_SOFTWARE_VERSION_ID = 0x00c1,
  RDM_PID_BOOT_SOFTWARE_VERSION_LABEL = 0x00c2,

  // Category: DMX512 Setup
  RDM_PID_DMX_PERSONALITY = 0x00e0,
  RDM_PID_DMX_PERSONALITY_DESCRIPTION = 0x00e1,
  /** @brief This parameter is used to set or get the DMX512 start address.
     @note Supports GET and SET. This parameter is required if the device uses a
     DMX slot.*/
  RDM_PID_DMX_START_ADDRESS = 0x00f0,
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
  RDM_PID_LAMP_STATE = 0x0403,    // TODO: See rdm_lamp_state_t
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

  /** @brief This parameter is used for the user to physically identify the
     device represented by the UID. @note Supports GET and SET. This parameter
     is required.*/
  RDM_PID_IDENTIFY_DEVICE = 0x1000,
  RDM_PID_RESET_DEVICE = 0x1001,
  RDM_PID_POWER_STATE = 0x1010,        // TODO: See rdm_power_state_t
  RDM_PID_PERFORM_SELF_TEST = 0x1020,  // TODO: See rdm_self_test_t
  RDM_PID_SELF_TEST_DESCRIPTION = 0x1021,
  RDM_PID_CAPTURE_PRESET = 0x1030,
  RDM_PID_PRESET_PLAYBACK = 0x1031,  // TODO: See rdm_preset_playback_t

  // Reserved for Future RDM Development: 0x7fe0-0x7fff
  // Manufacturer Specific PIDs:          0x8000-0xffdf
  RDM_PID_MANUFACTURER_SPECIFIC_BEGIN = 0x8000,
  RDM_PID_MANUFACTURER_SPECIFIC_END = 0xffdf,
  // Reserved for Future RDM Development: 0xffe0-0xffff
} rdm_pid_t;

/** @brief The response type field is used in messages from Responders to
 * indicate the acknowledgement type of the response.*/
typedef enum rdm_response_type_t {
  /** @brief Indicates that a response was received, but it was invalid.*/
  RDM_RESPONSE_TYPE_INVALID = 0xfe,
  /** @brief Indicates that a response was not received.*/
  RDM_RESPONSE_TYPE_NONE = 0xff,

  /** @brief Indicates that the responder has correctly received the controller
     message and is acting upon the message.*/
  RDM_RESPONSE_TYPE_ACK = 0x00,
  /** @brief Indicates that the responder is unable to supply the requested GET
     information or SET confirmation within the required response time.*/
  RDM_RESPONSE_TYPE_ACK_TIMER = 0x01,
  /** @brief Indicates that the responder is unable to reply with the requested
     GET information or unable to process the specified SET command.*/
  RDM_RESPONSE_TYPE_NACK_REASON = 0x02,
  /** @brief Indicates that the responder has correctly received the controller
     message and is acting upon the message, but there is more response data
     available than will fit in a single response message.*/
  RDM_RESPONSE_TYPE_ACK_OVERFLOW = 0x03,
} rdm_response_type_t;

/** @brief Status collection messages include messages used to retrieve deferred
 * (queued) responses, device status and error information, and information
 * regarding the RDM parameters supported by the device. Status collection
 * messages are normally addressed to root devices. The status type is used to
 * identify the severity of the condition.*/
typedef enum rdm_status_t {
  /** @brief The status type of RDM_STATUS_NONE shall be used when a controller
     wants to establish whether a device is present on the network without
     retrieving any status message data from the device. Not allowed for use
     with GET RDM_PID_QUEUED_MESSAGE.*/
  RDM_STATUS_NONE = 0x00,
  /** @brief If the status type requested is RDM_STATUS_GET_LAST_MESSAGE, the
     responder shall return the last message (which may be either a queued
     message or a status message) sent in response to a GET
     RDM_PID_QUEUED_MESSAGE.*/
  RDM_STATUS_GET_LAST_MESSAGE = 0x01,
  /** @brief The sub-device has an advisory or informational status message.*/
  RDM_STATUS_ADVISORY = 0x02,
  /** @brief The sub-device has a warning status message indicating a
     recoverable error has occurred.*/
  RDM_STATUS_WARNING = 0x03,
  /** @brief The sub-device has an error status message indicating a
     non-recoverable or fatal error has occurred.*/
  RDM_STATUS_ERROR = 0x04,
  /** @brief The sub-device previously had an advisory or informational status
     message but it has been cleared.*/
  RDM_STATUS_ADVISORY_CLEARED = 0x12,
  /** @brief The sub-device previously had a warning status message but it has
     been cleared.*/
  RDM_STATUS_WARNING_CLEARED = 0x13,
  /** @brief The sub-device previously had an error status message but it has
     been cleared.*/
  RDM_STATUS_ERROR_CLEARED = 0x14
} rdm_status_t;

/** @brief The NACK reason defines the reason that the responder is unable to
 * comply with the request.*/
typedef enum rdm_nr_t {
  /** @brief The responder cannot comply with the request because the message is
     not implemented in the responder.*/
  RDM_NR_UNKNOWN_PID = 0x0000,
  /** @brief The responder cannot interpret the request as the controller data
     was not formatted correctly.*/
  RDM_NR_FORMAT_ERROR = 0x0001,
  /** @brief The responder cannot comply due to an internal hardware fault.*/
  RDM_NR_HARDWARE_FAULT = 0x0002,
  /** @brief Proxy is not the RDM line master and cannot comply with the
     message.*/
  RDM_NR_PROXY_REJECT = 0x0003,
  /** @brief Set command normally allowed but being blocked currently.*/
  RDM_NR_WRITE_PROTECT = 0x0004,
  /** @brief Not valid for command class attempted. May be used where get
     allowed but set is not supported.*/
  RDM_NR_UNSUPPORTED_COMMAND_CLASS = 0x0005,
  /** @brief Value for given parameter out of allowable range or not
     supported.*/
  RDM_NR_DATA_OUT_OF_RANGE = 0x0006,
  /** @brief Buffer or queue space currently has no free space to store data.*/
  RDM_NR_BUFFER_FULL = 0x0007,
  /** @brief Incoming message exceeds buffer capacity.*/
  RDM_NR_PACKET_SIZE_UNSUPPORTED = 0x0008,
  /** @brief Sub-device is out of range or unknown.*/
  RDM_NR_SUB_DEVICE_OUT_OF_RANGE = 0x0009,
  /** @brief The proxy buffer is full and cannot store any more queued message
     or status message responses.*/
  RDM_NR_PROXY_BUFFER_FULL = 0x000a
} rdm_nr_t;

/** @brief The PID command class defines whether GET and or SET messages are
 * implemented for a specified PID.*/
typedef enum rdm_pid_cc_t {
  /** @brief PID supports DISC only.*/
  RDM_CC_DISC = 0x00,
  /** @brief PID supports GET only.*/
  RDM_CC_GET = 0x01,
  /** @brief PID supports SET only.*/
  RDM_CC_SET = 0x02,
  /** @brief PID supports GET and SET.*/
  RDM_CC_GET_SET = 0x03
} rdm_pid_cc_t;

/** @brief Devices shall report a product category based on the product's
 * primary function.*/
typedef enum rdm_product_category_t {
  /** @brief The product category is not declared.*/
  RDM_PRODUCT_CATEGORY_NOT_DECLARED = 0x0000,

  /** @brief The product is a fixture intended to create illumination.*/
  RDM_PRODUCT_CATEGORY_FIXTURE = 0x0100,

  /** @brief The product is an add-on to a fixture or projector.*/
  RDM_PRODUCT_CATEGORY_FIXTURE_ACCESSORY = 0x0200,

  /** @brief The product is a light source capable of producing realistic images
     from another media.*/
  RDM_PRODUCT_CATEGORY_PROJECTOR = 0x0300,

  /** @brief The product creates atmospheric effects such as haze, fog, or
     pyrotechnics.*/
  RDM_PRODUCT_CATEGORY_ATMOSPHERIC = 0x0400,

  /** @brief The product is for intensity control, specifically dimming
     equipment.*/
  RDM_PRODUCT_CATEGORY_DIMMER = 0x0500,

  /** @brief The product is for power control, other than dimming equipment.*/
  RDM_PRODUCT_CATEGORY_POWER = 0x0600,

  /** @brief The product is a scenic device unrelated to lighting equipment.*/
  RDM_PRODUCT_CATEGORY_SCENIC = 0x0700,

  /** @brief The product is a DMX converter, interface, or otherwise part of DMX
     infrastructure.*/
  RDM_PRODUCT_CATEGORY_DATA = 0x0800,

  /** @brief The product is audio-visual equipment.*/
  RDM_PRODUCT_CATEGORY_AV = 0x0900,

  /** @brief The product is monitoring equipment.*/
  RDM_PRODUCT_CATEGORY_MONITOR = 0x0a00,

  /** @brief The product is a controller or backup device.*/
  RDM_PRODUCT_CATEGORY_CONTROL = 0x7000,

  /** @brief The product is test equipment.*/
  RDM_PRODUCT_CATEGORY_TEST = 0x7100,

  /** @brief The product isn't described by any of the other product
     categories.*/
  RDM_PRODUCT_CATEGORY_OTHER = 0x7fff

  // Manufacturer Specific Categories: 0x8000-0xdfff
} rdm_product_category_t;

/** @brief Data type defines the size of the data entries in the parameter data
 * of an RDM message for a given PID.*/
typedef enum rdm_ds_t {
  /** @brief Data type is not defined.*/
  RDM_DS_NOT_DEFINED = 0x00,
  /** @brief Data is bit packed.*/
  RDM_DS_BIT_FIELD = 0x01,
  /** @brief Data is a string.*/
  RDM_DS_ASCII = 0x02,
  /** @brief Data is an array of unsigned bytes.*/
  RDM_DS_UNSIGNED_BYTE = 0x03,
  /** @brief Data is an array of signed bytes.*/
  RDM_DS_SIGNED_BYTE = 0x04,
  /** @brief Data is an array of unsigned 16-bit words.*/
  RDM_DS_UNSIGNED_WORD = 0x05,
  /** @brief Data is an array of signed 16-bit words.*/
  RDM_DS_SIGNED_WORD = 0x06,
  /** @brief Data is an array of unsigned 32-bit words.*/
  RDM_DS_UNSIGNED_DWORD = 0x07,
  /** @brief Data is an array of signed 32-bit words.*/
  RDM_DS_SIGNED_DWORD = 0x08,

  // Manufacturer Specific Data Types: 0x80-0xdf
} rdm_ds_t;

/** @brief The units define the SI unit of a specific PID.*/
typedef enum rdm_units_t {
  /** @brief An SI unit is not used.*/
  RDM_UNITS_NONE = 0x00,
  /** @brief The unit is bytes. When a prefix is used with this unit, the
     multiplier refers to binary multiple. i.e. KILO means multiply by 1024.*/
  RDM_UNITS_BYTES = 0x1c
} rdm_units_t;

/** @brief The prefix defines the SI prefix and multiplication factor of the
 * units*/
typedef enum rdm_prefix_t {
  /** @brief Multiply by 1.*/
  RDM_PREFIX_NONE = 0x00,
  /** @brief Multiply by 10E+24.*/
  RDM_PREFIX_YOTTA = 0x1a
} rdm_prefix_t;

/** @brief Responders and controllers identify themselves with a 48-bit Unique
 * ID (UID). The UID consists of a 16-bit ESTA assigned manufacturer ID with a
 * 32-bit device ID.*/
typedef struct __attribute__((packed)) rdm_uid_t {
  /** @brief The 16-bit manufacturer ID identifies a device's manufacturer. It
     shall be restricted to 0x0001 through 0x7fff (inclusive).*/
  uint16_t man_id;
  /** @brief The 32-bit device ID shall be unique throughout all products
     manufactured under a specific manufacturer ID, to ensure that no two
     devices with the same UID will appear on the data link*/
  uint32_t dev_id;
} rdm_uid_t;

/** @brief Provides information about RDM responses.*/
typedef struct rdm_ack_t {
  /** @brief Evaluates to true if an error occurred reading DMX data.*/
  dmx_err_t err;
  /** @brief The size of the packet received.*/
  size_t size;
  /** @brief The UID of the device originating the response packet.*/
  rdm_uid_t src_uid;
  /** @brief The PID of the response packet. It is typically the same PID as the
   * RDM request. */
  rdm_pid_t pid;
  /** @brief The type of the RDM response received.*/
  rdm_response_type_t type;
  /** @brief The message count field is used by a responder to indicate that
       additional data is now available for collection by a controller.*/
  int message_count;
  union {
    /** @brief The amount of time in FreeRTOS ticks until the responder device
       will be ready to respond to the request. This field should be read when
       the response type received is RDM_RESPONSE_TYPE_ACK_TIMER.*/
    TickType_t timer;
    /** @brief The reason that the request was unable to be fulfilled. This
       field should be read when the response type received is
       RDM_RESPONSE_TYPE_NACK_REASON.*/
    rdm_nr_t nack_reason;
  };
} rdm_ack_t;

/** @brief A struct which stores RDM packet header information. Header
 * information contains metadata about the packet message data block.*/
typedef struct __attribute__((packed)) rdm_header_t {
  uint8_t : 8;  // RDM start code.
  uint8_t : 8;  // RDM sub-start code.
  /** @brief The message length value is defined as the number of slots in the
     RDM packet including the start code and excluding the Checksum. Each slot
     is an 8-bit value.*/
  uint8_t message_len;
  /** @brief The UID of the target device(s).*/
  rdm_uid_t dest_uid;
  /** @brief The UID of the device originating this packet.*/
  rdm_uid_t src_uid;
  /** @brief The RDM transaction number. Controllers increment this field every
     time an RDM packet is transmitted. Responders set their transaction number
     to the transaction number of the packet to which they are responding.*/
  uint8_t tn;
  union {
    /** @brief The port ID field shall be set in the range 1-255 identifying the
       controller port being used, such that the combination of source UID and
       port ID will uniquely identify the controller and port where the message
       originated.*/
    uint8_t port_id;
    /** @brief The response type field is used in messages from responders to
       indicate the acknowledgement type of the response.*/
    uint8_t response_type;
  };
  /** @brief The message count field is used by a responder to indicate that
     additional data is now available for collection by a controller. The
     message count shall be set to 0 in all controller generated requests. The
     message count shall be incremented by a responder whenever there is a new
     message pending collection by a controller. Thus a controller can
     determine, from any response, the number of queued messages pending.*/
  uint8_t message_count;
  /** @brief Sub-devices should be used in devices containing a repetitive
     number of similar modules, such as a dimmer rack.*/
  uint16_t sub_device;
  /** @brief The command class (CC) specifies the action of the message.*/
  uint8_t cc;
  /** @brief The parameter ID (PID) identifies a specific type of parameter
     data.*/
  uint16_t pid;
  /** @brief The parameter data length (PDL) is the number of slots included in
     the parameter data area that it precedes. When this field is set to 0x00 it
     indicates that there is no parameter data following.*/
  uint8_t pdl;
} rdm_header_t;

/** @brief Parameters for use in RDM discovery requests. Discovery requests are
 * broadcast messages with a lower bound and upper bound. If a responding
 * device's UID falls within the lower bound and upper bound, it will respond to
 * the discovery request. */
typedef struct __attribute__((packed)) rdm_disc_unique_branch_t {
  /** @brief The lower bound of the RDM discovery request.*/
  rdm_uid_t lower_bound;
  /** @brief The upper bound of the RDM discovery request.*/
  rdm_uid_t upper_bound;
} rdm_disc_unique_branch_t;

/** @brief Parameters for use with RDM discovery mute and un-mute requests. When
 * a responder device is successfully muted or un-muted, it responds with these
 * parameters.*/
typedef struct __attribute__((packed)) rdm_disc_mute_t {
  union {
    struct {
      /** @brief The managed proxy flag shall be set to 1 when the responder is
         a proxy device.*/
      bool managed_proxy : 1;
      /** @brief The sub-device flag shall be set to 1 when the responder
         supports sub-devices.*/
      bool sub_device : 1;
      /** @brief The boot-loader flag shall only be set to 1 when the device is
         incapable of normal operation until receiving a firmware upload.*/
      bool boot_loader : 1;
      /** @brief The proxied device flag shall only be set to 1 when a proxy is
         responding to discovery on behalf of another device. This flag
         indicates that the response has come from a proxy rather than the
         actual device.*/
      bool proxied_device : 1;
    };
    /** @brief The control field contains bit flags. Bit 0 is the managed proxy
       flag, bit 1 is the sub-device flag, bit 2 is the boot-loader flag, bit 3
       is the proxied device flag. Bits 4 through 15 are reserved and shall be
       set to 0.*/
    uint16_t control_field;
  };
  /** @brief The binding UID field shall only be included when the responding
     device contains multiple responder ports. If the device does contain
     multiple ports then the binding UID field shall contain the UID for the
     primary port on the device. If the device does not contain multiple
     responder ports, this field is set to 0.*/
  rdm_uid_t binding_uid;
} rdm_disc_mute_t;

/** @brief Parameter for use with RDM device info requests.*/
typedef struct __attribute__((packed)) rdm_device_info_t {
  uint8_t : 8;  // RDM major version. Is always 1.
  uint8_t : 8;  // RDM minor version. Is always 0.
  /** @brief This field identifies the device model ID of the root device or
     sub-device. The manufacturer shall not use the same ID to represent more
     than one unique model type.*/
  uint16_t model_id;
  /** @brief Devices shall report a product category based on the product's
     primary function.*/
  uint16_t product_category;
  /** @brief This field indicates the software version ID for the device. The
     software version ID is a 32-bit value determined by the manufacturer.*/
  uint32_t software_version_id;
  /** @brief This field species the DMX footprint - the number of consecutive
     DMX slots required.*/
  uint16_t footprint;
  /** @brief The current selected DMX personality of the device. The personality
     is the configured arrangement of DMX slots used by the device. Many devices
     may have multiple personalities from which to choose. These personalities
     shall be consecutively numbered starting from 1.*/
  uint8_t current_personality;
  /** @brief The number of personalities supported by the device. The
     personality is the configured arrangement of DMX slots used by the device.
     Many devices may have multiple personalities from which to choose. These
     personalities shall be consecutively numbered starting from 1.*/
  uint8_t personality_count;
  /** @brief The DMX start address of the device. If the device or sub-device
     that the request is directed to has a DMX footprint of 0, then this field
     shall be set to 0xffff.*/
  uint16_t dmx_start_address;
  /** @brief This parameter is used to retrieve the number of sub-devices
     respresented by the root device. The response for this field shall always
     be the same regardless of whether this message is directed to the root
     device or a sub-device.*/
  uint16_t sub_device_count;
  /** @brief This field indicates the number of available sensors in a root
     device or sub-device. When this parameter is directed to a sub-device, the
     reply shall be identical for any sub-device owned by a specific root
     device.*/
  uint8_t sensor_count;
} rdm_device_info_t;

/** @brief The purpose of this parameter is to allow a controller to retrieve
 * enough information about the manufacturerspecific PID to generate GET and SET
 * commands.*/
typedef struct __attribute__((packed)) rdm_pid_description_t {
  /** @brief The manufacturer specific PID requested by the controller.*/
  uint16_t pid;
  /** @brief PDL Size defines the number used for the PDL field in all
     GET_RESPONSE and SET messages associated with this PID. This field is often
     the maximum possible size of the PDL not necessarily the absolute size of
     the PDL.*/
  uint8_t pdl_size;
  /** @brief Data type defines the size of the data entries in the PD of the
     message for this PID. For example: unsigned 8-bit character versus signed
     16-bit word.*/
  uint8_t data_type;
  /** @brief Command Class defines whether GET and or SET messages are
     implemented for the specified PID.*/
  uint8_t cc;
  uint8_t : 8;  // PID type. Should be set to 0.
  /** @brief Unit is an unsigned 8-bit value enumerated by rdm_units_t. It
     defines the SI unit of the specified PID data.*/
  uint8_t unit;
  /** @brief Prefix is an unsigned 8-bit value enumerated by rdm_prefix_t. It
     defines the SI Prefix and multiplication factor of the units. */
  uint8_t prefix;
  /** @brief This is a 32-bit field that represents the lowest value that data
     can reach. The format of the number is defined by DATA TYPE. This field has
     no meaning for a Data Type of RDM_DS_BIT_FIELD or RDM_DS_ASCII. For Data
     Types less than 32-bits, the Most Significant Bytes shall be padded with
     0x00 out to 32-bits. For example, an 8-bit data value of 0x12 shall be
     represented in the field as: 0x00000012.*/
  uint32_t min_value;
  /** @brief This is a 32-bit field that represents the highest value that data
     can reach. The format of the number is defined by DATA TYPE. This field has
     no meaning for a Data Type of RDM_DS_BIT_FIELD or RDM_DS_ASCII. For Data
     Types less than 32-bits, the Most Significant Bytes shall be padded with
     0x00 out to 32-bits. For example, an 8-bit data value of 0x12 shall be
     represented in the field as: 0x00000012.*/
  uint32_t max_value;
  /** @brief This is a 32-bit field that represents the default value of that
     data. This field has no meaning for a Data Type of RDM_DS_BIT_FIELD or
     RDM_DS_ASCII. The default value shall be within the minimum and maximum
     range. For Data Types less than 32-bits, the Most Significant Bytes shall
     be padded with 0x00 out to 32-bits. For example, an 8-bit data value of
     0x12 shall be represented in the field as: 0x00000012.*/
  uint32_t default_value;
  /** @brief The description field is used to describe the function of the
     specified PID. This text field shall be variable up to 32 characters in
     length.*/
  char description[33];
} rdm_pid_description_t;

/** @brief This parameter is used to set the responder's DMX personality. Many
 * devices such as moving lights have different DMX personalities. Many RDM
 * parameters may be affected by changing personality. */
typedef struct __attribute__((packed)) rdm_dmx_personality_t {
  /** @brief The current selected DMX personality of the device. The personality
     is the configured arrangement of DMX slots used by the device. Many devices
     may have multiple personalities from which to choose. These personalities
     shall be consecutively numbered starting from 1.*/
  uint8_t current_personality;
  /** @brief The number of personalities supported by the device. The
     personality is the configured arrangement of DMX slots used by the device.
     Many devices may have multiple personalities from which to choose. These
     personalities shall be consecutively numbered starting from 1.*/
  uint8_t personality_count;
} rdm_dmx_personality_t;

// TODO: docs
typedef struct __attribute__((packed)) rdm_status_message_t {
  uint16_t sub_device;
  uint8_t type;
  uint16_t id;
  union {
    struct {
      uint16_t data1;
      uint16_t data2;
    };
    uint16_t data[2];
  };
} rdm_status_message_t;

/**
 * @brief The function type for user callbacks in RDM responses.
 */
typedef void (*rdm_callback_t)(dmx_port_t dmx_num, const rdm_header_t *header,
                               void *context);

// TODO: docs
typedef struct rdm_pd_schema_t {
  rdm_ds_t data_type;
  rdm_pid_cc_t cc;
  size_t pdl_size;
  uint32_t min_value;
  uint32_t max_value;
  const char *format;
} rdm_pd_schema_t;

/**
 * @brief A function type for RDM responder callbacks. This is the type of
 * function that is called when responding to RDM requests.
 */
typedef int (*rdm_response_handler_t)(dmx_port_t dmx_num, rdm_header_t *header,
                                      void *pd, uint8_t *pdl_out,
                                      const rdm_pd_schema_t *schema);

// TODO: docs
typedef struct rdm_pd_definition_t {
  rdm_pd_schema_t schema;
  bool nvs;
  size_t alloc_size;
  rdm_response_handler_t response_handler;
  uint32_t default_value;
  rdm_units_t units;
  rdm_prefix_t prefix;
  const char *description;
} rdm_pd_definition_t;

/** @brief UID which indicates an RDM packet is being broadcast to all devices
 * regardless of manufacturer. Responders shall not respond to RDM broadcast
 * messages.*/
static const rdm_uid_t RDM_UID_BROADCAST_ALL = {0xffff, 0xffffffff};

/** @brief The maximum possible RDM UID.*/
static const rdm_uid_t RDM_UID_MAX = {0xffff, 0xfffffffe};

#ifdef __cplusplus
}
#endif
