/**
 * @file rdm/types.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file contains constants and types used in RDM.
 */
#pragma once

#include <stdint.h>

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

/** @brief Evaluates to true if the rdm_cc_t is a valid value.*/
#define rdm_cc_is_valid(cc) (((cc) >> 8) < 0x4 && ((cc) & 0xf) < 0x2)

/** @brief Evaluates to true if the rdm_cc_t is either RDM_CC_GET_COMMAND,
 * RDM_CC_SET_COMMAND, or RDM_CC_DISC_COMMAND.*/
#define rdm_cc_is_request(cc) (((cc) & 0x1) == 0)

/** @brief Evaluates to true if the response type is a valid value.*/
#define rdm_response_type_is_valid(t) \
  ((t) >= RDM_RESPONSE_TYPE_ACK && (t) <= RDM_RESPONSE_TYPE_ACK_OVERFLOW)

/** @brief The typical, maximum size for RDM ASCII parameters.*/
#define RDM_ASCII_SIZE_MAX (33)

/** @brief The maximum size for RDM parameter data.*/
#define RDM_PD_SIZE_MAX (232)

/** @brief The maximum RDM sensor number.*/
#define RDM_SENSOR_NUM_MAX (0xff)

/** @brief The parameter ID (PID) is a 16-bit number that identifies a specific
 * type of parameter data. The PID may represent either a well known parameter
 * such as those defined in the RDM standard document, or a
 * manufacturer-specific parameter whose details are either published by the
 * manufacturer for third-party support or proprietary for the manufacturer's
 * own use.*/
typedef uint16_t rdm_pid_t;

/** @brief RDM sub-device number type.*/
typedef dmx_device_num_t rdm_sub_device_t;

/** @brief The RDM command class (CC) type. The command class specifies the
 * action of the message. Responders shall always generate a response to
 * GET_COMMAND and SET_COMMAND messages except when the destination UID of the
 * message is a broadcast address. Responders shall not respond to commands sent
 * using broadcast addressing, in order to prevent collisions.*/
typedef uint8_t rdm_cc_t;

/** @brief The response type field is used in messages from responders to
 * indicate the acknowledgement type of the response.*/
typedef uint8_t rdm_response_type_t;

/** @brief The NACK reason defines the reason that the responder is unable to
 * comply with the request.*/
typedef uint16_t rdm_nr_t;

/** @brief Status collection messages include messages used to retrieve deferred
 * (queued) responses, device status and error information, and information
 * regarding the RDM parameters supported by the device. Status collection
 * messages are normally addressed to root devices. The status type is used to
 * identify the severity of the condition.*/
typedef uint8_t rdm_status_t;

/** @brief RDM sub-device number type.*/
enum {
  /** @brief Sub-device which respresents the root of a RDM device.*/
  RDM_SUB_DEVICE_ROOT = 0,
  /** @brief The sub-device maximum. All sub-devices numbers must be less than
     this value unless using RDM_SUB_DEVICE ALL.*/
  RDM_SUB_DEVICE_MAX = 513,
  /** @brief The RDM sub-device number which can be used to address all
     sub-devices of an RDM device in a request.*/
  RDM_SUB_DEVICE_ALL = 0xffff
};

/** @brief The RDM command class (CC) type. The command class specifies the
 * action of the message. Responders shall always generate a response to
 * GET_COMMAND and SET_COMMAND messages except when the destination UID of the
 * message is a broadcast address. Responders shall not respond to commands sent
 * using broadcast addressing, in order to prevent collisions.*/
enum {
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
};

/** @brief The parameter ID (PID) is a 16-bit number that identifies a specific
 * type of parameter data. The PID may represent either a well known parameter
 * such as those defined in the RDM standard document, or a
 * manufacturer-specific parameter whose details are either published by the
 * manufacturer for third-party support or proprietary for the manufacturer's
 * own use.*/
enum {
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
     single message response. @note Supports GET.*/
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
  /** @brief This parameter provides a text description of up to 32 character
     for the device model type. @note Supports GET.*/
  RDM_PID_DEVICE_MODEL_DESCRIPTION = 0x0080,
  /** @brief This parameter provides an ASCII test response with the
    manufacturer name for the device of up to 32 characters. The manufacturer
    name must be consistent between all products manufactured within an ESTA
    manufacturer ID. Therefore this value is only editable in the Kconfig. @note
    Supports GET.*/
  RDM_PID_MANUFACTURER_LABEL = 0x0081,
  /** @brief This parameter providdes a means of setting a descriptive label for
     each device. This may be used for identifying a dimmer rack number or
     specifying the device's location. @note Supports GET and SET.*/
  RDM_PID_DEVICE_LABEL = 0x0082,
  RDM_PID_FACTORY_DEFAULTS = 0x0090,
  RDM_PID_LANGUAGE_CAPABILITIES = 0x00a0,
  /** @brief This parameter is used to change the language of the messages from
     the device. Supported languages of the device can be determined by
     RDM_PID_LANGUAGE_CAPABILITIES. The language codes are two character alpha
     codes as defined by ISO 639-1. International standard ISO 639-1, code for
     the representation of names of languages - part 1: Alpha 2 code. @note
     Supports GET and SET.*/
  RDM_PID_LANGUAGE = 0x00b0,
  /** @brief This parameter is used to get a descriptive ASCII text label for
     the device's operating software version. The descriptive text returned by
     this parameter is intended for display to the user. @note Supports GET.
     This parameter is required.*/
  RDM_PID_SOFTWARE_VERSION_LABEL = 0x00c0,
  RDM_PID_BOOT_SOFTWARE_VERSION_ID = 0x00c1,
  RDM_PID_BOOT_SOFTWARE_VERSION_LABEL = 0x00c2,

  // Category: DMX512 Setup
  /** @brief This parameter is used to set the responder's DMX personality. Many
     devices such as moving lights have different DMX personalities. Many RDM
     parameters may be affected by changing personality. The DMX personality can
     also be retrieved as part of the RDM_PID_DEVICE info parameter message.
     @note Supports GET and SET.*/
  RDM_PID_DMX_PERSONALITY = 0x00e0,
  /** @brief This parameter is used to get a descriptive ASCII text label for a
     given DMX personality. The label may be up to 32 characters. @note Supports
     GET.*/
  RDM_PID_DMX_PERSONALITY_DESCRIPTION = 0x00e1,
  /** @brief This parameter is used to set or get the DMX512 start address.
     @note Supports GET and SET. This parameter is required if the device uses a
     DMX slot.*/
  RDM_PID_DMX_START_ADDRESS = 0x00f0,
  RDM_PID_SLOT_INFO = 0x0120,
  RDM_PID_SLOT_DESCRIPTION = 0x0121,
  RDM_PID_DEFAULT_SLOT_VALUE = 0x0122,

  // Category: Sensors (0x02xx)

  /** @brief This parameter is used to retrieve the definition of a specific
    sensor. When this parameter is directed to a sub-device, the reply shall be
    identical for any given sensor number in all sub-devices owned by a specific
    root device. @note Supports GET.*/
  RDM_PID_SENSOR_DEFINITION = 0x0200,
  /** @brief This parameter shall be used to retrieve or reset sensor data.
    @note Supports GET and SET.*/
  RDM_PID_SENSOR_VALUE = 0x0201,
  /** @brief This parameter instructs devices such as dimming racks that monitor
    load changes to store the current value for monitoring sensor changes. @note
    Supports SET.*/
  RDM_PID_RECORD_SENSORS = 0x0202,

  // Category: Dimmer Settings (0x03xx)
  // Not yet defined by ANSI/ESTA e1.20

  // Category: Power/Lamp Settings (0x04xx)
  /** @brief This parameter is used to retrieve or set the number of hours of
     operation the device has been in use. Some devices may only support GET for
     this parameter and not allow the device's hours to be set. The value for
     the device hours field shall be unsigned and not roll over when the maximum
     value is reached. @note Supports GET and SET.*/
  RDM_PID_DEVICE_HOURS = 0x0400,
  /** @brief This parameter is used to retrieve the number of lamp hours or to
     set the counter in the device to a specific starting value. The lamp hours
     are the total number of hours that the lamp has been on. The value for this
     field shall be unsigned and shall not roll over when the maximum value is
     reached. @note Supports GET and SET.*/
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
  /** @brief This parameter is used to instruct the responder to reset itself.
     This parameter shall also clear the discovery mute flag. A cold reset is
     theequivalent of removing and reapplying power to the device. @note
     Supports SET.
  */
  RDM_PID_RESET_DEVICE = 0x1001,
  RDM_PID_POWER_STATE = 0x1010,        // TODO: See rdm_power_state_t
  RDM_PID_PERFORM_SELF_TEST = 0x1020,  // TODO: See rdm_self_test_t
  RDM_PID_SELF_TEST_DESCRIPTION = 0x1021,
  RDM_PID_CAPTURE_PRESET = 0x1030,
  RDM_PID_PRESET_PLAYBACK = 0x1031,  // TODO: See rdm_preset_playback_t

  // Reserved for Future RDM Development: 0x7fe0-0x7fff

  // Manufacturer Specific PIDs
  RDM_PID_MANUFACTURER_SPECIFIC_BEGIN = 0x8000,
  RDM_PID_MANUFACTURER_SPECIFIC_END = 0xffdf,

  // Reserved for Future RDM Development: 0xffe0-0xffff
};

/** @brief The response type field is used in messages from responders to
 * indicate the acknowledgement type of the response.*/
enum {
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
};

/** @brief The NACK reason defines the reason that the responder is unable to
 * comply with the request.*/
enum {
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
};

/** @brief Status collection messages include messages used to retrieve deferred
 * (queued) responses, device status and error information, and information
 * regarding the RDM parameters supported by the device. Status collection
 * messages are normally addressed to root devices. The status type is used to
 * identify the severity of the condition.*/
enum {
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
};

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
  /** @brief RDM unit for temperature.*/
  RDM_UNITS_CENTIGRADE = 0x01,
  /** @brief RDM unit for DC voltage.*/
  RDM_UNITS_VOLTS_DC = 0x02,
  /** @brief RDM unit for peak AC voltage.*/
  RDM_UNITS_VOLTS_AC_PEAK = 0x03,
  /** @brief RDM unit for RMS AC voltage.*/
  RDM_UNITS_VOLTS_AC_RMS = 0x04,
  /** @brief RDM unit for DC amperage.*/
  RDM_UNITS_AMPERE_DC = 0x05,
  /** @brief RDM unit for peak AC amperage.*/
  RDM_UNITS_AMPERE_AC_PEAK = 0x06,
  /** @brief RDM unit for RMS AC amperage.*/
  RDM_UNITS_AMPERE_AC_RMS = 0x07,
  /** @brief RDM unit for frequency.*/
  RDM_UNITS_HERTZ = 0x08,
  /** @brief RDM unit for electrical resistance.*/
  RDM_UNITS_OHM = 0x09,
  /** @brief RDM unit for electrical power.*/
  RDM_UNITS_WATT = 0x0a,
  /** @brief RDM unit for mass.*/
  RDM_UNITS_KILOGRAM = 0x0b,
  /** @brief RDM unit for length or position.*/
  RDM_UNITS_METERS = 0x0c,
  /** @brief RDM unit for area.*/
  RDM_UNITS_METERS_SQUARED = 0x0d,
  /** @brief RDM unit for volume.*/
  RDM_UNITS_METERS_CUBED = 0x0e,
  /** @brief RDM unit for density.*/
  RDM_UNITS_KILOGRAMMES_PER_METER_CUBED = 0x0f,
  /** @brief RDM unit for velocity.*/
  RDM_UNITS_METERS_PER_SECOND = 0x10,
  /** @brief RDM unit for acceleration.*/
  RDM_UNITS_METERS_PER_SECOND_SQUARED = 0x11,
  /** @brief RDM unit for force.*/
  RDM_UNITS_NEWTON = 0x12,
  /** @brief RDM unit for energy.*/
  RDM_UNITS_JOULE = 0x13,
  /** @brief RDM unit for pressure.*/
  RDM_UNITS_PASCAL = 0x14,
  /** @brief RDM unit for time.*/
  RDM_UNITS_SECOND = 0x15,
  /** @brief RDM unit for angle.*/
  RDM_UNITS_DEGREE = 0x16,
  /** @brief RDM unit for angle.*/
  RDM_UNITS_STERADIAN = 0x17,
  /** @brief RDM unit for luminous intensity.*/
  RDM_UNITS_CANDELA = 0x18,
  /** @brief RDM unit for luminous flux.*/
  RDM_UNITS_LUMEN = 0x19,
  /** @brief RDM unit for illuminance.*/
  RDM_UNITS_LUX = 0x1a,
  /** @brief RDM unit for chrominance.*/
  RDM_UNITS_IRE = 0x1b,
  /** @brief RDM unit for memory. When a prefix is used with this unit, the
     multiplier refers to binary multiple. e.g. KILO means multiply by 1024.*/
  RDM_UNITS_BYTE = 0x1c
} rdm_units_t;

/** @brief The prefix defines the SI prefix and multiplication factor of the
 * units*/
typedef enum rdm_prefix_t {
  /** @brief Multiply by 1.*/
  RDM_PREFIX_NONE = 0x00,
  /** @brief Multiply by 10E-24.*/
  RDM_PREFIX_YOCTO = 0x0a,
  /** @brief Multiply by 10E+1.*/
  RDM_PREFIX_DECA = 0x11,
  /** @brief Multiply by 10E+24.*/
  RDM_PREFIX_YOTTA = 0x1a
} rdm_prefix_t;

/** @brief Sensor type enums used in rdm_sensor_definition_t. Defines what the
 * sensor measures, but not the units of the measurement.
 */
typedef enum rdm_sensor_type_t {
  /** @brief The sensor measures temperature.*/
  RDM_SENSOR_TYPE_TEMPERATURE = 0x00,
  /** @brief The sensor measures electric voltage.*/
  RDM_SENSOR_TYPE_VOLTAGE = 0x01,
  /** @brief The sensor measures electric current.*/
  RDM_SENSOR_TYPE_CURRENT = 0x02,
  /** @brief The sensor measures frequency.*/
  RDM_SENSOR_TYPE_FREQUENCY = 0x03,
  /** @brief The sensor measures electric resistance, e.g. cable resistance.*/
  RDM_SENSOR_TYPE_RESISTANCE = 0x04,
  /** @brief The sensor measures power.*/
  RDM_SENSOR_TYPE_POWER = 0x05,
  /** @brief The sensor measures mass.*/
  RDM_SENSOR_TYPE_MASS = 0x06,
  /** @brief The sensor measures length.*/
  RDM_SENSOR_TYPE_LENGTH = 0x07,
  /** @brief The sensor measures area.*/
  RDM_SENSOR_TYPE_AREA = 0x08,
  /** @brief The sensor measures volume, e.g. the volume of smoke fluid.*/
  RDM_SENSOR_TYPE_VOLUME = 0x09,
  /** @brief The sensor measures density.*/
  RDM_SENSOR_TYPE_DENSITY = 0x0a,
  /** @brief The sensor measures velocity.*/
  RDM_SENSOR_TYPE_VELOCITY = 0x0b,
  /** @brief The sensor measures acceleration.*/
  RDM_SENSOR_TYPE_ACCELERATION = 0x0c,
  /** @brief The sensor measures force.*/
  RDM_SENSOR_TYPE_FORCE = 0x0d,
  /** @brief The sensor measures energy.*/
  RDM_SENSOR_TYPE_ENERGY = 0x0e,
  /** @brief The sensor measures pressure.*/
  RDM_SENSOR_TYPE_PRESSURE = 0x0f,
  /** @brief The sensor measures time.*/
  RDM_SENSOR_TYPE_TIME = 0x10,
  /** @brief The sensor measures angle.*/
  RDM_SENSOR_TYPE_ANGLE = 0x11,
  /** @brief The sensor measures X position, e.g. lamp position on truss.*/
  RDM_SENSOR_TYPE_POSITION_X = 0x12,
  /** @brief The sensor measures Y position.*/
  RDM_SENSOR_TYPE_POSITION_Y = 0x13,
  /** @brief The sensor measures Z position.*/
  RDM_SENSOR_TYPE_POSITION_Z = 0x14,
  /** @brief The sensor measures angular velocity, e.g. wind speed.*/
  RDM_SENSOR_TYPE_ANGULAR_VELOCITY = 0x15,
  /** @brief The sensor measures luminous intensity.*/
  RDM_SENSOR_TYPE_LUMINOUS_INTENSITY = 0x16,
  /** @brief The sensor measures luminous flux.*/
  RDM_SENSOR_TYPE_LUMINOUS_FLUX = 0x17,
  /** @brief The sensor measures illuminance.*/
  RDM_SENSOR_TYPE_ILLUMINANCE = 0x18,
  /** @brief The sensor measures red chrominance.*/
  RDM_SENSOR_TYPE_CHROMINANCE_RED = 0x19,
  /** @brief The sensor measures green chrominance.*/
  RDM_SENSOR_TYPE_CHROMINANCE_GREEN = 0x1a,
  /** @brief The sensor measures blue chrominance.*/
  RDM_SENSOR_TYPE_CHROMINANCE_BLUE = 0x1b,
  /** @brief The sensor measures contacts, e.g. switch inputs.*/
  RDM_SENSOR_TYPE_CONTACTS = 0x1c,
  /** @brief The sensor measures memory, e.g. ROM size.*/
  RDM_SENSOR_TYPE_MEMORY = 0x1d,
  /** @brief The sensor measures items, e.g. scroller gel frames.*/
  RDM_SENSOR_TYPE_ITEMS = 0x1e,
  /** @brief The sensor measures humidity.*/
  RDM_SENSOR_TYPE_HUMIDITY = 0x1f,
  /** @brief The sensor is a 16-bit counter.*/
  RDM_SENSOR_TYPE_COUNTER_16BIT = 0x20,
  /** @brief The sensor measures some other value, but does not have a
     manufacturer-specific definition.*/
  RDM_SENSOR_TYPE_OTHER = 0x21,
} rdm_sensor_type_t;

enum {
  /** @brief Constant for RDM sensor definition when the minimum value of a
     sensor is undefined.*/
  RDM_SENSOR_MINIMUM_UNDEFINED = -32768,
  /** @brief Constant for RDM sensor definition when the maximum value of a
     sensor is undefined.*/
  RDM_SENSOR_MAXIMUM_UNDEFINED = 32767,
};

enum {
  /** @brief The reset type for use with RDM_PID_RESET_DEVICE. This indicates no
     reset of the device is requested. This constant is provided for convenience
     and readability. It should not be used in RDM requests.*/
  RDM_RESET_TYPE_NONE = 0x00,
  /** @brief The reset type for use with RDM_PID_RESET_DEVICE. This indicates a
     soft reset of the device is requested.*/
  RDM_RESET_TYPE_WARM = 0x01,
  /** @brief The reset type for use with RDM_PID_RESET_DEVICE. This indicates a
     hard reset of the device is requested. This is the equivalent of removing
     and reapplying power to the device.*/
  RDM_RESET_TYPE_COLD = 0xff,
};

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
      uint8_t managed_proxy : 1;
      /** @brief The sub-device flag shall be set to 1 when the responder
         supports sub-devices.*/
      uint8_t sub_device : 1;
      /** @brief The boot-loader flag shall only be set to 1 when the device is
         incapable of normal operation until receiving a firmware upload.*/
      uint8_t boot_loader : 1;
      /** @brief The proxied device flag shall only be set to 1 when a proxy is
         responding to discovery on behalf of another device. This flag
         indicates that the response has come from a proxy rather than the
         actual device.*/
      uint8_t proxied_device : 1;
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

/** @brief This parameter is used to set the responder's DMX personality. Many
 * devices such as moving lights have different DMX personalities. Many RDM
 * parameters may be affected by changing personality. */
typedef struct __attribute__((packed)) rdm_dmx_personality_t {
  /** @brief The current selected DMX personality of the device. The personality
     is the configured arrangement of DMX slots used by the device. Many devices
     may have multiple personalities from which to choose. These personalities
     shall be consecutively numbered starting from 1.*/
  uint8_t current;
  /** @brief The number of personalities supported by the device. The
     personality is the configured arrangement of DMX slots used by the device.
     Many devices may have multiple personalities from which to choose. These
     personalities shall be consecutively numbered starting from 1.*/
  uint8_t count;
} rdm_dmx_personality_t;

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
  /** @brief This parameter is used to set the responder's DMX personality. Many
   * devices such as moving lights have different DMX personalities. Many RDM
   * parameters may be affected by changing personality. */
  rdm_dmx_personality_t personality;
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
typedef struct __attribute__((packed)) rdm_parameter_description_t {
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
  char description[RDM_ASCII_SIZE_MAX];
} rdm_parameter_description_t;

/** @brief Used to get a descriptive ASCII text label for a given DMX
 * personality. The label may be up to 32 characters.*/
typedef struct __attribute__((packed)) rdm_dmx_personality_description_t {
  uint8_t personality_num;               // The personality number requested.
  uint16_t footprint;                    // The footprint of the personality.
  char description[RDM_ASCII_SIZE_MAX];  // The description of the personality.
} rdm_dmx_personality_description_t;

/**
 * @brief The RDM status message struct returned from a GET request to
 * RDM_PID_STATUS_MESSAGES. Used to report any informational, warning, or error
 * messages which are reported by the device.
 */
typedef struct __attribute__((packed)) rdm_status_message_t {
  /** @brief In a system containing sub-devices, this field shall be used to
     indicate the sub-device to which the status message belongs. If the status
     message does not reference a particular sub-device, the field shall be set
     to 0x0000 to reference the root device.*/
  uint16_t sub_device;
  /** @brief The type field is used to identify the severity of the condition.
     The message shall be reported with a status type enumerated in
     rdm_status_t.*/
  uint8_t type;
  /** @brief Status message IDs witthin the range of 0x000 and 0x7fff
     (inclusive) are reserved for publicly defined status messages. These are
     enumerated in rdm_status_id_t. Manufacturer specific messages are in the
     range 0x8000 and 0xffdf (inclusive). Each manufacturer specific status ID
     shall have a unique meaning which shall be consistent across all products
     having a given manufacturer ID.*/
  uint16_t id;
  union {
    struct {
      /** @brief Each status message supports the return of two separate data
         values relevant to the context of the specific message. The data value
         for ESTA public status messages is used to identify a property within
         the device to which the message corresponds. Each data value shall be a
         signed integer.*/
      int16_t data1;
      /** @brief Each status message supports the return of two separate data
         values relevant to the context of the specific message. The data value
         for ESTA public status messages is used to identify a property within
         the device to which the message corresponds. Each data value shall be a
         signed integer.*/
      int16_t data2;
    };
    /** @brief Each status message supports the return of two separate data
       values relevant to the context of the specific message. The data value
       for ESTA public status messages is used to identify a property within the
       device to which the message corresponds. Each data value shall be a
       signed integer.*/
    int16_t data[2];
  };
} rdm_status_message_t;

/** @brief This struct contains information about a specific sensor.*/
typedef struct __attribute__((packed)) rdm_sensor_definition_t {
  /** @brief The sensor number requested is in the range from 0x00 to 0xfe
     (inclusive).*/
  uint8_t num;
  /** @brief Type is an unsigned 8-bit value enumerated in rdm_sensor_type_t. It
     defines the type of data that is measured by the sensor.*/
  uint8_t type;
  /** @brief Unit is an unsigned 8-bit value enumerated in rdm_unit_t*/
  uint8_t unit;
  /** @brief Prefix is an unsigned 8-bit value enumerated in rdm_prefix_t. It
     defines the SI unit of the sensor data.*/
  uint8_t prefix;
  struct {
    /** @brief This is a 2's compliment signed 16-bit value that represents the
       lowest value the sensor can report. A value of -32768 indicates that the
       minimum is not defined.*/
    int16_t minimum;
    /** @brief This is a 2's compliment signed 16-bit value that represents the
       highest value the sensor can report. This also defines the maximum
       capacity. A value of 32767 indicates that the maximum is not defined.*/
    int16_t maximum;
  } range;
  struct {
    /** @brief This is a 2's compliment signed 16-bit value that represents the
       lowest value for which the sensor is in normal operation. A value of
       -32768 indicates that the minimum is not defined.*/
    int16_t minimum;
    /** @brief This is a 2's compliment signed 16-bit value that represents the
       highest value for which the sensor is in normal operation. A value of
       32767 indicates that the maximum is not defined.*/
    int16_t maximum;
  } normal;
  /** @brief This bit shall be set to 1 if this sensor supports recording
     data. This feature is optional. */
  uint8_t recorded_value_support : 1;
  /** @brief This bit shall be set to 1 if this sensor supports the highest or
     lowest detected value. This feature is optional.*/
  uint8_t lowest_highest_detected_value_support : 1;
  uint8_t : 6;  // Reserved. Should be set to 0.
  /** @brief The description field is used to describe the function of the
     sensor. This text shall be variable up to 32 characters in length, not
     including the null terminator.*/
  char description[RDM_ASCII_SIZE_MAX];
} rdm_sensor_definition_t;

/** @brief Stores information pertaining to RDM sensors.*/
typedef struct __attribute__((packed)) rdm_sensor_value_t {
  /** @brief The sensor number is in the range 0x00 to 0xFE. A value 0xFF is
     used to represent all sensors for the SET command. The sensor value fields
     in the response to a SET command sent to sensor 0xFF shall be ignored by
     the controller. There is no requirement on a responder to provide specific
     values in this response.*/
  uint8_t sensor_num;
  /** @brief This is a signed 16-bit value that represents the present value of
     the sensor data.*/
  int16_t present_value;
  /** @brief This is a signed 16-bit value that represents the lowest value
     registered by the sensor. Support for this data is optional. If this value
     is not supported then this field shall be set to 0x0000.*/
  int16_t lowest_value;
  /** @brief This is a signed 16-bit value that represents the highest value
   registered by the sensor. Support for this data is optional. If this value
   is not supported then this field shall be set to 0x0000.*/
  int16_t highest_value;
  /** @brief This is a signed 16-bit value that represents the value that was
     recorded  when the last RDM_PID_RECORD_SENSORS command was issued. Support
     for this data is optional. If this value is not supported then this field
     shall be set to 0x0000.*/
  int16_t recorded_value;
} rdm_sensor_value_t;

/** @brief UID which indicates an RDM packet is being broadcast to all devices
 * regardless of manufacturer. Responders shall not respond to RDM broadcast
 * messages.*/
static const rdm_uid_t RDM_UID_BROADCAST_ALL = {0xffff, 0xffffffff};

/** @brief The maximum possible RDM UID.*/
static const rdm_uid_t RDM_UID_MAX = {0xffff, 0xfffffffe};

#ifdef __cplusplus
}
#endif
