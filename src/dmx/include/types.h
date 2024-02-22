/**
 * @file dmx/types.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file contains constants and types used in DMX and RDM. It also
 * contains macros for checking DMX and RDM timing constraints.
 */
#pragma once

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Macro used to convert milliseconds to FreeRTOS ticks. Evaluates to
 * the minimum number of ticks needed for the specified number of milliseconds
 * to elapse.*/
#define dmx_ms_to_ticks(ms)                                 \
  (pdMS_TO_TICKS(ms) +                                      \
   (((TickType_t)(ms) * (TickType_t)(configTICK_RATE_HZ)) % \
        (TickType_t)1000U >                                 \
    0))

/** @brief Evaluates to true if the start code is a start code permitted in a
 * non-prototype DMX device. Several alternate start codes are reserved for
 * special purposes or for future development of the standard. No equipment
 * shall be manufactured that generates alternate start codes 0x92-0xA9 or
 * 0xAB-0xCD until their use is defined by the standard or by the E1 Accredited
 * Standards Committee. Manufacturers shall not advertise or sell products or
 * devices that use alternate start codes 0xF0-0xF7.*/
#define dmx_start_code_is_valid(sc)                              \
  (!((sc >= 0x92 && sc <= 0xa9) || (sc >= 0xab && sc <= 0xcb) || \
     (sc == 0xcd) || (sc >= 0xf0 && sc <= 0xf7)))

/** @brief Evaluates to true if the start code is one of the three start codes
 * used in RDM. The start codes used in RDM are one of RDM_SC, RDM_PREAMBLE, or
 * RDM_DELIMITER.*/
#define dmx_start_code_is_rdm(sc) \
  (sc == RDM_SC || sc == RDM_PREAMBLE || sc == RDM_DELIMITER)

/** @brief Evaluates to true if the baud rate is within DMX specification.*/
#define dmx_baud_rate_is_valid(baud) \
  (baud >= DMX_BAUD_RATE_MIN && baud <= DMX_BAUD_RATE_MAX)

/** @brief Evaluates to true if the received break duration is within DMX
 * specification.*/
#define dmx_break_len_is_valid(brk) \
  (brk >= DMX_BREAK_LEN_MIN_US && brk <= DMX_BREAK_LEN_MAX_US)

/** @brief Evaluates to true if the received mark-after-break duration is within
 * DMX specification.*/
#define dmx_mab_len_is_valid(mab) \
  (mab >= DMX_MAB_LEN_MIN_US && mab <= DMX_MAB_LEN_MAX_US)

/** @brief Evaluates to true if the baud rate is within RDM specification.*/
#define rdm_baud_rate_is_valid(baud) \
  (baud >= DMX_BAUD_RATE_MIN && baud <= DMX_BAUD_RATE_MAX)

/** @brief Evaluates to true if the received break duration is within RDM
 * specification.*/
#define rdm_break_len_is_valid(brk) \
  (brk >= RDM_BREAK_LEN_MIN_US && brk <= RDM_BREAK_LEN_MAX_US)

/** @brief Evaluates to true if the received mark-after-break duration is within
 * RDM specification.*/
#define rdm_mab_len_is_valid(mab) \
  (mab >= RDM_MAB_LEN_MIN_US && mab <= RDM_MAB_LEN_MAX_US)

/** @brief DMX port constants.*/
enum {
  DMX_NUM_0, /** @brief DMX port 0.*/
  DMX_NUM_1, /** @brief DMX port 1.*/
#if SOC_UART_NUM > 2
  DMX_NUM_2, /** @brief DMX port 2.*/
#endif
  DMX_NUM_MAX /** @brief DMX port max. Used for error checking.*/
};

/** @brief DMX pin constants.*/
enum {
  /** @brief Constant for dmx_set_pin(). Indicates the pin should not be
     changed.*/
  DMX_PIN_NO_CHANGE = -1
};

/** @brief DMX requirements constants. These constants are simplified
 * significantly to ensure ease of use for the end user. When used with this
 * library, these constants will ensure that library settings are always within
 * DMX specification, but it is possible for other values to be used that also
 * fall within DMX specification.*/
enum {
  /** @brief The typical packet size of DMX.*/
  DMX_PACKET_SIZE = 513,
  /** @brief The maximum packet size of DMX.*/
  DMX_PACKET_SIZE_MAX = 513,

  /** @brief The typical baud rate of DMX.*/
  DMX_BAUD_RATE = 250000,
  /** @brief The minimum baud rate of DMX.*/
  DMX_BAUD_RATE_MIN = 245000,
  /** @brief The maximum baud rate of DMX.*/
  DMX_BAUD_RATE_MAX = 255000,

  /** @brief The typical break length of DMX in microseconds.*/
  DMX_BREAK_LEN_US = 176,
  /** @brief The minimum DMX break length in microseconds.*/
  DMX_BREAK_LEN_MIN_US = 92,
  /** @brief The maximum DMX break length in microseconds.*/
  DMX_BREAK_LEN_MAX_US = 1000000,

  /** @brief The typical mark-after-break length of DMX in microseconds.*/
  DMX_MAB_LEN_US = 12,
  /** @brief The minimum DMX mark-after-break length in microseconds.*/
  DMX_MAB_LEN_MIN_US = 12,
  /** @brief The maximum DMX mark-after-break length in microseconds.*/
  DMX_MAB_LEN_MAX_US = 999999,

  /** @brief The DMX receive timeout length in FreeRTOS ticks. If it takes
     longer than this amount of time to receive the next DMX packet the signal
     is considered lost.*/
  DMX_TIMEOUT_TICK = dmx_ms_to_ticks(1250),

  /** @brief The typical RDM break length in microseconds.*/
  RDM_BREAK_LEN_US = 176,
  /** @brief The minimum RDM break length in microseconds.*/
  RDM_BREAK_LEN_MIN_US = 176,
  /** @brief The maximum RDM break length in microseconds.*/
  RDM_BREAK_LEN_MAX_US = 352,

  /** @brief The typical RDM mark-after-break length in microseconds.*/
  RDM_MAB_LEN_US = 12,
  /** @brief The minimum RDM mark-after-break length in microseconds.*/
  RDM_MAB_LEN_MIN_US = 12,
  /** @brief The maximum RDM mark-after-break length in microseconds.*/
  RDM_MAB_LEN_MAX_US = 88,
};

/** @brief DMX start codes. These are the start codes used within the DMX
 * specification. This enum also includes RDM specific codes than can be used in
 * place of a start code or are use similarly to DMX start codes.*/
enum {
  /** @brief DMX default NULL start code. A NULL start code identifies
     subsequent data slots as a block of untyped sequential 8-bit information.
     Packets identified by a NULL start code are the default packets sent on DMX
     networks.*/
  DMX_SC = 0x00,

  /** @brief Remote Device Management (RDM) start code. RDM is an extension to
     DMX. A key goal of the RDM standard is to allow the use of new and legacy
     DMX receiving devices in mixed systems with new RDM equipment and to
     provide a straightforward path to upgrade existing DMX distribution systems
     for support of the RDM protocol. The use of RDM devices in a DMX system
     will not compromise any DMX functionality.*/
  RDM_SC = 0xcc,

  /** @brief The sub-start code within RDM shall be sent in the slot after the
     RDM start code (RDM_SC). Future versions of the ANSI-ESTA E1.20 standard
     which may have additional or different packet structures would use the
     sub-start code code to identify the packet structure being used.
     Controllers shall always send RDM_SUB_SC, and responders shall ignore any
     packets containing other values.*/
  RDM_SUB_SC = 0x01,

  /** @brief When an RDM device responds to a DISC_UNIQUE_BRANCH command, it
     responds with a special response packet. Seven extra slots of preamble have
     been added to the start of the response packet to allow for inline devices
     that must shorten the packet for turning around transceivers. If the
     in-line device shortens the response packet, it shall shorten by exactly
     one slot time. The controller shall be able to process response packets
     with 0-7 bytes of preamble.*/
  RDM_PREAMBLE = 0xfe,

  /** @brief When an RDM device responds to a DISC_UNIQUE_BRANCH command, it
    responds with a special response packet. After sending up to seven extra
    slots of preamble in the response packet, a single byte delimiter is sent
    to indicate the start of the packet data.*/
  RDM_DELIMITER = 0xaa,

  /** @brief ASCII Text alternate start code. Alternate start code 0x17
     designates a special packet of between 3 and 512 data slots. The purpose of
     the ASCII text packet is to allow equipment to send diagnostic information
     coded per the American Standard Code for Information Interchange and
     formatted for display. Slot allocation is as follows:
     - Slot 1: Page number of one of the possible 256 text pages.
     - Slot 2: Characters per line. Indicates the number of characters per line
       that the transmitting device has used for the purposes of formatting
       the text. A slot value of zero indicates ignore this field.
     - Slots 3-512: Consecutive display characters in ASCII format. All
       characters are allowed and where a DMX512 text viewer is capable, it
       shall display the data using the ISO/IEC 646 standard character set. A
       slot value of zero shall terminate the ASCII string. Slots transmitted
       after this null terminator up to the reset sequence shall be ignored.*/
  DMX_TEXT_SC = 0x17,

  /** @brief Test Packet alternate start code. Alternate start code 0x55
     designates a special test packet of 512 data slots, where all data slots
     carry the value 0x55. Test packets shall be sent so that the time from the
     start of the break until the stop bit of the 513th slot shall be no more
     than 25 milliseconds. When test packets are sent back to back, the
     mark-before-break time shall be no more than 88 microseconds. The break
     timing for test packets shall be greater than or equal to 88 microseconds,
     and less than or equal to 120 microseconds. The mark-after-break time shall
     be greater than or equal to 8 microseconds and less than or equal to 16
     microseconds.*/
  DMX_TEST_SC = 0x55,

  /** @brief UTF-8 Text Packet alternate start code. Alternate start code 0x90
     designates a special packet of between 3 and 512 data slots. The purpose of
     the UTF-8 Text Packet is to allow equipment to send diagnostic information
     coded per UTF-8 as described in Unicode 5.0 published by The Unicode
     Consortium and formatted for display. UTF-8 should only be used when the
     text packet cannot be expressed in ASCII using the DMX_TEXT_ASC start code.
     Slot allocation is as follows:
     - Slot 1: Page number of one of the possible 256 text pages.
     - Slot 2: Characters per Line. Indicates the number of characters per line
       that the transmitting device has used for the purposes of formatting
       the text. A slot value of zero indicates "Ignore this field."
     - Slots 3-512:
       Consecutive display characters in UTF-8 format. All characters are
       allowed and where a DMX512 text viewer is capable, it shall display the
       data using the Unicode 5.0 character set. A slot value of zero shall
       terminate the UTF-8 text string. Slots transmitted after this null
       terminator up to the reset sequence shall be ignored.*/
  DMX_UTF8_SC = 0x90,

  /** @brief Manufacturer/Organization ID alternate start code. Alternate start
     code 0x91 followed by a 2 byte manufacturer ID field is reserved for
     Manufacturer/Organization specific use, transmitted byte order is MSB, LSB.
     The next byte after the manufacturers ID would normally be a manufacturer’s
     sub-code.*/
  DMX_ORG_ID_SC = 0x91,

  /** @brief System Information Packet alternate start code. Alternate start
     code 0xCF is reserved for a System Information Packet (SIP). The SIP
     includes a method of sending checksum data relating to the previous NULL
     start code packet on the data link and other control information. No other
     packet shall be sent between the NULL start code packet and the SIP that
     carries its checksum. For more information on the System Information Packet
     alternate start code, see annex D5 in the ANSI-ESTA E1.11 DMX512-A
     standards document.*/
  DMX_SIP_SC = 0xcf
};

/** @brief DMX port type.*/
typedef unsigned int dmx_port_t;

/** @brief DMX device number type.*/
typedef uint16_t dmx_device_num_t;

/** @brief Type which indicates errors, or lack thereof, for DMX operations.*/
typedef enum dmx_err_t {
  /** @brief DMX error value indicating no error.*/
  DMX_OK = 0,
  /** @brief The DMX operation timed out.*/
  DMX_ERR_TIMEOUT,
  /** @brief The UART overflowed while reading DMX data.*/
  DMX_ERR_UART_OVERFLOW,
  /** @brief The UART received an improperly framed DMX slot.*/
  DMX_ERR_IMPROPER_SLOT,
  /** @brief The DMX packet size is smaller than expected.*/
  DMX_ERR_NOT_ENOUGH_SLOTS,
  /** @brief Generic DMX error code indicating failure.*/
  DMX_FAIL = -1
} dmx_err_t;

/** @brief Configuration settings for the DMX driver.*/
typedef struct dmx_config_t {
  /** @brief The interrupt allocation flags to use.*/
  int interrupt_flags;
  /** @brief The number of parameters that the root device supports.*/
  uint32_t root_device_parameter_count;
  /** @brief The number of parameters that the sub-devices support.*/
  uint32_t sub_device_parameter_count;
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
  /** @brief This RDM parameter is used to get a descriptive ASCII text label
   for the device's operating software version. The descriptive text returned
   by this parameter is intended for display to the user.*/
  const char *software_version_label;
  /** @brief The maximum size of the RDM queue. Setting this value to 0 disables
   * the RDM queue.*/
  uint32_t queue_size_max;
} dmx_config_t;

/** @brief A struct which defines DMX personalities. Used to declare the
 * personalities of the DMX device upon calling dmx_driver_install(). */
typedef struct __attribute__((packed)) dmx_personality_t {
  uint8_t : 8;  // This bitfield allows this type to alias rdm_personality_t
  /** @brief The footprint of the DMX personality.*/
  uint16_t footprint;
  /** @brief A description of the DMX personality.*/
  char description[33];
} dmx_personality_t;

/** @brief Provides a summary of the received DMX packet so that users may
 * quickly and easily process and respond to DMX data.*/
typedef struct dmx_packet_t {
  /** @brief Evaluates to true if an error occurred reading DMX data.*/
  dmx_err_t err;
  /** @brief Start code of the DMX packet.*/
  int sc;
  /** @brief The size of the received DMX packet in bytes.*/
  size_t size;
  /** @brief True if the received packet is RDM.*/
  bool is_rdm;
} dmx_packet_t;

/** @brief Metadata for received DMX packets. For use in the DMX sniffer.*/
typedef struct dmx_metadata_t {
  /** @brief Length in microseconds of the last received DMX break.*/
  uint32_t break_len;
  /** @brief Length in microseconds of the last received DMX mark-after-break.*/
  uint32_t mab_len;
} dmx_metadata_t;

/** @brief DMX start address which indicates the device does not have a DMX
 * start address.*/
static const uint16_t DMX_START_ADDRESS_NONE = 0xffff;

#ifdef __cplusplus
}
#endif
