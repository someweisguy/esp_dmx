/**
 * @file dmx_constants.h
 * @author Mitch Weisbrod
 * @brief This file contains constants used in DMX and RDM. It also contains
 * macros for checking DMX and RDM timing constraints.
 */
#pragma once

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DMX port max. Used for error checking.
 */
#define DMX_NUM_MAX SOC_UART_NUM

/**
 * @brief The default interrupt flags for the DMX driver.
 */
#define DMX_DEFAULT_INTR_FLAGS ESP_INTR_FLAG_IRAM

/**
 * @brief The default interrupt flags for the DMX sniffer.
 */
#define DMX_SNIFFER_DEFAULT_INTR_FLAGS \
  (ESP_INTR_FLAG_EDGE | DMX_DEFAULT_INTR_FLAGS)

/**
 * @brief UID which indicates an RDM packet is being broadcast. Responders shall
 * not respond to RDM broadcast messages.
 */
static const uint64_t RDM_BROADCAST_UID = 0xffffffffffff;

/**
 * @brief DMX port number constants.
 */
enum dmx_num {
  DMX_NUM_0,  // DMX port 0.
  DMX_NUM_1,  // DMX port 1.
#if SOC_UART_NUM > 2
  DMX_NUM_2,  // DMX port 2.
#endif
};

enum dmx_pin {
  /**
   * @brief Constant for dmx_set_pin(). Indicates the pin should not be changed.
   */
  DMX_PIN_NO_CHANGE = -1
};

/**
 * @brief DMX parameter constants. These constants are simplified significantly
 * to ensure ease of use for the end user. When used with this library, these 
 * constants will ensure that library settings are always within DMX 
 * specification, but it is possible for other values to be used that also fall
 * within DMX specification.
 */
enum dmx_parameter {
  DMX_PACKET_SIZE = 513,      // The typical packet size of DMX.
  DMX_MAX_PACKET_SIZE = 513,  // The maximum packet size of DMX.

  DMX_BAUD_RATE = 250000,      // The typical baud rate of DMX.
  DMX_MIN_BAUD_RATE = 245000,  // The minimum baud rate of DMX.
  DMX_MAX_BAUD_RATE = 255000,  // The maximum baud rate of DMX.

  DMX_BREAK_LEN_US = 176,          // The typical break length of DMX in microseconds.
  DMX_MIN_BREAK_LEN_US = 92,       // The minimum DMX break length in microseconds.
  DMX_MAX_BREAK_LEN_US = 1000000,  // The maximum DMX break length in microseconds.

  DMX_MAB_LEN_US = 12,          // The typical mark-after-break length of DMX in microseconds.
  DMX_MIN_MAB_LEN_US = 12,      // The minimum DMX mark-after-break length in microseconds.
  DMX_MAX_MAB_LEN_US = 999999,  // The maximum DMX mark-after-break length in microseconds.

  DMX_TIMEOUT_TICK = pdMS_TO_TICKS(1250),  // The DMX receive timeout length in FreeRTOS ticks. If it takes longer than this amount of time to receive the next DMX packet the signal is considered lost.

  RDM_BASE_PACKET_SIZE = 26,  // TODO: do we need this?

  RDM_BREAK_LEN_US = 176,      // The typical RDM break length in microseconds.
  RDM_MIN_BREAK_LEN_US = 176,  // The minimum RDM break length in microseconds.
  RDM_MAX_BREAK_LEN_US = 352,  // The maximum RDM break length in microseconds.

  RDM_MAB_LEN_US = 12,      // The typical RDM mark-after-break length in microseconds.
  RDM_MIN_MAB_LEN_US = 12,  // The minimum RDM mark-after-break length in microseconds.
  RDM_MAX_MAB_LEN_US = 88,  // The maximum RDM mark-after-break length in microseconds.
};

enum rdm_packet_timing {
  RDM_DISCOVERY_NO_RESPONSE_PACKET_SPACING = 5800,  // Spacing between the RDM controller sending a discovery packet and sending any other packet.
  RDM_REQUEST_NO_RESPONSE_PACKET_SPACING = 3000,    // Spacing between the RDM controller sending a request, considering the response lost, and sending any other packet.
  RDM_BROADCAST_PACKET_SPACING = 176,               // Spacing between the RDM controller sending a broadcast packet and sending any other packet.
  RDM_RESPOND_TO_REQUEST_PACKET_SPACING = 176,      // Spacing between the RDM controller sending a request and the RDM responder sending a response packet.

  RDM_CONTROLLER_RESPONSE_LOST_TIMEOUT = 2800,  // The timeout at which the RDM controller considers a response packet lost. 
  RDM_RESPONDER_RESPONSE_LOST_TIMEOUT = 2000    // The timeout at which the RDM responder considers a response packet lost.
};

/**
 * @brief DMX start codes. These are the start codes used within the DMX
 * specification. This enum also includes RDM specific codes than can be used in
 * place of a start code or are use similarly to DMX start codes.
 */
enum dmx_start_code {
  /**
   * @brief DMX default NULL start code. A NULL start code identifies subsequent
   * data slots as a block of untyped sequential 8-bit information. Packets
   * identified by a NULL start code are the default packets sent on DMX
   * networks.
   */
  DMX_SC = 0x00,

  /**
   * @brief Remote Device Management (RDM) start code. RDM is an extension to
   * DMX. A key goal of the RDM standard is to allow the use of new and legacy
   * DMX receiving devices in mixed systems with new RDM equipment and to
   * provide a straightforward path to upgrade existing DMX distribution systems
   * for support of the RDM protocol. The use of RDM devices in a DMX system
   * will not compromise any DMX functionality.
   */
  RDM_SC = 0xcc,

  /**
   * @brief The sub-START code within RDM shall be sent in the slot after the
   * RDM START code (RDM_SC). Future versions of the ANSI-ESTA E1.20 standard
   * which may have additional or different packet structures would use the
   * sub-START code code to identify the packet structure being used.
   *
   * Controllers shall always send RDM_SUB_SC, and responders shall ignore any
   * packets containing other values.
   */
  RDM_SUB_SC = 0x01,

  /**
   * @brief When an RDM device responds to a DISC_UNIQUE_BRANCH command, it
   * responds with a special response packet. Seven extra slots of preamble have
   * been added to the start of the response packet to allow for inline devices
   * that must shorten the packet for turning around transceivers. If the
   * in-line device shortens the response packet, it shall shorten by exactly
   * one slot time. The controller shall be able to process response packets
   * with 0-7 bytes of preamble.
   */
  RDM_PREAMBLE = 0xfe,

  /**
   * @brief When an RDM device responds to a DISC_UNIQUE_BRANCH command, it
   * responds with a special response packet. After sending up to seven extra
   * slots of preamble in the response packet, a single byte delimiter is sent
   * to indicate the start of the packet data.
   */
  RDM_DELIMITER = 0xaa,

  /**
   * @brief ASCII Text alternate start code. Alternate start code 0x17
   * designates a special packet of between 3 and 512 data slots. The purpose of
   * the ASCII text packet is to allow equipment to send diagnostic information
   * coded per the American Standard Code for Information Interchange and
   * formatted for display.
   *
   * Slot allocation is as follows:
   *   Slot 1: Page number of one of the possible 256 text pages.
   *   Slot 2: Characters per line. Indicates the number of characters per line
   *     that the transmitting device has used for the purposes of formatting
   *     the text. A slot value of zero indicates ignore this field.
   *   Slots 3-512: Consecutive display characters in ASCII format. All
   *     characters are allowed and where a DMX512 text viewer is capable, it
   *     shall display the data using the ISO/IEC 646 standard character set. A
   *     slot value of zero shall terminate the ASCII string. Slots transmitted
   *     after this null terminator up to the reset sequence shall be ignored.
   */
  DMX_TEXT_SC = 0x17,

  /**
   * @brief Test Packet alternate start code. Alternate start code 0x55
   * designates a special test packet of 512 data slots, where all data slots
   * carry the value 0x55. Test packets shall be sent so that the time from the
   * start of the break until the stop bit of the 513th slot shall be no more
   * than 25 milliseconds. When test packets are sent back to back, the
   * mark-before-break time shall be no more than 88 microseconds. The break
   * timing for test packets shall be greater than or equal to 88 microseconds,
   * and less than or equal to 120 microseconds. The mark-after-break time shall
   * be greater than or equal to 8 microseconds and less than or equal to 16
   * microseconds.
   */
  DMX_TEST_SC = 0x55,

  /**
   * @brief UTF-8 Text Packet alternate start code. Alternate start code 0x90
   * designates a special packet of between 3 and 512 data slots. The purpose of
   * the UTF-8 Text Packet is to allow equipment to send diagnostic information
   * coded per UTF-8 as described in Unicode 5.0 published by The Unicode
   * Consortium and formatted for display. UTF-8 should only be used when the
   * text packet cannot be expressed in ASCII using the DMX_TEXT_ASC start code.
   *
   * Slot allocation is as follows:
   *   Slot 1: Page number of one of the possible 256 text pages.
   *   Slot 2: Characters per Line. Indicates the number of characters per line
   *     that the transmitting device has used for the purposes of formatting
   *     the text. A slot value of zero indicates "Ignore this field."
   *   Slots 3-512:
   *     Consecutive display characters in UTF-8 format. All characters are
   *     allowed and where a DMX512 text viewer is capable, it shall display the
   *     data using the Unicode 5.0 character set. A slot value of zero shall
   *     terminate the UTF-8 text string. Slots transmitted after this null
   *     terminator up to the reset sequence shall be ignored.
   */
  DMX_UTF8_SC = 0x90,

  /**
   * @brief Manufacturer/Organization ID alternate start code. Alternate start
   * code 0x91 followed by a 2 byte manufacturer ID field is reserved for
   * Manufacturer/Organization specific use, transmitted byte order is MSB, LSB.
   * The next byte after the manufacturers ID would normally be a manufacturer’s
   * sub-code
   */
  DMX_ORG_ID_SC = 0x91,

  /**
   * @brief System Information Packet alternate start code. Alternate startcode
   * 0xCF is reserved for a System Information Packet (SIP). The SIP includes a
   * method of sending checksum data relating to the previous NULL start code
   * packet on the data link and other control information. No other packet
   * shall be sent between the NULL start code packet and the SIP that carries
   * its checksum.
   *
   * For more information on the System Information Packet alternate start code,
   * see annex D5 in the ANSI-ESTA E1.11 DMX512-A standards document.
   */
  DMX_SIP_SC = 0xcf
};

/* DMX parameter checking macros */
/**
 * @brief Evaluates to true if the start code is a start code permitted in a
 * non-prototype DMX device.
 *
 * Several alternate start codes are reserved for special purposes or for
 * future development of the standard. No equipment shall be manufactured that
 * generates alternate start codes 0x92-0xA9 or 0xAB-0xCD until their use is
 * defined by the standard or by the E1 Accredited Standards Committee.
 * Manufacturers shall not advertise or sell products or devices that use
 * alternate start codes 0xF0-0xF7.
 */
#define DMX_START_CODE_IS_VALID(sc)                              \
  (!(sc >= 0x92 && sc <= 0xa9) && !(sc >= 0xab && sc <= 0xcd) && \
   !(sc >= 0xf0 && sc <= 0xf7))

/**
 * @brief Evaluates to true if the baud rate is within DMX specification.
 */
#define DMX_BAUD_RATE_IS_VALID(baud) \
  (baud >= DMX_MIN_BAUD_RATE && baud <= DMX_MAX_BAUD_RATE)

/**
 * @brief Evaluates to true if the received break duration is within DMX
 * specification.
 */
#define DMX_BREAK_LEN_IS_VALID(brk) \
  (brk >= DMX_MIN_BREAK_LEN_US && brk <= DMX_MAX_BREAK_LEN_US)

/**
 * @brief Evaluates to true if the received mark-after-break duration is within
 * DMX specification.
 */
#define DMX_MAB_LEN_IS_VALID(mab) \
  (mab >= DMX_MIN_MAB_LEN_US && mab <= DMX_MAX_MAB_LEN_US)

/**
 * @brief Evaluates to true if the baud rate is within RDM specification.
 */
#define RDM_BAUD_RATE_IS_VALID(baud) \
  (baud >= DMX_MIN_BAUD_RATE && baud <= DMX_MAX_BAUD_RATE)

/**
 * @brief Evaluates to true if the received break duration is within RDM
 * specification.
 */
#define RDM_BREAK_LEN_IS_VALID(brk) \
  (brk >= RDM_MIN_BREAK_LEN_US && brk <= RDM_MAX_BREAK_LEN_US)

/**
 * @brief Evaluates to true if the received mark-after-break duration is within
 * RDM specification.
 */
#define RDM_MAB_LEN_IS_VALID(mab) \
  (mab >= RDM_MIN_MAB_LEN_US && mab <= RDM_MAX_MAB_LEN_US)

#ifdef __cplusplus
}
#endif
