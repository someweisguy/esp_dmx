#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"

#define RDM_MAX_UID (0xfffffffffffe)  // The highest UID possible in RDM.
#define RDM_MESSAGE_LEN_INDEX (2)  // Index of a standard RDM packet message length byte.

/* RDM discovery packet descriptors */
#define RDM_DISCOVERY_RESP_LEN (17)  // Length of the RDM DISC_UNIQUE_BRANCH response after the preamble.
#define RDM_PREAMBLE_MAX_LEN (7)  // Maximum length of the RDM DISC_UNIQUE_BRANCH response preamble.

// General DMX parameters
enum {
  DMX_BAUD_RATE = 250000,      // The typical baud rate of DMX.
  DMX_MIN_BAUD_RATE = 245000,  // The minimum baud rate of DMX.
  DMX_MAX_BAUD_RATE = 255000,  // The maximum baud rate of DMX.

  DMX_BREAK_LEN_US = 176,  // The typical break length of DMX in microseconds.

  DMX_PACKET_SIZE = 513,     // The typical packet size of DMX.
  DMX_MAX_PACKET_SIZE = 513  // The maximum packet size of DMX.
};

// DMX Read timing parameters
enum {
  DMX_READ_MIN_PACKET_LEN_US = 1196,     // The minimum DMX responder packet length in microseconds.
  DMX_READ_MAX_PACKET_LEN_US = 1250000,  // The maximum DMX responder packet length in microseconds.

  DMX_READ_TIMEOUT_US = 1250000,                                            // The DMX responder timeout length in microseconds. If it takes longer than this amount of time to receive the next DMX packet the signal is considered lost.
  DMX_READ_TIMEOUT_TICK = DMX_READ_TIMEOUT_US / 1000 / portTICK_PERIOD_MS,  // The DMX responder timeout length in FreeRTOS ticks. If it takes longer than this amount of time to receive the next DMX packet the signal is considered lost.

  DMX_READ_MIN_BREAK_LEN_US = 88,                          // The minimum DMX responder break length in microseconds.
  DMX_READ_MAX_BREAK_LEN_US = DMX_READ_MAX_PACKET_LEN_US,  // The maximum DMX responder break length in microseconds. This value isn't specified in the DMX standard, so DMX_READ_MAX_PACKET_LEN_US is used instead.

  DMX_READ_MIN_MAB_LEN_US = 8,       // The minimum DMX responder mark-after-break length in microseconds.
  DMX_READ_MAX_MAB_LEN_US = 999999,  // The maximum DMX responder mark-after-break length in microseconds.
};

// DMX Write timing parameters
enum {
  DMX_WRITE_MIN_PACKET_LEN_US = 1204,     // The minimum DMX controller packet length in microseconds.
  DMX_WRITE_MAX_PACKET_LEN_US = 1000000,  // The maximum DMX controller packet length in microseconds.

  DMX_WRITE_TIMEOUT_US = 1000000,                                             // The DMX controller timeout length in microseconds. If it takes longer than this amount of time to receive the next DMX packet the signal is considered lost.
  DMX_WRITE_TIMEOUT_TICK = DMX_WRITE_TIMEOUT_US / 1000 / portTICK_PERIOD_MS,  // The DMX controller timeout length in FreeRTOS ticks. If it takes longer than this amount of time to receive the next DMX packet the signal is considered lost.

  DMX_WRITE_MIN_BREAK_LEN_US = 92,                           // The minimum DMX controller break length in microseconds.
  DMX_WRITE_MAX_BREAK_LEN_US = DMX_WRITE_MAX_PACKET_LEN_US,  // The maximum DMX controller break length in microseconds. This value isn't specified in the DMX standard, so DMX_WRITE_MAX_PACKET_LEN_US is used instead.

  DMX_WRITE_MIN_MAB_LEN_US = 12,      // The minimum DMX controller mark-after-break length in microseconds.
  DMX_WRITE_MAX_MAB_LEN_US = 999999,  // The maximum DMX controller mark-after-break length in microseconds.
};

// DMX start codes
enum {
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
 * @brief Evaluates to true if the baud rate is within DMX specification.
 */
#define DMX_BAUD_RATE_IS_VALID(baud) \
  (baud >= DMX_MIN_BAUD_RATE && baud <= DMX_MAX_BAUD_RATE)

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
 * @brief Evaluates to true if the received packet duration is within DMX
 * specification.
 */
// #define DMX_RX_PKT_DURATION_IS_VALID(pkt)  (pkt >= DMX_RX_MIN_BRK_TO_BRK_US && pkt <= DMX_RX_MAX_BRK_TO_BRK_US)

/**
 * @brief Evaluates to true if the received break duration is within DMX
 * specification.
 */
// #define DMX_RX_BRK_DURATION_IS_VALID(brk) (brk >= DMX_RX_MIN_SPACE_FOR_BRK_US)

/**
 * @brief Evaluates to true if the received mark-after-break duration is within
 * DMX specification.
 */
// #define DMX_RX_MAB_DURATION_IS_VALID(mab)    (mab >= DMX_RX_MIN_MRK_AFTER_BRK_US && mab <= DMX_RX_MAX_MRK_AFTER_BRK_US)

/**
 * @brief Evaluates to true if the transmitted packet duration is within DMX
 * specification.
 */
// #define DMX_TX_PKT_DURATION_IS_VALID(pkt)    (pkt >= DMX_TX_MIN_BRK_TO_BRK_US && pkt <= DMX_TX_MAX_BRK_TO_BRK_US)

/**
 * @brief Evaluates to true if the transmitted break duration is within DMX
 * specification.
 */
//#define DMX_TX_BRK_DURATION_IS_VALID(brk) (brk >= DMX_TX_MIN_SPACE_FOR_BRK_US)

/**
 * @brief Evaluates to true of the transmitted mark-after-break duration is
 * within DMX specification.
 *
 */
//#define DMX_TX_MAB_DURATION_IS_VALID(mab)   (mab >= DMX_TX_MIN_MRK_AFTER_BRK_US && mab <= DMX_TX_MAX_MRK_AFTER_BRK_US)

#ifdef __cplusplus
}
#endif
