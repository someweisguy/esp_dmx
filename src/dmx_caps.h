#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* DMX shared parameters */
#define DMX_MIN_BAUD_RATE           (245000)        // DMX minimum baud rate.
#define DMX_TYP_BAUD_RATE           (250000)        // DMX typical baud rate.
#define DMX_MAX_BAUD_RATE           (255000)        // DMX maximum baud rate.
#define DMX_MAX_PACKET_SIZE         (513)           // DMX maximum packet size.

/* DMX client/receive timing parameters */
#define DMX_RX_MIN_SPACE_FOR_BRK_US (88)            // DMX minimum receivable break length in microseconds.
#define DMX_RX_MIN_MRK_AFTER_BRK_US (8)             // DMX minimum receivable mark after break length in microseconds.
#define DMX_RX_MAX_MRK_AFTER_BRK_US (999999)        // DMX maximum receivable mark after break length in microseconds.
#define DMX_RX_MIN_BRK_TO_BRK_US    (1196)          // DMX minimum receivable break-to-break length in microseconds.
#define DMX_RX_MAX_BRK_TO_BRK_US    (1250000)       // DMX maximum receivable break-to-break length in microseconds.
#define DMX_RX_PACKET_TOUT_MS       (1250)          // DMX client packet timeout in milliseconds.
#define DMX_RX_PACKET_TOUT_TICK     ((TickType_t)DMX_RX_PACKET_TOUT_MS / portTICK_PERIOD_MS) // DMX client packet timeout in FreeRTOS ticks.

/* DMX host/transmit timing parameters */
#define DMX_TX_MIN_SPACE_FOR_BRK_US (92)            // DMX minimum transmittable break length in microseconds.
#define DMX_TX_MIN_MRK_AFTER_BRK_US (12)            // DMX minimum transmittable mark after break length in microseconds.
#define DMX_TX_MAX_MRK_AFTER_BRK_US (999999)        // DMX maximum transmittable mark after break length in microseconds.
#define DMX_TX_MIN_BRK_TO_BRK_US    (1204)          // DMX minimum transmittable break-to-break length in microseconds.
#define DMX_TX_MAX_BRK_TO_BRK_US    (1000000)       // DMX maximum transmittable break-to-break length in microseconds.
#define DMX_TX_PACKET_TOUT_MS       (1000)          // DMX host packet timeout in milliseconds.
#define DMX_TX_PACKET_TOUT_TICK     ((TickType_t)DMX_TX_PACKET_TOUT_MS / portTICK_PERIOD_MS) // DMX host packet timeout in FreeRTOS ticks.

#define DMX_PACKET_TIMEOUT_TICK     (DMX_RX_PACKET_TOUT_TICK)  // DMX host packet timeout in FreeRTOS ticks.

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
#define DMX_START_CODE_IS_VALID(sc) \
  (!(sc >= 0x92 && sc <= 0xa9) && !(sc >= 0xab && sc <= 0xcd) && !(sc >= 0xf0 && sc <= 0xf7))

/**
 * @brief Evaluates to true if the received packet duration is within DMX
 * specification.
 */
#define DMX_RX_PKT_DURATION_IS_VALID(pkt) \
  (pkt >= DMX_RX_MIN_BRK_TO_BRK_US && pkt <= DMX_RX_MAX_BRK_TO_BRK_US)

/**
 * @brief Evaluates to true if the received break duration is within DMX
 * specification.
 */
#define DMX_RX_BRK_DURATION_IS_VALID(brk) \
    (brk >= DMX_RX_MIN_SPACE_FOR_BRK_US)

/**
 * @brief Evaluates to true if the received mark-after-break duration is within
 * DMX specification.
 */
#define DMX_RX_MAB_DURATION_IS_VALID(mab) \
  (mab >= DMX_RX_MIN_MRK_AFTER_BRK_US && mab <= DMX_RX_MAX_MRK_AFTER_BRK_US)

/**
 * @brief Evaluates to true if the transmitted packet duration is within DMX
 * specification.
 */
#define DMX_TX_PKT_DURATION_IS_VALID(pkt) \
 (pkt >= DMX_TX_MIN_BRK_TO_BRK_US && pkt <= DMX_TX_MAX_BRK_TO_BRK_US)

/**
 * @brief Evaluates to true if the transmitted break duration is within DMX
 * specification.
 */
#define DMX_TX_BRK_DURATION_IS_VALID(brk) \
  (brk >= DMX_TX_MIN_SPACE_FOR_BRK_US)

/**
 * @brief Evaluates to true of the transmitted mark-after-break duration is
 * within DMX specification.
 * 
 */
#define DMX_TX_MAB_DURATION_IS_VALID(mab) \
  (mab >= DMX_TX_MIN_MRK_AFTER_BRK_US && mab <= DMX_TX_MAX_MRK_AFTER_BRK_US)

/* DMX start codes */
/**
 * @brief DMX default NULL start code. A NULL start code identifies subsequent
 * data slots as a block of untyped sequential 8-bit information. Packets
 * identified by a NULL start code are the default packets sent on DMX 
 * networks.
 */
#define DMX_SC     (0x00)

/**
 * @brief Remote Device Management (RDM) start code. RDM is an extension to
 * DMX. A key goal of the RDM standard is to allow the use of new and legacy
 * DMX receiving devices in mixed systems with new RDM equipment and to provide
 * a straightforward path to upgrade existing DMX distribution systems for
 * support of the RDM protocol. The use of RDM devices in a DMX system will not
 * compromise any DMX functionality. 
 */
#define RDM_SC     (0xcc)

/**
 * @brief ASCII Text alternate start code. Alternate start code 0x17 designates
 * a special packet of between 3 and 512 data slots. The purpose of the ASCII
 * text packet is to allow equipment to send diagnostic information coded per
 * the American Standard Code for Information Interchange and formatted for 
 * display.
 * 
 * Slot allocation is as follows:
 *   Slot 1: Page number of one of the possible 256 text pages.
 *   Slot 2: Characters per line. Indicates the number of characters per line
 *     that the transmitting device has used for the purposes of formatting the
 *     text. A slot value of zero indicates ignore this field.
 *   Slots 3-512: Consecutive display characters in ASCII format. All
 *     characters are allowed and where a DMX512 text viewer is capable, it
 *     shall display the data using the ISO/IEC 646 standard character set. A
 *     slot value of zero shall terminate the ASCII string. Slots transmitted 
 *     after this null terminator up to the reset sequence shall be ignored.
 */
#define DMX_TEXT_ASC     (0x17) 

/**
 * @brief Test Packet alternate start code. Alternate start code 0x55 designates a
 * special test packet of 512 data slots, where all data slots carry the value
 * 0x55. Test packets shall be sent so that the time from the start of the
 * break until the stop bit of the 513th slot shall be no more than 25
 * milliseconds. When test packets are sent back to back, the mark-before-break
 * time shall be no more than 88 microseconds. The break timing for test
 * packets shall be greater than or equal to 88 microseconds, and less than or
 * equal to 120 microseconds. The mark-after-break time shall be greater than
 * or equal to 8 microseconds and less than or equal to 16 microseconds.
 */
#define DMX_TEST_ASC     (0x55)

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
 *     that the transmitting device has used for the purposes of formatting the
 *     text. A slot value of zero indicates "Ignore this field."
 *   Slots 3-512: Consecutive display characters in UTF-8 format. All
 *     characters are allowed and where a DMX512 text viewer is capable, it
 *     shall display the data using the Unicode 5.0 character set. A slot value
 *     of zero shall terminate the UTF-8 text string. Slots transmitted after
 *     this null terminator up to the reset sequence shall be ignored.
 */
#define DMX_UTF8_ASC     (0x90)

/**
 * @brief Manufacturer/Organization ID alternate start code. Alternate start
 * code 0x91 followed by a 2 byte manufacturer ID field is reserved for 
 * Manufacturer/Organization specific use, transmitted byte order is MSB, LSB.
 * The next byte after the manufacturers ID would normally be a manufacturer’s
 * sub-code
 */
#define DMX_ORG_ID_ASC   (0x91)

/**
 * @brief System Information Packet alternate start code. Alternate startcode
 * 0xCF is reserved for a System Information Packet (SIP). The SIP includes a
 * method of sending checksum data relating to the previous NULL start code
 * packet on the data link and other control information. No other packet shall
 * be sent between the NULL start code packet and the SIP that carries its 
 * checksum.
 * 
 * For more information on the System Information Packet alternate start code,
 * see annex D5 in the ANSI-ESTA E1.11 DMX512-A standards document.
 */
#define DMX_SIP_ASC      (0xcf)

#ifdef __cplusplus
}
#endif
