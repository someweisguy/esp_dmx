/**
 * @file rdm/utils/bus_ctl.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief // TODO
 * 
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Reads an RDM packet from the DMX driver buffer. Header information is
 * emplaced into a header pointer so that it may be read by the caller.
 * Parameter data information needs to be emplaced before it can be properly
 * read by the caller. This function does not perform any data error checking to
 * ensure that the RDM packet is within specification.
 *
 * @param dmx_num The DMX port number.
 * @param[out] header A pointer which stores RDM header information.
 * @param[out] pd A pointer to store parameter data from the RDM packet.
 * @param num The size of the pd pointer. Used to prevent buffer overflows.
 * @return The size of the RDM packet that was read or 0 on error.
 */
size_t rdm_read(dmx_port_t dmx_num, rdm_header_t *header, void *pd, size_t num);

/**
 * @brief Writes an RDM packet into the DMX driver buffer so it may be sent with
 * dmx_send(). Header information is emplaced into the DMX driver buffer but
 * parameter data information must be emplaced before calling this function to
 * ensure that the RDM packet is properly formatted. This function does not
 * perform any data error checking to ensure that the RDM packet is within
 * specification.
 *
 * @param dmx_num The DMX port number.
 * @param[in] header A pointer which stores RDM header information.
 * @param[in] pd A pointer which stores parameter data to be written.
 * @return The size of the RDM packet that was written or 0 on error.
 */
size_t rdm_write(dmx_port_t dmx_num, rdm_header_t *header, const void *pd);

/**
 * @brief Emplaces parameter data from a source buffer to a destination buffer.
 * It is necessary to emplace parameter data before it is written and read to
 * ensure it is formatted correctly for the RDM data bus or for the ESP32's
 * memory. Emplacing data swaps the endianness of each parameter field and also
 * optionally writes null terminators for strings and writes optional UID
 * fields. The destination buffer and the source buffer may overlap.
 *
 * Parameter fields are emplaced using a format string. This provides the
 * instructions on how data is written. Fields are written in the order provided
 * in the format string. The following characters can be used to write parameter
 * data:
 * - 'b' writes an 8-bit byte of data.
 * - 'w' writes a 16-bit word of data.
 * - 'd' writes a 32-bit dword of data.
 * - 'u' writes a 48-bit UID.
 * - 'v' writes an optional 48-bit UID if the UID is not 0000:00000000. Optional
 *   UIDs must be at the end of the format string.
 * - 'a' writes an ASCII string. ASCII strings may be up to 32 characters long
 *   and may or may not be null-terminated. An ASCII string must be at the end
 *   of the format string.
 *
 * Integer literals may be written by beginning the integer with '#' and writing
 * the literal in hexadecimal form. Integer literals must be terminated with an
 * 'h' character. For example, the integer 0xbeef is represented as "#beefh".
 * Integer literals are written regardless of what the underlying value is. This
 * is used for situations such as emplacing a rdm_device_info_t wherein the
 * first two bytes are 0x01 and 0x00.
 *
 * Parameters will continue to be emplaced as long as the number of bytes
 * written does not exceed the size of the destination buffer, as provided in
 * the num argument. A single parameter may be emplaced instead of multiple by
 * including a '$' character at the end of the format string.
 *
 * Null terminators are not used for strings sent on the RDM data bus. When
 * emplacing data onto the RDM data bus, the emplace_nulls argument should be
 * set to false. When emplacing into ESP32 memory to be read by the caller,
 * emplace_nulls should be set to true to ensure that strings are null
 * terminated. Setting emplace_nulls to true will also affect optional UID
 * fields by emplacing a 0000:00000000 into the destination buffer when an
 * optional UID is not present in the source buffer. When emplace_nulls is
 * false, optional UIDs will not be emplaced when its value is 0000:00000000. It
 * is considered good practice to set emplace_nulls to true when the destination
 * buffer is intended to be read by the user, and false when the destination
 * buffer will be sent on the RDM data bus.
 *
 * Example format strings and their corresponding PIDs are included below.
 *
 * RDM_PID_DISC_UNIQUE_BRANCH: "uu$"
 * RDM_PID_DISC_MUTE: "wv$"
 * RDM_PID_DEVICE_INFO: "#0100hwwdwbbwwb$"
 * RDM_PID_SOFTWARE_VERSION_LABEL: "a$"
 * RDM_PID_DMX_START_ADDRESS: "w$"
 *
 * @param[out] destination The destination into which to emplace the data.
 * @param[in] format The format string which instructs the function how to
 * emplace data.
 * @param[in] source The source buffer which is emplaced into the destination.
 * @param num The maximum number of bytes to emplace.
 * @param emplace_nulls True to emplace null terminators and optional UIDs into
 * the source buffer.
 * @return The size of the data that was emplaced.
 */
// TODO
// size_t rdm_emplace(void *destination, const char *format, const void *source,
//                    size_t num, bool emplace_nulls);

/**
 * @brief Sends an RDM controller request and processes the response. This
 * function writes, sends, receives, and reads a request and response RDM
 * packet. It performs error checking on the written packet to ensure that it
 * adheres to RDM specification and prevents RDM bus errors. An rdm_ack_t is
 * provided to process DMX and RDM errors.
 * - ack.err will evaluate to true if an error occurred during the sending or
 *   receiving of raw DMX data. RDM data will not be processed if an error
 *   occurred. If a response was expected but none was received, ack.err will
 *   evaluate to DMX_ERR_TIMEOUT. If no response was expected, ack.err will be
 *   set to DMX_OK.
 * - ack.size is the size of the received RDM packet, including the RDM
 *   checksum.
 * - ack.src_uid is the UID of the device which responds to the request.
 * - ack.pid is the PID of the RDM response packet. This is typically the same
 *   as the PID which was sent in the RDM request, but may be a different value
 *   in certain responses.
 * - ack.type will evaluate to RDM_RESPONSE_TYPE_INVALID if an invalid
 *   response is received but does not necessarily indicate a DMX error
 *   occurred. If no response is received ack.type will be set to
 *   RDM_RESPONSE_TYPE_NONE whether or not a response was expected. Otherwise,
 *   ack.type will be set to the ack type received in the RDM response.
 * - ack.message_count indicates the number of messages that are waiting to be
 *   retrieved from the responder's message queue.
 * - ack.timer and ack.nack_reason are a union which should be read depending on
 *   the value of ack.type. If ack.type is RDM_RESPONSE_TYPE_ACK_TIMER,
 *   ack.timer should be read. ack.timer is the estimated amount of time in
 *   FreeRTOS ticks until the responder is able to provide a response to the
 *   request. If ack.type is RDM_RESPONSE_TYPE_NACK_REASON, ack.nack_reason
 *   should be read to get the NACK reason.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which stores header information for the RDM
 * request.
 * @param[in] pd_in A pointer which stores parameter data to be written.
 * @param[out] pd_out A pointer which stores parameter data which was read, if a
 * response was received. This can be an alias of pd_in.
 * @param[inout] pdl The size of the pd_out buffer. When receiving data, this is
 * set to the PDL of the received data. Used to prevent buffer overflows.
 * @param[out] ack A pointer to an rdm_ack_t which stores information about the
 * RDM response.
 * @return true if an RDM_RESPONSE_TYPE_ACK response was received.
 * @return false if any other response type was received.
 */
bool rdm_send_request(dmx_port_t dmx_num, rdm_header_t *header,
                      const void *pd_in, void *pd_out, size_t *pdl,
                      rdm_ack_t *ack);

// TODO: docs
rdm_pid_t rdm_queue_pop(dmx_port_t dmx_num);

// TODO docs
uint8_t rdm_queue_size(dmx_port_t dmx_num);

// TODO: docs
rdm_pid_t rdm_queue_get_last_sent(dmx_port_t dmx_num);

// TODO: docs
void rdm_set_boot_loader(dmx_port_t dmx_num);

// TODO: docs
bool rdm_status_push(dmx_port_t dmx_num, const rdm_status_message_t *message);

// TODO: docs
bool rdm_status_pop(dmx_port_t dmx_num, rdm_status_t status,
                   rdm_status_message_t *message);

// TODO docs
void rdm_status_clear(dmx_port_t dmx_num);

// TODO: docs
rdm_status_t rdm_status_get_threshold(dmx_port_t dmx_num);

// TODO: docs
void rdm_status_set_threshold(dmx_port_t dmx_num, rdm_status_t status);

#ifdef __cplusplus
}
#endif