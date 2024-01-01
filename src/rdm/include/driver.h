#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"

/**
 * @brief Returns the 48-bit unique ID of the desired DMX port. The specified
 * DMX driver must be installed before calling this function.
 *
 * @param dmx_num The DMX port number.
 * @return A pointer to the DMX driver's RDM UID or NULL on failure.
 */
const rdm_uid_t *rdm_uid_get(dmx_port_t dmx_num);

/**
 * @brief Reads an RDM packet from the DMX driver buffer. Header information is
 * read into a header pointer so that it may be read by the caller.
 *
 * @param dmx_num The DMX port number.
 * @param[out] header A pointer which stores RDM header information.
 * @return true if the packet is a valid RDM packet.
 * @return false if the packet is not a valid RDM packet.
 */
bool rdm_read_header(dmx_port_t dmx_num, rdm_header_t *header);

/**
 * @brief Reads RDM parameter data from the DMX driver buffer. This function
 * does not verify that the packet being read is a valid RDM packet.
 *
 * Parameter fields are written and read using a format string. This provides
 * instructions on how data is written. It is necessary to use a format string
 * because RDM requires that fields in a parameter are written and read in big
 * endian format. The following characters can be used to write parameter data:
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
 * Integer literals may be written and read by beginning the integer with 'x'
 * and writing the literal in hexadecimal form. Integer literals must contain
 * two hexadecimal digits. For example, 0xAB is represented as "xab". Integer
 * literals are written into the destination regardless of what the underlying
 * value is. This is used for situations such as reading or writing an
 * rdm_device_info_t wherein the first two bytes are 0x01 and 0x00.
 *
 * Parameters will continue to be written or read as long as the number of bytes
 * does not exceed the size of the destination buffer as provided by the num
 * argument. A single parameter may be written or read instead of multiple by
 * including a '$' character at the end of the format string. The 'a' and 'v'
 * fields must be at the end of each parameter. When reading or writing a
 * parameter which uses one of these fields, a terminating '$' is not necessary
 * though it may be used for readability.
 *
 * Example format strings and their corresponding PIDs are included below.
 *
 * RDM_PID_DISC_UNIQUE_BRANCH: "uu$"
 * RDM_PID_DISC_MUTE: "wv$" or "wv"
 * RDM_PID_DEVICE_INFO: "x01x00wwdwbbwwb$"
 * RDM_PID_SOFTWARE_VERSION_LABEL: "a$" or "a"
 * RDM_PID_DMX_START_ADDRESS: "w$"
 *
 * @param dmx_num The DMX port number.
 * @param[in] format The format string of the RDM parameter data.
 * @param[out] destination A pointer to a destination buffer into which to copy
 * parameter data.
 * @param size The size of the destination buffer.
 * @return The size of the RDM parameter data or 0 on error.
 */
size_t rdm_read_pd(dmx_port_t dmx_num, const char *format, void *destination,
                   size_t size);

/**
 * @brief Writes an RDM packet into the DMX driver buffer so it may be sent with
 * dmx_send().
 *
 * Parameter fields are written and read using a format string. This provides
 * instructions on how data is written. It is necessary to use a format string
 * because RDM requires that fields in a parameter are written and read in big
 * endian format. The following characters can be used to write parameter data:
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
 * Integer literals may be written and read by beginning the integer with 'x'
 * and writing the literal in hexadecimal form. Integer literals must contain
 * two hexadecimal digits. For example, 0xAB is represented as "xab". Integer
 * literals are written into the destination regardless of what the underlying
 * value is. This is used for situations such as reading or writing an
 * rdm_device_info_t wherein the first two bytes are 0x01 and 0x00.
 *
 * Parameters will continue to be written or read as long as the number of bytes
 * does not exceed the size of the destination buffer as provided by the num
 * argument. A single parameter may be written or read instead of multiple by
 * including a '$' character at the end of the format string. The 'a' and 'v'
 * fields must be at the end of each parameter. When reading or writing a
 * parameter which uses one of these fields, a terminating '$' is not necessary
 * though it may be used for readability.
 *
 * Example format strings and their corresponding PIDs are included below.
 *
 * RDM_PID_DISC_UNIQUE_BRANCH: "uu$"
 * RDM_PID_DISC_MUTE: "wv$" or "wv"
 * RDM_PID_DEVICE_INFO: "x01x00wwdwbbwwb$"
 * RDM_PID_SOFTWARE_VERSION_LABEL: "a$" or "a"
 * RDM_PID_DMX_START_ADDRESS: "w$"
 *
 * @param dmx_num The DMX port number.
 * @param[in] header A pointer which stores RDM header information.
 * @param[in] format The format string of the RDM parameter data.
 * @param[in] pd A pointer which stores parameter data to be written.
 * @return The size of the RDM packet that was written or 0 on error.
 */
size_t rdm_write(dmx_port_t dmx_num, const rdm_header_t *header,
                 const char *format, const void *pd);

/**
 * @brief Returns true if the RDM format string is valid.
 *
 * @param format The RDM format string.
 * @return true if the RDM format string is valid.
 * @return false if it is not valid.
 */
bool rdm_format_is_valid(const char *format);