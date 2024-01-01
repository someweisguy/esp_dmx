/**
 * @file dmx/include/driver.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This header defines functions which allow for installation and
 * configuration of the DMX driver. This includes functions which modify DMX
 * timing and can enable or disable the DMX driver.
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Installs the DMX driver, sets the default configuration, and sets the
 * DMX personalities.
 *
 * @note The DMX driver will allocate a hardware timer to use. When using
 * ESP-IDF v4.4 the DMX driver will allocate a hardware timer group and timer
 * relative to the DMX port number. It is not recommended to use the hardware
 * timer that the DMX driver is using while the DMX driver is installed.
 *
 * @note The DMX interrupt service routine is installed on the same CPU core
 * that this function is running on.
 *
 * @param dmx_num The DMX port number.
 * @param[in] config A pointer to a DMX configuration which will be used to
 * setup the DMX driver.
 * @param[in] personalities A pointer to an array of DMX personalities which
 * the DMX driver will support.
 * @param personality_count The number of personalities in the previous
 * argument. May be set to 0 if this device does not support a DMX address.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_driver_install(dmx_port_t dmx_num, dmx_config_t *config,
                        dmx_personality_t *personalities,
                        uint8_t personality_count);

/**
 * @brief Uninstalls the DMX driver.
 *
 * @param dmx_num The DMX port number
 * @return true on success.
 * @return false on failure.
 */
bool dmx_driver_delete(dmx_port_t dmx_num);

/**
 * @brief Disables the DMX driver.
 *
 * @note When the DMX driver is not placed in IRAM, functions which disable the
 * cache, such as functions which read or write to flash, will also stop DMX
 * interrupts from firing. This can cause incoming DMX data to become corrupted.
 * To avoid this issue, the DMX driver should be disabled before disabling the
 * cache. When cache is reenabled, the DMX driver can be reenabled as well. When
 * the DMX driver is placed in IRAM, disabling and reenabling the DMX driver is
 * not needed.
 *
 * @param dmx_num The DMX port number.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_driver_disable(dmx_port_t dmx_num);

/**
 * @brief Enables the DMX driver.
 *
 * @note When the DMX driver is not placed in IRAM, functions which disable the
 * cache, such as functions which read or write to flash, will also stop DMX
 * interrupts from firing. This can cause incoming DMX data to become corrupted.
 * To avoid this issue, the DMX driver should be disabled before disabling the
 * cache. When cache is reenabled, the DMX driver can be reenabled as well. When
 * the DMX driver is placed in IRAM, disabling and reenabling the DMX driver is
 * not needed.
 *
 * @param dmx_num The DMX port number.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_driver_enable(dmx_port_t dmx_num);

/**
 * @brief Checks if DMX driver is installed.
 *
 * @param dmx_num The DMX port number.
 * @return true if the driver is installed.
 * @return false if the driver is not installed or DMX port does not exist.
 * */
bool dmx_driver_is_installed(dmx_port_t dmx_num);

/**
 * @brief Checks if the DMX driver is enabled.
 *
 * @note When the DMX driver is not placed in IRAM, functions which disable the
 * cache, such as functions which read or write to flash, will also stop DMX
 * interrupts from firing. This can cause incoming DMX data to become corrupted.
 * To avoid this issue, the DMX driver should be disabled before disabling the
 * cache. When cache is reenabled, the DMX driver can be reenabled as well. When
 * the DMX driver is placed in IRAM, disabling and reenabling the DMX driver is
 * not needed.
 *
 * @param dmx_num The DMX port number.
 * @return true if the driver is enabled.
 * @return false if the driver is disabled.
 */
bool dmx_driver_is_enabled(dmx_port_t dmx_num);

/**
 * @brief Sets DMX pin number.
 *
 * @param dmx_num The DMX port number.
 * @param tx_pin The pin to which the TX signal will be assigned.
 * @param rx_pin The pin to which the RX signal will be assigned.
 * @param rts_pin The pin to which the RTS signal will be assigned.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_set_pin(dmx_port_t dmx_num, int tx_pin, int rx_pin, int rts_pin);

/**
 * @brief Gets the DMX baud rate.
 *
 * @param dmx_num The DMX port number.
 * @return the current baud rate or 0 on error.
 */
uint32_t dmx_get_baud_rate(dmx_port_t dmx_num);

/**
 * @brief Sets the DMX baud rate. The baud rate will be clamped to DMX
 * specification. If the input baud rate is lower than DMX_MIN_BAUD_RATE it will
 * be set to DMX_MIN_BAUD_RATE. If the input baud rate is higher than
 * DMX_MAX_BAUD_RATE it will be set to DMX_MAX_BAUD_RATE.
 *
 * @param dmx_num The DMX port number.
 * @param baud_rate The baud rate to which to set the DMX driver.
 * @return the value that the baud rate was set to or 0 on error.
 */
uint32_t dmx_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate);

/**
 * @brief Gets the DMX break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX break length or 0 on error.
 */
uint32_t dmx_get_break_len(dmx_port_t dmx_num);

/**
 * @brief Sets the DMX break length in microseconds. The break length will be
 * clamped to DMX specification. If the input break length is lower than
 * DMX_MIN_BREAK_LEN_US it will be set to DMX_MIN_BREAK_LEN_US. If the input
 * break length is higher than DMX_MAX_BREAK_LEN_US it will be set to
 * DMX_MAX_BREAK_LEN_US.
 *
 * @note The DMX break length specification is not the same as the RDM break
 * length specification. It is possible to use this function to set the DMX
 * break length so that RDM is unusable. This function should be used carefully
 * to ensure correct RDM functionality!
 *
 * @param dmx_num The DMX port number.
 * @param break_num The length in microseconds of the DMX break.
 * @return the value that the DMX break length was set to or 0 on error.
 */
uint32_t dmx_set_break_len(dmx_port_t dmx_num, uint32_t break_len);

/**
 * @brief Gets the DMX mark-after-break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX mark-after-break length or 0 on error.
 */
uint32_t dmx_get_mab_len(dmx_port_t dmx_num);

/**
 * @brief Sets the DMX mark-after-break length in microseconds. The
 * mark-after-break length will be clamped to DMX specification. If the input
 * mark-after-break length is lower than DMX_MIN_MAB_LEN_US it will be set to
 * DMX_MIN_MAB_LEN_US. If the input mark-after-break length is higher than
 * DMX_MAX_MAB_LEN_US it will be set to DMX_MAX_MAB_LEN_US.
 *
 * @note The DMX mark-after-break length specification is not the same as the
 * RDM mark-after-break length specification. It is possible to use this
 * function to set the DMX mark-after-break length so that RDM is unusable. This
 * function should be used carefully to ensure correct RDM functionality!
 *
 * @param dmx_num The DMX port number.
 * @param mab_len The length in microseconds of the DMX mark-after-break.
 * @return the value that the DMX mark-after-break length was set to or 0 on
 * error.
 */
uint32_t dmx_set_mab_len(dmx_port_t dmx_num, uint32_t mab_len);

// TODO
/**
 * @brief Returns the 48-bit unique ID of the desired DMX port. The specified
 * DMX driver must be installed before calling this function.
 *
 * @param dmx_num The DMX port number.
 * @return A pointer to the DMX driver's RDM UID or NULL on failure.
 */
const rdm_uid_t *rdm_uid_get(dmx_port_t dmx_num);

/**
 * @brief Reads DMX data from the driver into a destination buffer with an
 * offset. This can be useful when a receiving DMX device only needs to process
 * a small footprint of the DMX packet.
 *
 * @param dmx_num The DMX port number.
 * @param offset The number of slots with which to offset the read. If set to 0
 * this function is equivalent to dmx_read().
 * @param[out] destination The destination buffer into which to read the DMX
 * data.
 * @param size The size of the destination buffer.
 * @return The number of bytes read from the DMX driver.
 */
size_t dmx_read_offset(dmx_port_t dmx_num, size_t offset, void *destination,
                       size_t size);

/**
 * @brief Reads DMX data from the driver into a destination buffer.
 *
 * @param dmx_num The DMX port number.
 * @param[out] destination The destination buffer into which to read the DMX
 * data.
 * @param size The size of the destination buffer.
 * @return The number of bytes read from the DMX driver.
 */
size_t dmx_read(dmx_port_t dmx_num, void *destination, size_t size);

/**
 * @brief Reads a single slot of DMX data.
 *
 * @param dmx_num The DMX port number.
 * @param slot_num The DMX slot number to read.
 * @return The value of the DMX slot or -1 on error.
 */
int dmx_read_slot(dmx_port_t dmx_num, size_t slot_num);

// TODO
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

// TODO
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
 * @brief Writes DMX data from a source buffer into the DMX driver buffer with
 * an offset. Allows a source buffer to be written to a specific slot number in
 * the DMX driver buffer.
 *
 * @param dmx_num The DMX port number.
 * @param offset The number of slots with which to offset the write. If set to 0
 * this function is equivalent to dmx_write().
 * @param[in] source The source buffer which is copied to the DMX driver.
 * @param size The size of the source buffer.
 * @return The number of bytes written into the DMX driver.
 */
size_t dmx_write_offset(dmx_port_t dmx_num, size_t offset, const void *source,
                        size_t size);

/**
 * @brief Writes DMX data from a source buffer into the DMX driver buffer. Data
 * written into the DMX driver buffer can then be sent to DMX devices.
 *
 * @param dmx_num The DMX port number.
 * @param[in] source The source buffer which is copied to the DMX driver.
 * @param size The size of the source buffer.
 * @return The number of bytes written into the DMX driver.
 */
size_t dmx_write(dmx_port_t dmx_num, const void *source, size_t size);

/**
 * @brief Writes a single slot of DMX data.
 *
 * @param dmx_num The DMX port number.
 * @param slot_num The DMX slot number to write.
 * @return The value written to the DMX slot or -1 on error.
 */
int dmx_write_slot(dmx_port_t dmx_num, size_t slot_num, uint8_t value);

// TODO:
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
 * @brief Receives a DMX packet from the DMX bus. This is a blocking function.
 * This function first blocks until the DMX driver is idle and then it blocks
 * using a timeout until a new packet is received. This function will timeout
 * early according to RDM specification if an RDM packet is expected.
 *
 * @note This function uses FreeRTOS direct-to-task notifications to block and
 * unblock. Using task notifications on the same task that calls this function
 * can lead to undesired behavior and program instability.
 *
 * @param dmx_num The DMX port number.
 * @param[out] packet An optional pointer to a dmx_packet_t which contains
 * information about the received DMX packet.
 * @param wait_ticks The number of ticks to wait before this function times out.
 * @return The size of the received DMX packet or 0 if no packet was received.
 */
size_t dmx_receive(dmx_port_t dmx_num, dmx_packet_t *packet,
                   TickType_t wait_ticks);

/**
 * @brief Sends a DMX packet on the DMX bus. This function blocks until the DMX
 * driver is idle and then sends a packet.
 *
 * @note This function uses FreeRTOS direct-to-task notifications to block and
 * unblock. Using task notifications on the same task that calls this function
 * can lead to undesired behavior and program instability.
 *
 * @param dmx_num The DMX port number.
 * @param size The size of the packet to send. If 0, sends a full DMX packet. If
 * an RDM packet was written, this value is ignored.
 * @return The number of bytes sent on the DMX bus.
 */
size_t dmx_send_num(dmx_port_t dmx_num, size_t size);

/**
 * @brief Sends a DMX packet on the DMX bus. This function blocks until the DMX
 * driver is idle and then sends a packet. Calling this function is the same as
 * calling dmx_send_num(dmx_num, 0).
 *
 * @note This function uses FreeRTOS direct-to-task notifications to block and
 * unblock. Using task notifications on the same task that calls this function
 * can lead to undesired behavior and program instability.
 *
 * @param dmx_num The DMX port number.
 * @return The number of bytes sent on the DMX bus.
 */
size_t dmx_send(dmx_port_t dmx_num);

/**
 * @brief Waits until the DMX packet is done being sent. This function can be
 * used to ensure that calls to dmx_write() happen synchronously with the
 * current DMX frame.
 *
 * @note This function uses FreeRTOS direct-to-task notifications to block and
 * unblock. Using task notifications on the same task that calls this function
 * can lead to undesired behavior and program instability.
 *
 * @param dmx_num The DMX port number.
 * @param wait_ticks The number of ticks to wait before this function times out.
 * @retval true if the DMX driver is done sending.
 * @retval false if the function timed out.
 */
bool dmx_wait_sent(dmx_port_t dmx_num, TickType_t wait_ticks);

#ifdef __cplusplus
}
#endif
