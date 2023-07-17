#pragma once


#include "dmx/types.h"
#include "rdm/types.h"
#include "dmx/hal.h"

#ifdef __cplusplus
extern "C" {
#endif 

/**
 * @brief Checks if the DMX driver is enabled. When the DMX driver is not placed
 * in IRAM, functions which disable the cache, such as functions which read or
 * write to flash, will also stop DMX interrupts from firing. This can cause
 * incoming DMX data to become corrupted. To avoid this issue, the DMX driver
 * should be disabled before disabling the cache. When cache is reenabled, the
 * DMX driver can be reenabled as well. When the DMX driver is placed in IRAM,
 * disabling and reenabling the DMX driver is not needed.
 *
 * @param dmx_num The DMX port number.
 * @retval true if the driver is enabled.
 * @retval false if the driver is disabled.
 */
bool dmx_driver_is_enabled(dmx_port_t dmx_num);

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
 * @brief Gets the DMX break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX break length or 0 on error.
 */
uint32_t dmx_get_break_len(dmx_port_t dmx_num);

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

/**
 * @brief Gets the DMX mark-after-break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX mark-after-break length or 0 on error.
 */
uint32_t dmx_get_mab_len(dmx_port_t dmx_num);

/**
 * @brief Gets the current personality of the DMX driver.
 * 
 * @param dmx_num The DMX port number.
 * @return The current personality or 0 on failure.
 */
uint8_t dmx_get_current_personality(dmx_port_t dmx_num);

/**
 * @brief Sets the current personality of the DMX driver. Personalities are
 * indexed starting at 1.
 *
 * @param dmx_num The DMX port number.
 * @param personality_num The personality number to which to set the DMX driver.
 * Personality number are indexed starting at 1.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_set_current_personality(dmx_port_t dmx_num, uint8_t personality_num);

/**
 * @brief Gets the personality count of the DMX driver.
 * 
 * @param dmx_num The DMX port number.
 * @return The personality count or 0 on failure.
 */
uint8_t dmx_get_personality_count(dmx_port_t dmx_num);

/**
 * @brief Gets the footprint of the specified personality.
 * 
 * @param dmx_num The DMX port number.
 * @param personality_num The personality number of the footprint to get.
 * Personality numbers are indexed starting at 1.
 * @return The footprint of the specified personality or 0 on failure.
 */
size_t dmx_get_footprint(dmx_port_t dmx_num, uint8_t personality_num);

/**
 * @brief Gets the description of the specified personality.
 * 
 * @param dmx_num The DMX port number.
 * @param personality_num The personality number of the description to get.
 * Personality numbers are indexed starting at 1.
 * @return The description of the DMX personality or NULL on failure.
 */
const char *dmx_get_personality_description(dmx_port_t dmx_num,
                                            uint8_t personality_num);

/**
 * @brief Gets the DMX start address of the DMX driver.
 * 
 * @param dmx_num The DMX port number.
 * @return The DMX start address of the DMX driver or 0 on failure.
 */
uint16_t dmx_get_start_address(dmx_port_t dmx_num);


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
size_t dmx_read_rdm(dmx_port_t dmx_num, rdm_header_t *header, void *pd,
                    size_t num);


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
size_t dmx_write_rdm(dmx_port_t dmx_num, rdm_header_t *header, const void *pd);


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

/**
 * @brief Checks if the sniffer is enabled.
 *
 * @param dmx_num The DMX port number.
 * @retval true if the sniffer is installed.
 * @retval false if the sniffer is not installed or DMX port does not exist.
 */
bool dmx_sniffer_is_enabled(dmx_port_t dmx_num);

/**
 * @brief Gets sniffer data if it is available.
 *
 * @param dmx_num The DMX port number.
 * @param[out] metadata A pointer to a dmx_metadata_t struct into which to
 * copy DMX sniffer data.
 * @param wait_ticks The number of ticks to wait before this function times out.
 * @return true if data was copied.
 * @return false if data was not copied.
 */
bool dmx_sniffer_get_data(dmx_port_t dmx_num, dmx_metadata_t *metadata,
                          TickType_t wait_ticks);

#ifdef __cplusplus
}
#endif 


