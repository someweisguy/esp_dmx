/**
 * @file esp_dmx.h
 * @author Mitch Weisbrod
 * @brief This is the main header file for esp_dmx. This file declares functions
 * needed for installing the DMX driver and sending or receiving DMX data. It is
 * possible to implement RDM using the functions found in this header file
 * alone. However, RDM can be complex to users who aren't familiar with the
 * standard. Functions found in rdm/agent.h and rdm/requests.h can be used to
 * simplify basic RDM tasks.
 */
#pragma once

#include "dmx/types.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @brief The major version number of this library. (X.x.x)*/
#define ESP_DMX_VERSION_MAJOR 3

/** @brief The minor version number of this library. (x.X.x)*/
#define ESP_DMX_VERSION_MINOR 0

/** @brief The patch version number of this library. (x.x.X)*/
#define ESP_DMX_VERSION_PATCH 3

/** @brief The default configuration for the DMX driver. Passing this
 * configuration to dmx_driver_install() installs the driver with one DMX
 * personality which has a footprint of one DMX address. The DMX address will
 * automatically be searched for in NVS and set to 1 if not found or if NVS is
 * disabled. */
#define DMX_CONFIG_DEFAULT                                                    \
  (dmx_config_t) {                                                            \
    .alloc_size = 255, .model_id = 0, .product_category = 0x0100,             \
    .software_version_id = ESP_IDF_VERSION_VAL(                               \
        ESP_DMX_VERSION_MAJOR, ESP_DMX_VERSION_MINOR, ESP_DMX_VERSION_PATCH), \
    .software_version_label =                                                 \
        "esp_dmx v" __XSTRING(ESP_DMX_VERSION_MAJOR) "." __XSTRING(           \
            ESP_DMX_VERSION_MINOR) "." __XSTRING(ESP_DMX_VERSION_PATCH),      \
    .current_personality = 1, .personalities = {{1, "Default Personality"}},  \
    .personality_count = 1, .dmx_start_address = 0                            \
  }

// FIXME: use definition in dmx/driver.h
#ifdef CONFIG_DMX_ISR_IN_IRAM
/** @brief The default interrupt flags for the DMX driver. Places the interrupts
 * in IRAM.*/
#define DMX_INTR_FLAGS_DEFAULT (ESP_INTR_FLAG_IRAM)
#else
/** @brief The default interrupt flags for the DMX driver.*/
#define DMX_INTR_FLAGS_DEFAULT (0)
#endif

/**
 * @brief Installs the DMX driver and sets the default configuration. To
 * generate the DMX reset sequence, users may choose to use either the hardware
 * timers or busy-waiting. The default configuration sets the DMX break to 176
 * microseconds and the DMX mark-after-break to 12 microseconds.
 *
 * @note By default, the DMX driver will allocate a hardware timer for the DMX
 * driver to use. When using ESP-IDF v4.4 the DMX driver will allocate a
 * hardware timer group and timer relative to the DMX port number. The function
 * to determine which timer group and number to use is
 * timer_group == (dmx_num / 2) and timer_num == (dmx_num % 2). It is not
 * recommended to use the hardware timer that the DMX driver is using while the
 * DMX driver is installed. On the ESP32-C3, hardware timer number 0 will always
 * be used to avoid clobbering the watchdog timer.
 *
 * @note The DMX interrupt service routine is installed on the same CPU core
 * that this function is running on.
 *
 * @param dmx_num The DMX port number.
 * @param[in] config A pointer to a DMX configuration which will be used to
 * setup the DMX driver.
 * @param intr_flags The interrupt allocation flags to use.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there is an argument error.
 * @retval ESP_ERR_NO_MEM if there is not enough memory.
 * @retval ESP_ERR_INVALID_STATE if the driver already installed.
 * */
esp_err_t dmx_driver_install(dmx_port_t dmx_num, const dmx_config_t *config,
                             int intr_flags);

/**
 * @brief Uninstalls the DMX driver.
 *
 * @param dmx_num The DMX port number
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there is an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver not installed.
 */
esp_err_t dmx_driver_delete(dmx_port_t dmx_num);

/**
 * @brief Checks if DMX driver is installed.
 *
 * @param dmx_num The DMX port number.
 * @retval true if the driver is installed.
 * @retval false if the driver is not installed or DMX port does not exist.
 * */
bool dmx_driver_is_installed(dmx_port_t dmx_num);

/**
 * @brief Disables the DMX driver. When the DMX driver is not placed in IRAM,
 * functions which disable the cache, such as functions which read or write to
 * flash, will also stop DMX interrupts from firing. This can cause incoming DMX
 * data to become corrupted. To avoid this issue, the DMX driver should be
 * disabled before disabling the cache. When cache is reenabled, the DMX driver
 * can be reenabled as well. When the DMX driver is placed in IRAM, disabling
 * and reenabling the DMX driver is not needed.
 *
 * @param dmx_num The DMX port number.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there is an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver is not installed or already
 * disabled.
 * @retval ESP_ERR_NOT_FINISHED if the driver is currently sending data.
 */
esp_err_t dmx_driver_disable(dmx_port_t dmx_num);

/**
 * @brief Enables the DMX driver. When the DMX driver is not placed in IRAM,
 * functions which disable the cache, such as functions which read or write to
 * flash, will also stop DMX interrupts from firing. This can cause incoming DMX
 * data to become corrupted. To avoid this issue, the DMX driver should be
 * disabled before disabling the cache. When cache is reenabled, the DMX driver
 * can be reenabled as well. When the DMX driver is placed in IRAM, disabling
 * and reenabling the DMX driver is not needed.
 *
 * @param dmx_num The DMX port number.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there is an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver is not installed or already
 * enabled.
 */
esp_err_t dmx_driver_enable(dmx_port_t dmx_num);

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
 * @brief Sets DMX pin number.
 *
 * @param dmx_num The DMX port number.
 * @param tx_pin The pin to which the TX signal will be assigned.
 * @param rx_pin The pin to which the RX signal will be assigned.
 * @param rts_pin The pin to which the RTS signal will be assigned.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * */
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_pin, int rx_pin, int rts_pin);

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
 * @brief Gets the DMX baud rate.
 *
 * @param dmx_num The DMX port number.
 * @return the current baud rate or 0 on error.
 */
uint32_t dmx_get_baud_rate(dmx_port_t dmx_num);

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
 * @brief Sets the DMX start address of the DMX driver. The DMX start address
 * cannot be set if it is set to DMX_START_ADDRESS_NONE.
 * 
 * @param dmx_num The DMX port number.
 * @param dmx_start_address The start address at which to set the DMX driver.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_set_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address);

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
 * @brief Sends a DMX packet on the DMX bus. This function blocks indefinitely
 * until the DMX driver is idle and then sends a packet.
 *
 * @note This function uses FreeRTOS direct-to-task notifications to block and
 * unblock. Using task notifications on the same task that calls this function
 * can lead to undesired behavior and program instability.
 *
 * @param dmx_num The DMX port number.
 * @param size The size of the packet to send. If 0, sends the number of bytes
 * equal to the highest slot number that was written or sent in the previous
 * call to dmx_write(), dmx_write_offset(), dmx_write_slot(), or dmx_send().
 * @return The number of bytes sent on the DMX bus.
 */
size_t dmx_send(dmx_port_t dmx_num, size_t size);

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
