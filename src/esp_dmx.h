/**
 * @file esp_dmx.h
 * @author Mitch Weisbrod
 * @brief This is the main header file for esp_dmx. This file declares functions
 * needed for installing the DMX driver and sending or receiving DMX data. It is
 * possible to implement RDM using the functions found in this header file
 * alone. However, RDM can be complex to users who aren't familiar with the 
 * standard. Functions found in rdm_tools.h can be used to simplify basic RDM
 * tasks.
 */
#pragma once

#include "dmx_constants.h"
#include "dmx_types.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Installs the DMX driver and sets the default configuration. To
 * generate the DMX reset sequence, users may choose to use either the hardware
 * timers or busy-waiting. The default configuration sets the DMX break to 176 
 * microseconds and the DMX mark-after-break to 12 microseconds. 
 * 
 * @note The DMX interrupt service routine is installed on the same CPU core
 * that this function is running on. 
 *
 * @param dmx_num The DMX port number.
 * @param[in] dmx_config A pointer to a dmx_config_t.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there is an argument error.
 * @retval ESP_ERR_NO_MEM if there is not enough memory.
 * @retval ESP_ERR_INVALID_STATE if the driver already installed.
 * */
esp_err_t dmx_driver_install(dmx_port_t dmx_num, dmx_config_t *dmx_config);

/**
 * @brief Uninstalls the DMX driver.
 *
 * @param dmx_num The DMX port number
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there is an argument error.
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
 * @brief Sets DMX pin number.
 *
 * @param dmx_num The DMX port number.
 * @param tx_num The pin to which the TX signal will be assigned.
 * @param rx_num The pin to which the RX signal will be assigned.
 * @param rts_num The pin to which the RTS signal will be assigned.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * */
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_num, int rx_num, int rts_num);

/**
 * @brief Enables the DMX sniffer to determine the DMX break and 
 * mark-after-break length.
 *
 * @note The sniffer uses the default GPIO ISR handler, which allows for many
 * ISRs to be registered to different GPIO pins. Depending on how many GPIO
 * interrupts are registered, there could be significant latency between when
 * the analyzer ISR runs and when an ISR condition actually occurs. A quirk of
 * this implementation is that ISRs are handled from lowest GPIO number to
 * highest. It is therefore recommended that the user shorts the UART rx pin to
 * the lowest numbered GPIO possible and enables the sniffer interrupt on that
 * pin to ensure that the analyzer ISR is called with the lowest latency
 * possible.
 *
 * @param dmx_num The DMX port number.
 * @param intr_num The pin to which to assign the interrupt.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver is not installed or sniffer 
 * already enabled.
 */
esp_err_t dmx_sniffer_enable(dmx_port_t dmx_num, int intr_num);

/**
 * @brief Disables the DMX sniffer.
 *
 * @param dmx_num The DMX port number.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver is not installed, no queue, or
 * sniffer already disabled.
 */
esp_err_t dmx_sniffer_disable(dmx_port_t dmx_num);

/**
 * @brief Checks if the sniffer is enabled.
 *
 * @param dmx_num The DMX port number.
 * @retval true if the sniffer is installed.
 * @retval false if the sniffer is not installed or DMX port does not exist.
 */
bool dmx_sniffer_is_enabled(dmx_port_t dmx_num);

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
size_t dmx_set_baud_rate(dmx_port_t dmx_num, size_t baud_rate);

/**
 * @brief Gets the DMX baud rate.
 *
 * @param dmx_num The DMX port number.
 * @return the current baud rate or 0 on error.
 */
size_t dmx_get_baud_rate(dmx_port_t dmx_num);

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
size_t dmx_set_break_len(dmx_port_t dmx_num, size_t break_len);

/**
 * @brief Gets the DMX break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX break length or 0 on error.
 */
size_t dmx_get_break_len(dmx_port_t dmx_num);

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
size_t dmx_set_mab_len(dmx_port_t dmx_num, size_t mab_len);

/**
 * @brief Gets the DMX mark-after-break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX mark-after-break length or 0 on error.
 */
size_t dmx_get_mab_len(dmx_port_t dmx_num);

/**
 * @brief Reads DMX data from the driver into a destination buffer.
 * 
 * @param dmx_num The DMX port number.
 * @param destination The destination buffer into which to read the DMX data.
 * @param size The size of the destination buffer.
 * @return The number of bytes read from the DMX driver.
 */
size_t dmx_read(dmx_port_t dmx_num, void *destination, size_t size);

/**
 * @brief Writes DMX data from a source buffer into the DMX driver buffer. Data
 * written into the DMX driver buffer can then be sent to DMX devices.
 * 
 * @param dmx_num The DMX port number.
 * @param source The source buffer which is copied to the DMX driver.
 * @param size The size of the source buffer.
 * @return The number of bytes written into the DMX driver.
 */
size_t dmx_write(dmx_port_t dmx_num, const void *source, size_t size);

/**
 * @brief Receives a DMX packet from the DMX bus. This is a blocking function.
 * This function first blocks indefinitely until the DMX driver is idle and then 
 * it blocks using a timeout until a packet is received. This function will 
 * timeout early according to RDM specification if an RDM packet is expected.
 * 
 * @note This function uses FreeRTOS direct-to-task notifications to block and
 * unblock. Using task notifications on the same task that calls this function
 * can lead to undesired behavior and program instability.
 * 
 * @param dmx_num The DMX port number.
 * @param[out] event A pointer to a dmx_event_t which contains information about
 * the received DMX packet.
 * @param timeout The number of ticks to wait before this function times out.
 * @return The size of the received DMX packet or 0 if no packet was received. 
 */
size_t dmx_receive(dmx_port_t dmx_num, dmx_event_t *event, TickType_t timeout);

/**
 * @brief Sends a DMX packet on the DMX bus. This function blocks indefinitely
 * until the DMX driver is idle and then it sends a packet. The size of the
 * packet is equal to the size passed to the previous call to dmx_write().
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
 * @param timeout The number of ticks to wait before this function times out.
 * @retval true if the DMX driver is done sending.
 * @retval false if the function timed out.
 */
bool dmx_wait_sent(dmx_port_t dmx_num, TickType_t timeout);

#ifdef __cplusplus
}
#endif
