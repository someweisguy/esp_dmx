#pragma once

#include "dmx_constants.h"
#include "dmx_types.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "soc/soc_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Driver Functions  #########################################################
/**
 * @brief Install DMX driver and set the DMX to the default configuration. DMX
 * ISR handler will be attached to the same CPU core that this function is
 * running on. The default configuration sets the DMX break to 176 microseconds
 * and the DMX mark-after-break to 12 microseconds.
 *
 * @param dmx_num The DMX port number.
 * @param[in] dmx_config A pointer to a dmx_config_t.
 * @param queue_size The size of the DMX event queue.
 * @param[in] dmx_queue Handle to the event queue.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there is an argument error.
 * @retval ESP_ERR_NO_MEM if there is not enough memory.
 * @retval ESP_ERR_INVALID_STATE if the driver already installed.
 * */
esp_err_t dmx_driver_install(dmx_port_t dmx_num, dmx_config_t *dmx_config);

/**
 * @brief Uninstall the DMX driver.
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
bool dmx_is_driver_installed(dmx_port_t dmx_num);

/**
 * @brief Enable the DMX sniffer to determine the break and mark-after-break
 * length.
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
 * @param intr_io_num The pin to assign the to which to assign the interrupt.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver is not installed, no queue, or
 * sniffer already enabled.
 */
esp_err_t dmx_sniffer_enable(dmx_port_t dmx_num, int intr_io_num);

/**
 * @brief Disable the DMX sniffer.
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
bool dmx_is_sniffer_enabled(dmx_port_t dmx_num);

/// Hardware Configuration  ###################################################
/**
 * @brief Set DMX pin number.
 *
 * @param dmx_num The DMX port number.
 * @param tx_io_num The pin to which the TX signal will be assigned.
 * @param rx_io_num The pin to which the RX signal will be assigned.
 * @param rts_io_num The pin to which the RTS signal will be assigned.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * */
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_io_num, int rx_io_num,
                      int rts_io_num);

/**
 * @brief Set the DMX baud rate. The baud rate will be clamped to DMX 
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
 * @brief Get the DMX baud rate.
 *
 * @param dmx_num The DMX port number.
 * @return the current baud rate or 0 on error.
 */
size_t dmx_get_baud_rate(dmx_port_t dmx_num);

/**
 * @brief Set the DMX break length in microseconds. The break length will be 
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
 * @brief Get the DMX break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX break length or 0 on error.
 */
size_t dmx_get_break_len(dmx_port_t dmx_num);

/**
 * @brief Set the DMX mark-after-break length in microseconds. The 
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
 * @brief Get the DMX mark-after-break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX mark-after-break length or 0 on error.
 */
size_t dmx_get_mab_len(dmx_port_t dmx_num);

/// Read/Write  ###############################################################
// TODO: docs
size_t dmx_read(dmx_port_t dmx_num, void *destination, size_t size);

// TODO: docs
size_t dmx_write(dmx_port_t dmx_num, const void *source, size_t size);

// TODO: docs
size_t dmx_receive(dmx_port_t dmx_num, dmx_event_t *event,
                 TickType_t ticks_to_wait);

// TODO: docs
size_t dmx_send(dmx_port_t dmx_num, size_t size, TickType_t ticks_to_wait);

// TODO: docs
bool dmx_wait_sent(dmx_port_t dmx_num, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif
