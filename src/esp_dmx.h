#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "dmx_caps.h"
#include "dmx_types.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/gpio_types.h"

#define DMX_NUM_0 (0)  // DMX port 0.
#define DMX_NUM_1 (1)  // DMX port 1.
#if UART_NUM_MAX > 2
#define DMX_NUM_2 (2)  // DMX port 2.
#endif
#define DMX_NUM_MAX UART_NUM_MAX  // DMX port max.

// Constant for dmx_set_pin() which indicates the pin should not be changed.
#define DMX_PIN_NO_CHANGE (UART_PIN_NO_CHANGE)

/**
 * @brief The default configuration for DMX. This macro may be used to
 * initialize a dmx_config_t to the standard's defined typical values.
 */
#define DMX_DEFAULT_CONFIG \
  { .baud_rate = DMX_TYP_BAUD_RATE, .break_num = 44, .idle_num = 3, }

/// Driver Functions  #########################################################
/**
 * @brief Install DMX driver and set the DMX to the default configuration. DMX
 * ISR handler will be attached to the same CPU core that this function is
 * running on.
 *
 * @param dmx_num The DMX port number.
 * @param buffer_size The size of the DMX driver send and receive buffer.
 * @param queue_size The size of the DMX event queue.
 * @param[in] dmx_queue Handle to the event queue.
 * @param intr_alloc_flags Interrupt allocation flags as specified in
 * esp_intr_alloc.h.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there is an argument error.
 * @retval ESP_ERR_NO_MEM if there is not enough memory.
 * @retval ESP_ERR_INVALID_STATE if the driver already installed.
 * */
esp_err_t dmx_driver_install(dmx_port_t dmx_num, uint16_t buffer_size, 
                             uint32_t queue_size, QueueHandle_t *dmx_queue, 
                             int intr_alloc_flags);

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
 * @brief Sets the DMX mode, either DMX_MODE_READ or DMX_MODE_WRITE.
 *
 * @param dmx_num The DMX port number.
 * @param dmx_mode The mode that the DMX driver will be set to.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver is not installed.
 */
esp_err_t dmx_set_mode(dmx_port_t dmx_num, dmx_mode_t dmx_mode);

/**
 * @brief Gets the DMX mode.
 *
 * @param dmx_num The DMX port number.
 * @param[out] dmx_mode A pointer to a dmx_mode_t to return the current mode.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver is not installed.
 */
esp_err_t dmx_get_mode(dmx_port_t dmx_num, dmx_mode_t *dmx_mode);

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
 * @brief Set DMX configuration parameters.
 *
 * @param dmx_num The DMX port number.
 * @param[in] dmx_config A pointer to a dmx_config_t structure to assign
 * configuration parameters.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_param_config(dmx_port_t dmx_num, const dmx_config_t *dmx_config);

/**
 * @brief Set the DMX baud rate.
 *
 * @param dmx_num The DMX port number.
 * @param baud_rate The baud rate to set the DMX port to.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate);

/**
 * @brief Get the DMX baud rate.
 *
 * @param dmx_num The DMX port number.
 * @param[out] baud_rate The baud rate returned from the DMX configuration.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_get_baud_rate(dmx_port_t dmx_num, uint32_t *baud_rate);

/**
 * @brief Set the DMX packet break time.
 *
 * @param dmx_num The DMX port number.
 * @param break_num The break number to set the UART hardware to.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_set_break_num(dmx_port_t dmx_num, uint8_t break_num);

/**
 * @brief Get the DMX break time.
 *
 * @param dmx_num The DMX port number.
 * @param break_num The currently configured break number returned from the
 * UART hardware.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_get_break_num(dmx_port_t dmx_num, uint8_t *break_num);

/**
 * @brief Set the DMX idle time. The idle time is equivalent to mark after
 * break.
 *
 * @note In hardware, the idle num is stored as a 10-bit number. Passing any
 * idle_num larger than 1023 will result in a parameter error.
 *
 * @param dmx_num The DMX port number.
 * @param idle_num The value to set the idle number to.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_set_idle_num(dmx_port_t dmx_num, uint16_t idle_num);

/**
 * @brief Get the DMX idle time. The idle time is equivalent to mark after
 * break.
 *
 * @param dmx_num The DMX port number.
 * @param idle_num The idle number currently configured in the UART hardware.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_get_idle_num(dmx_port_t dmx_num, uint16_t *idle_num);

/// Interrupt Configuration  ##################################################
/**
 * @brief Configure DMX interrupts.
 *
 * @param dmx_num The DMX port number.
 * @param intr_conf A pointer to a dmx_intr_config_t to configure the UART
 * hardware interrupts.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_intr_config(dmx_port_t dmx_num,
                          const dmx_intr_config_t *intr_conf);

/**
 * @brief Configure DMX rx full interrupt threshold.
 *
 * @param dmx_num The DMX port number.
 * @param threshold The threshold value to set the UART hardware to. This is
 * the number of bytes that must be in the UART RX FIFO for the FIFO to be
 * "full."
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_set_rx_full_threshold(dmx_port_t dmx_num, int threshold);

/**
 * @brief Configure DMX tx empty interrupt threshold.
 *
 * @param dmx_num The DMX port number.
 * @param threshold The threshold value to set the UART hardware to. This is
 * the number of bytes or fewer that must be in the UART TX FIFO for it to
 * be 'empty.'
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_set_tx_empty_threshold(dmx_port_t dmx_num, int threshold);

/**
 * @brief Configure DMX rx timeout interrupt threshold.
 *
 * @param dmx_num The DMX port number.
 * @param timeout The timeout threshold for the UART FIFO. This is the
 * amount of time that must pass without receiving data for the UART to
 * timeout. The unit of time is the time it takes for the UART to receive
 * 1 byte of data.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 */
esp_err_t dmx_set_rx_timeout(dmx_port_t dmx_num, uint8_t timeout);

/// Read/Write  ###############################################################
/**
 * @brief Read data from the DMX driver.
 *
 * @note This function is not synchronous with the DMX frame.
 *
 * @param dmx_num The DMX port number.
 * @param[out] buffer The buffer that will be read into from the DMX driver.
 * @param size The size of the receiving buffer.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver was not installed.
 * @retval ESP_FAIL on driver error.
 */
esp_err_t dmx_read_packet(dmx_port_t dmx_num, void *buffer, uint16_t size);

/**
 * @brief Reads a slot value from the DMX bus.
 *
 * @param dmx_num The DMX port number.
 * @param slot_idx The index of the slot to be read from.
 * @param[out] value A pointer to the byte that will store the value read.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver was not installed.
 */
esp_err_t dmx_read_slot(dmx_port_t dmx_num, uint16_t slot_idx, uint8_t *value);

/**
 * @brief Write data to the DMX driver from a given buffer and length.
 *
 * @note This function is not synchronous with the DMX frame.
 *
 * @param dmx_num The DMX port number.
 * @param[in] buffer The buffer that will be written to the DMX driver.
 * @param size The size of the buffer that will be written to the DMX driver.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver was not installed.
 * @retval ESP_FAIL on driver error.
 */
esp_err_t dmx_write_packet(dmx_port_t dmx_num, const void *buffer,
                           uint16_t size);

/**
 * @brief Write a slot value to the DMX bus.
 *
 * @param dmx_num The DMX port number.
 * @param slot_idx The index of the slot to be read from.
 * @param value The byte value that will be written to the DMX bus.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver was not installed.
 */
esp_err_t dmx_write_slot(dmx_port_t dmx_num, uint16_t slot_idx,
                         const uint8_t value);

/**
 * @brief Transmits a packet of DMX. This sends the number of slots as was
 * declared in dmx_driver_install().
 *
 * @param dmx_num The DMX port number.
 * @param num_slots The number of slots to transmit.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver was not installed.
 * @retval ESP_FAIL if a packet is already being sent.
 */
esp_err_t dmx_send_packet(dmx_port_t dmx_num, uint16_t num_slots);

/**
 * @brief Wait until the DMX port is done transmitting. This function blocks
 * the current task until the DMX port is finished with transmission.
 *
 * @param dmx_num The DMX port number.
 * @param ticks_to_wait The number of FreeRTOS ticks to wait.
 * @retval ESP_OK on success.
 * @retval ESP_ERR_INVALID_ARG if there was an argument error.
 * @retval ESP_ERR_INVALID_STATE if the driver was not installed.
 * @retval ESP_ERR_TIMEOUT on timeout.
 */
esp_err_t dmx_wait_send_done(dmx_port_t dmx_num, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif
