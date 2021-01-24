#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "dmx_caps.h"
#include "dmx_types.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define DMX_NUM_0                   0             // DMX port 0.
#define DMX_NUM_1                   1             // DMX port 1.
#if SOC_DMX_NUM > 2
#define DMX_NUM_2                   2             // DMX port 2.
#endif
#define DMX_NUM_MAX                 SOC_DMX_NUM   // Number of available DMX ports.

/// Driver Functions  #########################################################
/**
 * @brief Install DMX driver and set the DMX to the default configuration.
 *
 * DMX ISR handler will be attached to the same CPU core that this function is
 * running on.
 *
 * @param dmx_num
 * @param buf_size
 * @param queue_size
 * @param dmx_queue
 * @param intr_alloc_flags
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 *  - ESP_ERR_NO_MEM        Not enough memory
 *  - ESP_ERR_INVALID_STATE Driver already installed
 * */
esp_err_t dmx_driver_install(dmx_port_t dmx_num, int buf_size,
    int queue_size, QueueHandle_t *dmx_queue, int intr_alloc_flags);

/**
 * @brief Uninstall DMX driver.
 *
 * @param dmx_num
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 * */
esp_err_t dmx_driver_delete(dmx_port_t dmx_num);

/**
 * @brief Checks if DMX driver is installed.
 *
 * @param dmx_num
 * @return
 *  - true  Driver is installed
 *  - false Driver is not installed
 * */
bool dmx_is_driver_installed(dmx_port_t dmx_num);

/**
 * @brief Sets the DMX mode, either DMX_MODE_RX or DMX_MODE_TX.
 * 
 * @param dmx_num 
 * @param dmx_mode 
 * @return 
 * - ESP_OK                 Success
 * - ESP_ERR_INVALID_ARG    Parameter error
 * - ESP_ERR_INVALID_STATE  Driver not installed
 */
esp_err_t dmx_set_mode(dmx_port_t dmx_num, dmx_mode_t dmx_mode);

/**
 * @brief Gets the DMX mode, either DMX_MODE_RX, or DMX_MODE_TX.
 * 
 * @param dmx_num 
 * @param dmx_mode 
 * @return
 * - ESP_OK                 Success
 * - ESP_ERR_INVALID_ARG    Parameter error
 * - ESP_ERR_INVALID_STATE  Driver not installed 
 */
esp_err_t dmx_get_mode(dmx_port_t dmx_num, dmx_mode_t *dmx_mode);

/**
 * @brief Enable the DMX rx analysis tool to determine the break and 
 * mark-after-break length.
 * 
 * @note The analysis tool uses the default GPIO ISR handler, which allows for
 * many ISRs to be registered to different GPIO pins. Depending on how many 
 * GPIO interrupts are registered, there could be significant latency between
 * when the analyzer ISR runs and when an ISR condition actually occurs. A 
 * quirk of this implementation is that ISRs are handled from lowest GPIO
 * number to highest. It is therefore recommended that the user shorts the UART
 * rx pin to the lowest numbered GPIO possible and enables the rx analyzer 
 * interrupt on that pin to ensure that the analyzer ISR is called with the
 * lowest latency possible.
 * 
 * @param dmx_num 
 * @param intr_io_num 
 * @return
 * - ESP_OK                 Success
 * - ESP_ERR_INVALID_ARG    Parameter error
 * - ESP_ERR_INVALID_STATE  Driver not installed, no queue, or already enabled
 */
esp_err_t dmx_rx_timing_enable(dmx_port_t dmx_num, int intr_io_num);

/**
 * @brief Disable the DMX analysis tool.
 * 
 * @param dmx_num 
 * @return
 * - ESP_OK                 Success
 * - ESP_ERR_INVALID_ARG    Parameter error
 * - ESP_ERR_INVALID_STATE  Driver not installed, or already disabled 
 */
esp_err_t dmx_rx_timing_disable(dmx_port_t dmx_num);

/// Hardware Configuration  ###################################################
/**
 * @brief Set DMX pin number.
 *
 * @param dmx_num
 * @param tx_io_num
 * @param rx_io_num
 * @param rts_io_num
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 * */
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_io_num, int rx_io_num,
    int rts_io_num);

/**
 * @brief Set DMX configuration parameters.
 * 
 * @param dmx_num 
 * @param dmx_config 
 * @return 
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_param_config(dmx_port_t dmx_num, const dmx_config_t *dmx_config);

/**
 * @brief Set the DMX baudrate.
 * 
 * @param dmx_num 
 * @param baudrate 
 * @return 
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_set_baudrate(dmx_port_t dmx_num, uint32_t baudrate);

/**
 * @brief Get the DMX baudrate.
 * 
 * @param dmx_num 
 * @param baudrate 
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_get_baudrate(dmx_port_t dmx_num, uint32_t *baudrate);

/**
 * @brief Set the DMX break time.
 * 
 * @param dmx_num 
 * @param break_num 
 * @return 
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_set_break_num(dmx_port_t dmx_num, uint8_t break_num);

/**
 * @brief Get the DMX break time.
 * 
 * @param dmx_num 
 * @param break_num 
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_get_break_num(dmx_port_t dmx_num, uint8_t *break_num);

/**
 * @brief Set the DMX idle time. The idle time is equivalent to mark after break.
 * 
 * @note In hardware, the idle num is stored as a 10-bit number. Passing any
 * idle_num larger than 1023 will result in a parameter error.
 * 
 * @param dmx_num 
 * @param idle_num 
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_set_idle_num(dmx_port_t dmx_num, uint16_t idle_num);

/**
 * @brief Get the DMX idle time. The idle time is equivalent to mark after break.
 * 
 * @param dmx_num 
 * @param idle_num 
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_get_idle_num(dmx_port_t dmx_num, uint16_t *idle_num);

/**
 * @brief Invert or un-invert the RTS line.
 * 
 * @param dmx_num 
 * @param invert 
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_invert_rts(dmx_port_t dmx_num, bool invert);

/// Interrupt Configuration  ##################################################
/**
 * @brief Configure DMX interrupts.
 *
 * @param dmx_num
 * @param intr_conf
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t dmx_intr_config(dmx_port_t dmx_num, const dmx_intr_config_t* intr_conf);

/**
 * @brief Configure DMX rx full interrupt threshold.
 * 
 * @param dmx_num 
 * @param threshold 
 * @return 
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t dmx_set_rx_full_threshold(dmx_port_t dmx_num, int threshold);

/**
 * @brief Configure DMX tx empty interrupt threshold.
 * 
 * @param dmx_num 
 * @param threshold 
 * @return 
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t dmx_set_tx_empty_threshold(dmx_port_t dmx_num, int threshold);

/**
 * @brief Configure DMX rx timeout interrupt threshold.
 * 
 * @param dmx_num 
 * @param tout_thresh 
 * @return 
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t dmx_set_rx_timeout(dmx_port_t dmx_num, uint8_t tout_thresh);

/// Read/Write  ###############################################################
/**
 * @brief Wait until the DMX port is done transmitting.
 * 
 * @param dmx_num 
 * @param ticks_to_wait 
 * @return
 * - ESP_OK                 Success
 * - ESP_ERR_INVALID_ARG    Parameter error
 * - ESP_ERR_INVALID_STATE  Driver not installed
 * - ESP_ERR_TIMEOUT        Timed out
 */
esp_err_t dmx_wait_tx_done(dmx_port_t dmx_num, TickType_t ticks_to_wait);

/**
 * @brief Transmits a frame of DMX on the UART bus.
 * 
 * @param dmx_num 
 * @return 
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error 
 */
esp_err_t dmx_tx_frame(dmx_port_t dmx_num);

/**
 * @brief Send data to the DMX driver from a given buffer and length.
 * 
 * @note This function is not synchronous with the DMX frame.
 * 
 * @param dmx_num 
 * @param frame_buffer 
 * @param length 
 * @return  
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error  
 *  - ESP_ERR_INVALID_STATE Driver not installed
 *  - ESP_FAIL              Driver error
 */
esp_err_t dmx_write_frame(dmx_port_t dmx_num, const uint8_t *frame_buffer, uint16_t length);

/**
 * @brief Read data from the DMX driver.
 * 
 * @note This function is not synchronous with the DMX frame.
 * 
 * @param dmx_num 
 * @param frame_buffer 
 * @param length 
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error  
 *  - ESP_ERR_INVALID_STATE Driver not installed
 *  - ESP_FAIL              Driver error
 */
esp_err_t dmx_read_frame(dmx_port_t dmx_num, uint8_t *frame_buffer, uint16_t length);

// TODO:
esp_err_t dmx_write_slot(dmx_port_t dmx_num, int slot_idx, uint8_t value);
esp_err_t dmx_read_slot(dmx_port_t dmx_num, int slot_idx, uint8_t *value);

#ifdef __cplusplus
}
#endif
