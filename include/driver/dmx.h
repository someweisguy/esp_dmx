#pragma once

#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/dmx_types.h"
#include "soc/uart_caps.h"

// Valid DMX port number
#define DMX_NUM_0 0  // DMX port 0
#define DMX_NUM_1 1  // DMX port 1
#if SOC_UART_NUM > 2
#define DMX_NUM_2 2  // DMX port 2
#endif
#define DMX_NUM_MAX SOC_UART_NUM  // DMX port max 

typedef int dmx_port_t;

typedef intr_handle_t dmx_isr_handle_t;

typedef struct {
} dmx_event_t;

typedef struct {
  uint32_t intr_enable_mask;  // UART interrupt enable mask, choose from UART_XXXX_INT_ENA_M under UART_INT_ENA_REG(i), connect with bit-or operator.
  uint8_t rx_timeout_thresh;  // UART timeout interrupt threshold (unit: time of sending one byte).
  uint8_t txfifo_empty_intr_thresh;  // UART TX empty interrupt threshold.
  uint8_t rxfifo_full_thresh;  // UART RX full interrupt threshold.
} dmx_intr_config_t;


/// Driver Functions  #########################################################
/**
 * @brief Install DMX driver and set the DMX to the default configuration.
 *
 * DMX ISR handler will be attached to the same XPU core that this function is
 * running on.
 *
 * @param dmx_num
 * @param buffer_size
 * @param queue_size
 * @param dmx_queue
 * @param intr_alloc_flags
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 *  - ESP_ERR_NO_MEM        Not enough memory
 * */
esp_err_t dmx_driver_install(dmx_port_t dmx_num, int buffer_size,
    int queue_size, QueueHandle_t* dmx_queue, int intr_alloc_flags);

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

/// Hardware Configuration  ###################################################
/**
 * @brief Set DMX pin number.
 *
 * @param dmx_num
 * @param tx_io_num
 * @param rx_io_num
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 * */
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_io_num, int rx_io_num);

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

/**
 * @brief Register DMX interrupt handler (ISR).
 *
 * @note DMX ISR handler will be attached to the same CPU core that this
 * function is running on.
 *
 * @param dmx_num
 * @param fn
 * @param arg
 * @param intr_alloc_flags
 * @param handle
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t dmx_isr_register(dmx_port_t dmx_num, void (*fn)(void*), void* arg,
    int intr_alloc_flags, dmx_isr_handle_t* handle);

/**
 * @brief Free the DMX interrupt handler (ISR).
 * 
 * @param dmx_num 
 * @return esp_err_t 
 */
esp_err_t dmx_isr_free(dmx_port_t dmx_num);