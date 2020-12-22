#pragma once

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "soc/uart_caps.h"

// Valid DMX port number
#define DMX_NUM_0 0                 /* DMX port 0 */
#define DMX_NUM_1 1                 /* DMX port 1 */
#if SOC_UART_NUM > 2
#define DMX_NUM_2 2                 /* DMX port 2 */
#endif
#define DMX_NUM_MAX SOC_UART_NUM    /* DMX port max */

typedef int dmx_port_t;

/**
 * @brief Install DMX driver and set the DMX to the default configuration.
 *
 * DMX ISR handler will be attached to the same XPU core that this function is
 * running on.
 *
 * @return
 *  - ESP_OK    Success
 *  - ESP_FAIL  Parameter error
 * */
esp_err_t dmx_driver_install(dmx_port_t dmx_num, int buffer_size,
    int queue_size, QueueHandle_t *dmx_queue, int intr_alloc_flags);

/**
 * @brief Uninstall DMX driver.
 *
 * @return
 *  - ESP_OK    Success
 *  - ESP_FAIL  Parameter error
 * */
esp_err_t dmx_driver_delete(dmx_port_t dmx_num);

/**
 * @brief Checks if DMX driver is installed.
 *
 * @return
 *  - true  Driver is installed
 *  - false Driver is not installed
 * */
bool dmx_is_driver_installed(dmx_port_t dmx_num);

/**
 * @brief Set DMX pin number.
 *
 * @return
 *  - ESP_OK    Success
 *  - ESP_FAIL  Parameter error
 * */
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_io_num, int rx_io_num);