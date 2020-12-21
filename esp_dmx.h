#pragma once

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef int dmx_port_t;

/** 
 * @brief Install DMX driver and set the DMX to the default configuration.
 * 
 * DMX ISR handler will be attached to the same XPU core that this function is running on.
 * 
 * @return 
 *  - ESP_OK    Success
 *  - ESP_FAIL  Parameter error
 * */
esp_err_t dmx_driver_install(dmx_port_t dmx_num, int buffer_size, int queue_size, QueueHandle_t *dmx_queue, int intr_alloc_flags);

/**
 * @brief Uninstall DMX driver.
 * 
 * @return 
 *  - ESP_OK    Success
 *  - ESP_FAIL  Parameter error
 * */
esp_err_t dmx_driver_delete(dmx_port_t dmx_num);

/** 
 * @brief Set DMX pin number.
 * 
 * @return
 *  - ESP_OK    Success
 *  - ESP_FAIL  Parameter error
 * */
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_io_num, int rx_io_num);