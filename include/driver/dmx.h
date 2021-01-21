#pragma once

#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/dmx_types.h"
#include "soc/uart_caps.h"

#define DMX_NUM_0           0             // DMX port 0.
#define DMX_NUM_1           1             // DMX port 1.
#if SOC_UART_NUM > 2
#define DMX_NUM_2           2             // DMX port 2.
#endif
#define DMX_NUM_MAX         SOC_UART_NUM  // DMX port max.

#define DMX_MIN_BAUDRATE    245000        // DMX minimum baudrate.
#define DMX_TYP_BAUDRATE    250000        // DMX typical baudrate.
#define DMX_MAX_BAUDRATE    255000        // DMX maximum baudrate.
#define DMX_RX_PACKET_MS    1250          // DMX rx packet timeout in milliseconds.
#define DMX_TX_PACKET_MS    1000          // DMX tx packet timeout in milliseconds.

#define DMX_TICK_RX_PACKET  ((TickType_t)DMX_RX_PACKET_MS / portTICK_PERIOD_MS) // DMX rx packet timeout in FreeRTOS ticks.
#define DMX_TICK_TX_PACKET  ((TickType_t)DMX_TX_PACKET_MS / portTICK_PERIOD_MS) // DMX tx packet timeout in FreeRTOS ticks.

typedef int dmx_port_t;     // DMX port type.

typedef intr_handle_t dmx_isr_handle_t;

typedef enum {
  DMX_OK = 0,                       // The DMX packet is valid.
  DMX_ERR_BRK_TO_BRK,               // The break-to-break time is invalid.
  DMX_ERR_IMPROPER_SLOT,            // A slot is improperly framed (missing stop bits).
  DMX_ERR_PACKET_LENGTH,            // The data packet size is shorter or longer than the DMX standard allows.
  DMX_ERR_BUFFER_LENGTH,            // The user defined buffer is too small for the received packet.
  DMX_ERR_PACKET_OVERFLOW,          // The hardware FIFO overflowed, causing loss of data.
} dmx_event_type_t;

typedef struct {
  dmx_event_type_t type;            // The type of DMX packet received.
  int16_t start_code;               // The start code (slot 0) of the DMX packet, or -1 on error.
  size_t size;                      // The length of the received DMX packet.
} dmx_event_t;

typedef struct {
  uint32_t intr_enable_mask;        // DMX interrupt enable mask, choose from UART_XXXX_INT_ENA_M under UART_INT_ENA_REG(i), connect with bit-or operator.
  uint8_t rx_timeout_thresh;        // DMX timeout interrupt threshold (unit: time of sending one byte).
  uint8_t txfifo_empty_intr_thresh; // DMX tx empty interrupt threshold.
  uint8_t rxfifo_full_thresh;       // DMX rx full interrupt threshold.
} dmx_intr_config_t;

typedef enum {
  DMX_MODE_RX,                      // DMX receive mode.
  DMX_MODE_TX                       // DMX transmit mode.
} dmx_mode_t;

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
 * @brief Sets the dmx mode, either DMX_MODE_RX or DMX_MODE_TX.
 * 
 * @param dmx_num 
 * @param dmx_mode 
 * @return 
 * - ESP_OK                 Success
 * - ESP_ERR_INVALID_ARG    Parameter error
 * - ESP_ERR_INVALID_STATE  Driver not installed
 */
esp_err_t dmx_set_mode(dmx_port_t dmx_num, dmx_mode_t dmx_mode);

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
