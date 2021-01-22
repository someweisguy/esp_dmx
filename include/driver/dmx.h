#pragma once

#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/dmx_types.h"
#include "soc/uart_caps.h"

#define DMX_NUM_0                   0             // DMX port 0.
#define DMX_NUM_1                   1             // DMX port 1.
#if SOC_UART_NUM > 2
#define DMX_NUM_2                   2             // DMX port 2.
#endif
#define DMX_NUM_MAX                 SOC_UART_NUM  // DMX port max.

/* DMX shared parameters */
#define DMX_MIN_BAUDRATE            245000        // DMX minimum baudrate.
#define DMX_TYP_BAUDRATE            250000        // DMX typical baudrate.
#define DMX_MAX_BAUDRATE            255000        // DMX maximum baudrate.
#define DMX_MAX_PACKET_SIZE         513           // DMX maximum packet size.

/* DMX client/receive timing parameters */
#define DMX_RX_MIN_SPACE_FOR_BRK_US 88            // DMX minimum receivable break length in microseconds.
#define DMX_RX_MIN_MRK_AFTER_BRK_US 8             // DMX minimum receivable mark after break length in microseconds.
#define DMX_RX_MAX_MRK_AFTER_BRK_US 999999        // DMX maximum receivable mark after break length in microseconds.
#define DMX_RX_MIN_BRK_TO_BRK_US    1196          // DMX minimum receivable break-to-break length in microseconds.
#define DMX_RX_MAX_BRK_TO_BRK_US    1250000       // DMX maximum receivable break-to-break length in microseconds.
#define DMX_RX_PACKET_TOUT_MS       1250          // DMX client packet timeout in milliseconds.
#define DMX_RX_PACKET_TOUT_TICK     ((TickType_t)DMX_RX_PACKET_TOUT_MS / portTICK_PERIOD_MS) // DMX client packet timeout in FreeRTOS ticks.

/* DMX host/transmit timing parameters */
#define DMX_TX_MIN_SPACE_FOR_BRK_US 92            // DMX minimum transmittable break length in microseconds.
#define DMX_TX_MIN_MRK_AFTER_BRK_US 12            // DMX minimum transmittable mark after break length in microseconds.
#define DMX_TX_MAX_MRK_AFTER_BRK_US 999999        // DMX maximum transmittable mark after break length in microseconds.
#define DMX_TX_MIN_BRK_TO_BRK_US    1204          // DMX minimum transmittable break-to-break length in microseconds.
#define DMX_TX_MAX_BRK_TO_BRK_US    1000000       // DMX maximum transmittable break-to-break length in microseconds.
#define DMX_TX_PACKET_TOUT_MS       1000          // DMX host packet timeout in milliseconds.
#define DMX_TX_PACKET_TOUT_TICK     ((TickType_t)DMX_TX_PACKET_TOUT_MS / portTICK_PERIOD_MS) // DMX host packet timeout in FreeRTOS ticks.


typedef int dmx_port_t;             // DMX port type.

/**
 * @brief DMX packet types reported to the event queue when a packet is received.
 */
typedef enum {
  DMX_OK                    = 0,    // The DMX packet is valid.
  DMX_ERR_BRK_TO_BRK        = BIT0, // The break-to-break time is invalid.
  DMX_ERR_IMPROPER_SLOT     = BIT1, // A slot is improperly framed (missing stop bits).
  DMX_ERR_PACKET_SIZE       = BIT2, // The packet size is 0 or longer than the DMX standard allows.
  DMX_ERR_BUFFER_SIZE       = BIT3, // The user defined buffer is too small for the received packet.
  DMX_ERR_DATA_OVERFLOW     = BIT4, // The hardware FIFO overflowed, causing loss of data.

  // The remaining event types only occur if detailed rx analysis is enabled

  DMX_ERR_SPACE_FOR_BRK     = BIT5, // The space length is invalid.
  DMX_ERR_MRK_AFTER_BRK     = BIT6, // The mark after break length is invalid.
} dmx_event_type_t;

#define DMX_ERR_CORRUPT_DATA  (DMX_ERR_IMPROPER_SLOT | DMX_ERR_DATA_OVERFLOW) // Bitmask for error conditions where data is lost.
#define DMX_ERR_NOT_TO_SPEC   (DMX_ERR_BRK_TO_BRK | DMX_ERR_PACKET_SIZE | DMX_ERR_SPACE_FOR_BRK | DMX_ERR_MRK_AFTER_BRK) // Bitmask for error conditions where the bytestream is not to DMX specification.

/**
 * @brief DMX data events reported to the event queue when a packet is received.
 */
typedef struct {
  dmx_event_type_t type;            // The type of DMX packet received.
  int16_t start_code;               // The start code (slot 0) of the DMX packet, or -1 on error (except for DMX_ERR_BUFFER_SIZE).
  size_t size;                      // The length of the received DMX packet.
  uint32_t packet_len;
  uint32_t brk_len;
  uint32_t mab_len;
} dmx_event_t;

/**
 * @brief Interrupt configuration used to configure the DMX hardware ISR.
 */
typedef struct {
  uint8_t rx_timeout_thresh;        // DMX timeout interrupt threshold (unit: time of sending one byte).
  uint8_t txfifo_empty_intr_thresh; // DMX tx empty interrupt threshold.
  uint8_t rxfifo_full_thresh;       // DMX rx full interrupt threshold.
} dmx_intr_config_t;

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

esp_err_t dmx_rx_analyze_enable(dmx_port_t dmx_num, int analyze_io_num, int intr_alloc_flags);

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
