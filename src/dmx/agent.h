#pragma once

#include "dmx/types.h"
#include "esp_system.h"

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
    .alloc_size = 512, .model_id = 0, .product_category = 0x0100,             \
    .software_version_id = ESP_IDF_VERSION_VAL(                               \
        ESP_DMX_VERSION_MAJOR, ESP_DMX_VERSION_MINOR, ESP_DMX_VERSION_PATCH), \
    .current_personality = 1, .personalities = {{1, "Default Personality"}},  \
    .personality_count = 1, .dmx_start_address = 0                            \
  }

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

#ifdef __cplusplus
}
#endif
