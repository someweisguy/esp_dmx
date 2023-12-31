/**
 * @file dmx/include/driver.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This header defines functions which allow for installation and
 * configuration of the DMX driver. This includes functions which modify DMX
 * timing and can enable or disable the DMX driver.
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Installs the DMX driver, sets the default configuration, and sets the
 * DMX personalities.
 *
 * @note The DMX driver will allocate a hardware timer to use. When using
 * ESP-IDF v4.4 the DMX driver will allocate a hardware timer group and timer
 * relative to the DMX port number. It is not recommended to use the hardware
 * timer that the DMX driver is using while the DMX driver is installed.
 *
 * @note The DMX interrupt service routine is installed on the same CPU core
 * that this function is running on.
 *
 * @param dmx_num The DMX port number.
 * @param[in] config A pointer to a DMX configuration which will be used to
 * setup the DMX driver.
 * @param[in] personalities A pointer to an array of DMX personalities which
 * the DMX driver will support.
 * @param personality_count The number of personalities in the previous
 * argument. May be set to 0 if this device does not support a DMX address.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_driver_install(dmx_port_t dmx_num, dmx_config_t *config,
                        dmx_personality_t *personalities,
                        uint8_t personality_count);

/**
 * @brief Uninstalls the DMX driver.
 *
 * @param dmx_num The DMX port number
 * @return true on success.
 * @return false on failure.
 */
bool dmx_driver_delete(dmx_port_t dmx_num);

/**
 * @brief Disables the DMX driver.
 *
 * @note When the DMX driver is not placed in IRAM, functions which disable the
 * cache, such as functions which read or write to flash, will also stop DMX
 * interrupts from firing. This can cause incoming DMX data to become corrupted.
 * To avoid this issue, the DMX driver should be disabled before disabling the
 * cache. When cache is reenabled, the DMX driver can be reenabled as well. When
 * the DMX driver is placed in IRAM, disabling and reenabling the DMX driver is
 * not needed.
 *
 * @param dmx_num The DMX port number.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_driver_disable(dmx_port_t dmx_num);

/**
 * @brief Enables the DMX driver.
 *
 * @note When the DMX driver is not placed in IRAM, functions which disable the
 * cache, such as functions which read or write to flash, will also stop DMX
 * interrupts from firing. This can cause incoming DMX data to become corrupted.
 * To avoid this issue, the DMX driver should be disabled before disabling the
 * cache. When cache is reenabled, the DMX driver can be reenabled as well. When
 * the DMX driver is placed in IRAM, disabling and reenabling the DMX driver is
 * not needed.
 *
 * @param dmx_num The DMX port number.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_driver_enable(dmx_port_t dmx_num);

/**
 * @brief Checks if DMX driver is installed.
 *
 * @param dmx_num The DMX port number.
 * @return true if the driver is installed.
 * @return false if the driver is not installed or DMX port does not exist.
 * */
bool dmx_driver_is_installed(dmx_port_t dmx_num);

/**
 * @brief Checks if the DMX driver is enabled.
 *
 * @note When the DMX driver is not placed in IRAM, functions which disable the
 * cache, such as functions which read or write to flash, will also stop DMX
 * interrupts from firing. This can cause incoming DMX data to become corrupted.
 * To avoid this issue, the DMX driver should be disabled before disabling the
 * cache. When cache is reenabled, the DMX driver can be reenabled as well. When
 * the DMX driver is placed in IRAM, disabling and reenabling the DMX driver is
 * not needed.
 *
 * @param dmx_num The DMX port number.
 * @return true if the driver is enabled.
 * @return false if the driver is disabled.
 */
bool dmx_driver_is_enabled(dmx_port_t dmx_num);

/**
 * @brief Sets DMX pin number.
 *
 * @param dmx_num The DMX port number.
 * @param tx_pin The pin to which the TX signal will be assigned.
 * @param rx_pin The pin to which the RX signal will be assigned.
 * @param rts_pin The pin to which the RTS signal will be assigned.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_set_pin(dmx_port_t dmx_num, int tx_pin, int rx_pin, int rts_pin);

/**
 * @brief Gets the DMX baud rate.
 *
 * @param dmx_num The DMX port number.
 * @return the current baud rate or 0 on error.
 */
uint32_t dmx_get_baud_rate(dmx_port_t dmx_num);

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
 * @brief Gets the DMX break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX break length or 0 on error.
 */
uint32_t dmx_get_break_len(dmx_port_t dmx_num);

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
 * @brief Gets the DMX mark-after-break length in microseconds.
 *
 * @param dmx_num The DMX port number.
 * @return the current DMX mark-after-break length or 0 on error.
 */
uint32_t dmx_get_mab_len(dmx_port_t dmx_num);

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
 * @brief Returns the 48-bit unique ID of the desired DMX port. The specified
 * DMX driver must be installed before calling this function.
 *
 * @param dmx_num The DMX port number.
 * @return A pointer to the DMX driver's RDM UID or NULL on failure.
 */
const rdm_uid_t *rdm_uid_get(dmx_port_t dmx_num);

#ifdef __cplusplus
}
#endif
