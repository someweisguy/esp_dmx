/**
 * @file driver.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "dmx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enables the DMX sniffer to determine the DMX break and
 * mark-after-break length.
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
 * @param intr_pin The pin to which to assign the interrupt.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_sniffer_enable(dmx_port_t dmx_num, int intr_pin);

/**
 * @brief Disables the DMX sniffer.
 *
 * @param dmx_num The DMX port number.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_sniffer_disable(dmx_port_t dmx_num);

/**
 * @brief Checks if the sniffer is enabled.
 *
 * @param dmx_num The DMX port number.
 * @retval true if the sniffer is installed.
 * @retval false if the sniffer is not installed or DMX port does not exist.
 */
bool dmx_sniffer_is_enabled(dmx_port_t dmx_num);

/**
 * @brief Gets sniffer data if it is available.
 *
 * @param dmx_num The DMX port number.
 * @param[out] metadata A pointer to a dmx_metadata_t struct into which to
 * copy DMX sniffer data.
 * @param wait_ticks The number of ticks to wait before this function times out.
 * @return true if data was copied.
 * @return false if data was not copied.
 */
bool dmx_sniffer_get_data(dmx_port_t dmx_num, dmx_metadata_t *metadata,
                          TickType_t wait_ticks);

#ifdef __cplusplus
}
#endif
