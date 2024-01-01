/**
 * @file dmx/include/device.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This header contains functions will allow for modification of the DMX
 * device. This includes getting and setting the DMX start address, current
 * personality, and information about the DMX personality.
 */
#pragma once

#include "dmx/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets the DMX start address of the DMX driver.
 *
 * @param dmx_num The DMX port number.
 * @return The DMX start address of the DMX driver or 0 on failure.
 */
uint16_t dmx_get_start_address(dmx_port_t dmx_num);

/**
 * @brief Sets the DMX start address of the DMX driver. The DMX start address
 * cannot be set if it is set to DMX_START_ADDRESS_NONE.
 *
 * @param dmx_num The DMX port number.
 * @param dmx_start_address The start address at which to set the DMX driver.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_set_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address);

/**
 * @brief Gets the current personality of the DMX driver.
 *
 * @param dmx_num The DMX port number.
 * @return The current personality or 0 on failure.
 */
uint8_t dmx_get_current_personality(dmx_port_t dmx_num);

/**
 * @brief Sets the current personality of the DMX driver. Personalities are
 * indexed starting at 1.
 *
 * @param dmx_num The DMX port number.
 * @param personality_num The personality number to which to set the DMX driver.
 * Personality number are indexed starting at 1.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_set_current_personality(dmx_port_t dmx_num, uint8_t personality_num);

/**
 * @brief Gets the personality count of the DMX driver.
 *
 * @param dmx_num The DMX port number.
 * @return The personality count or 0 on failure.
 */
uint8_t dmx_get_personality_count(dmx_port_t dmx_num);

/**
 * @brief Gets the footprint of the specified personality.
 *
 * @param dmx_num The DMX port number.
 * @param personality_num The personality number of the footprint to get.
 * Personality numbers are indexed starting at 1.
 * @return The footprint of the specified personality or 0 on failure.
 */
size_t dmx_get_footprint(dmx_port_t dmx_num, uint8_t personality_num);

/**
 * @brief Gets the description of the specified personality.
 *
 * @param dmx_num The DMX port number.
 * @param personality_num The personality number of the description to get.
 * Personality numbers are indexed starting at 1.
 * @return The description of the DMX personality or NULL on failure.
 */
const char *dmx_get_personality_description(dmx_port_t dmx_num,
                                            uint8_t personality_num);

#ifdef __cplusplus
}
#endif
