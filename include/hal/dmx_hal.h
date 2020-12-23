#pragma once

#include "hal/dmx_ll.h"

/**
 * @brief Get the idle num (in bits).
 * 
 * @param  hal Context of the HAL layer
 *
 * @return UART idle num
 */
#define dmx_hal_get_idle_num(hal) dmx_ll_get_idle_num((hal)->dev)

/**
 * @brief Get the break num (in bits).
 * 
 * @param  hal Context of the HAL layer
 *
 * @return UART break num
 */
#define dmx_hal_get_break_num(hal) dmx_ll_get_break_num((hal)->dev)