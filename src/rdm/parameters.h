/**
 * @file parameters.h
 * @author Mitch Weisbrod
 * @brief This file contains getters and setters for various RDM parameters. It
 * is included with `rdm/responder.h`.
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Gets the device label.
 * 
 * @param dmx_num The DMX port number.
 * @param[out] label A pointer to a buffer that the device_label will be copied into.
 *                   This will not contain a trailing '\0'
 * @param labelLen The size of @p label
 * @return The number of bytes copied
*/
size_t rdm_get_device_label(dmx_port_t dmx_num, char * label, size_t label_len);

#ifdef __cplusplus
}
#endif
