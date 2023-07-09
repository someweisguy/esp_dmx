#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
bool rdm_get_disc_mute(dmx_port_t dmx_num, uint8_t *mute);

// TODO: docs
bool rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info);

// TODO: docs
bool rdm_get_software_version_label(dmx_port_t dmx_num,
                                    const char **software_version_label);

// TODO: docs
bool rdm_get_identify_device(dmx_port_t dmx_num, uint8_t *identify);

// TODO: docs
bool rdm_set_identify_device(dmx_port_t dmx_num, const uint8_t identify);

// TODO: docs
bool rdm_get_dmx_start_address(dmx_port_t dmx_num, uint16_t *dmx_start_address);

// TODO: docs
bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address);

#ifdef __cplusplus
}
#endif
