#pragma once

#include <stdint.h>

#include "dmx/types.h"
#include "rdm/agent.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
typedef void (*rdm_identify_cb_t)(dmx_port_t dmx_num, bool identify,
                                  void *context);

// TODO: docs
bool rdm_register_disc_unique_branch(dmx_port_t dmx_num);

// TODO: docs
bool rdm_register_disc_mute(dmx_port_t dmx_num);

// TODO: docs
bool rdm_register_disc_un_mute(dmx_port_t dmx_num);

// TODO: docs
bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info);

// TODO: docs
bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         const char *software_version_label);

// TODO: docs
bool rdm_register_identify_device(dmx_port_t dmx_num,
                                  rdm_identify_cb_t identify_cb, void *context);

// TODO: docs
bool rdm_register_dmx_start_address(dmx_port_t dmx_num,
                                    uint16_t *dmx_start_address);

#ifdef __cplusplus
}
#endif
