#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
typedef rdm_response_type_t (*rdm_response_cb_t)(dmx_port_t dmx_num,
                                                 const rdm_header_t *header,
                                                 rdm_mdb_t *mdb, void *context);

// TODO: docs
bool rdm_register_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                           rdm_response_cb_t callback, void *context);

#ifdef __cplusplus
}
#endif
