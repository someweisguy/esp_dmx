/**
 * @file rdm/responder/queue_status.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/types.h"
#include "rdm/responder.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO docs
bool rdm_register_queued_message(dmx_port_t dmx_num, rdm_callback_t cb,
                                 void *context);

#ifdef __cplusplus
}
#endif
