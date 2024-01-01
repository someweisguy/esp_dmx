/**
 * @file rdm/responder/queue_status.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/responder.h"
#include "rdm/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO docs
bool rdm_register_queued_message(dmx_port_t dmx_num, uint32_t max_count,
                                 rdm_callback_t cb, void *context);

// TODO: docs
bool rdm_queue_push(dmx_port_t dmx_num, rdm_pid_t pid);

// TODO: docs
rdm_pid_t rdm_queue_pop(dmx_port_t dmx_num);

// TODO: docs
uint8_t rdm_queue_size(dmx_port_t dmx_num);

// TODO: docs
rdm_pid_t rdm_queue_previous(dmx_port_t dmx_num);

#ifdef __cplusplus
}
#endif
