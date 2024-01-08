/**
 * @file rdm/responder/include/queue_status.h
 * @author Mitch Weisbrod
 * @brief This file contains RDM queue and status functions for the RDM
 * responder. The PIDs in RDM queue and status include RDM_PID_QUEUED_MESSAGE,
 * RDM_PID_STATUS_MESSAGES, RDM_PID_STATUS_ID_DESCRIPTION,
 * RDM_PID_CLEAR_STATUS_ID, and RDM_PID_SUB_DEVICE_STATUS_ID_THRESHOLD. This
 * file also includes getters and setters for these function as appropriate.
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

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
