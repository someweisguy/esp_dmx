/**
 * @file utils.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/include/parameter.h"
#include "dmx/types.h"
#include "rdm/responder.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
size_t rdm_write_ack(dmx_port_t dmx_num, const rdm_header_t *header,
                     const char *format, const void *pd, size_t pdl);

// TODO: docs
size_t rdm_write_nack_reason(dmx_port_t dmx_num, const rdm_header_t *header, 
                             rdm_nr_t nack_reason);

/*
// TODO:
size_t rdm_write_ack_timer(dmx_port_t dmx_num, const rdm_header_t *header,
                           TickType_t ready_ticks);
*/

/*
// TODO:
size_t rdm_write_ack_overflow(dmx_port_t dmx_num, const rdm_header_t *header,
                              const char *format, const void *pd, size_t pdl,
                              int page);
*/

// TODO: docs
bool rdm_queue_push(dmx_port_t dmx_num, rdm_pid_t pid);

// TODO: docs
rdm_pid_t rdm_queue_pop(dmx_port_t dmx_num);

// TODO: docs
uint8_t rdm_queue_size(dmx_port_t dmx_num);

// TODO: docs
rdm_pid_t rdm_queue_previous(dmx_port_t dmx_num);

// TODO: docs
void rdm_set_boot_loader(dmx_port_t dmx_num);

size_t rdm_simple_response_handler(dmx_port_t dmx_num,
                                   const rdm_parameter_definition_t *definition,
                                   const rdm_header_t *header);

#ifdef __cplusplus
}
#endif
