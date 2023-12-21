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
typedef struct rdm_parameter_definition_t {
  rdm_pid_t pid;
  uint8_t pid_cc;
  uint8_t ds;
  struct rdm_command_t {
    struct {
      const char *format;
    } request, response;
    size_t (*handler)(dmx_port_t dmx_num,
                      const struct rdm_parameter_definition_t *definition,
                      const rdm_header_t *header);
  } get, set;
  uint8_t pdl_size;
  uint32_t max_value;
  uint32_t min_value;
  uint32_t default_value;
  uint8_t units;
  uint8_t prefix;
  const char *description;
} rdm_parameter_definition_t;

// TODO: docs, not thread-safe
bool rdm_parameter_define(const rdm_parameter_definition_t *definition);

const rdm_parameter_definition_t *rdm_parameter_lookup(rdm_pid_t pid);

// TODO: docs, not thread-safe
bool rdm_parameter_callback_set(rdm_pid_t pid, rdm_callback_t callback,
                                void *context);

// TODO: docs
bool rdm_parameter_callback_handle(dmx_port_t dmx_num, rdm_pid_t pid,
                                   rdm_header_t *request_header,
                                   rdm_header_t *response_header);

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
void rdm_set_boot_loader(dmx_port_t dmx_num);

size_t rdm_simple_response_handler(dmx_port_t dmx_num,
                                   const rdm_parameter_definition_t *definition,
                                   const rdm_header_t *header);

#ifdef __cplusplus
}
#endif
