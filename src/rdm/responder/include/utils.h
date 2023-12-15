/**
 * @file utils.h
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

/*
definition/callbacks
parameters
write acks
simple response handler
queue
bootloader
*/


// FIXME
#define rdm_pd_format_is_valid(f) \
  ((f) != NULL && rdm_pd_format_get_max_size(f) > 0)

// TODO: docs
typedef struct rdm_pd_definition_s {
  rdm_pid_t pid;
  size_t alloc_size;
  uint8_t pid_cc;
  uint8_t ds;
  struct {
    struct {
      const char *format;
    } request, response;
    size_t (*handler)(dmx_port_t dmx_num,
                      const struct rdm_pd_definition_s *definition,
                      const rdm_header_t *header);
  } get, set;
  uint8_t pdl_size;
  uint32_t max_value;
  uint32_t min_value;
  uint32_t default_value;
  uint8_t units;
  uint8_t prefix;
  const char *description;
} rdm_pd_definition_t;

const rdm_pd_definition_t *rdm_parameter_lookup(rdm_pid_t pid);

// TODO: docs, not thread-safe
int rdm_parameter_define(const rdm_pd_definition_t *definition);

// TODO: docs
void rdm_pd_handle_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_header_t *request_header,
                            rdm_header_t *response_header);

// TODO: docs, not thread-safe
bool rdm_pd_set_callback(rdm_pid_t pid, rdm_callback_t callback, void *context);

// TODO: docs
size_t rdm_write_ack(dmx_port_t dmx_num, const rdm_header_t *header,
                     const char *format, const void *pd, size_t pdl);

// TODO: docs
size_t rdm_write_nack_reason(dmx_port_t dmx_num, const rdm_header_t *header, 
                             rdm_nr_t nack_reason);

// TODO:
/*
size_t rdm_write_ack_timer(dmx_port_t dmx_num, const rdm_header_t *header,
                           TickType_t ready_ticks);
*/

// TODO:
/*
size_t rdm_write_ack_overflow(dmx_port_t dmx_num, const rdm_header_t *header,
                              const char *format, const void *pd, size_t pdl,
                              int page);
*/

size_t rdm_pd_format_get_max_size(const char *format);

// TODO: docs, not thread-safe
bool rdm_parameter_add_dynamic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               rdm_pid_t pid, bool non_volatile,
                               const void *init, size_t size);

// TODO: docs, not thread-safe
bool rdm_parameter_add_static(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, bool non_volatile, void *data,
                              size_t size);

// TODO: docs, returned pointer is not thread-safe
const void *rdm_parameter_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid);                  

// TODO: docs
size_t rdm_parameter_copy(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid, void *destination, size_t size);

// TODO: docs
size_t rdm_parameter_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                         rdm_pid_t pid, const void *source, size_t size);

// TODO: docs
size_t rdm_parameter_set_and_queue(dmx_port_t dmx_num,
                                   rdm_sub_device_t sub_device, rdm_pid_t pid,
                                   const void *source, size_t size);

// TODO: docs, not thread-safe
rdm_pid_t rdm_parameter_commit(dmx_port_t dmx_num);

// TODO: docs
rdm_pid_t rdm_queue_pop(dmx_port_t dmx_num);

// TODO: docs
uint8_t rdm_queue_size(dmx_port_t dmx_num);

// TODO: docs
rdm_pid_t rdm_queue_previous(dmx_port_t dmx_num);

size_t rdm_simple_response_handler(dmx_port_t dmx_num,
                                   const rdm_pd_definition_t *definition,
                                   const rdm_header_t *header);

// TODO: docs
void rdm_set_boot_loader(dmx_port_t dmx_num);


#ifdef __cplusplus
}
#endif