/**
 * @file rdm/utils/pd.h
 * @author Mitch Weisbrod
 * @brief // TODO
 *
 */
#pragma once

#include <stdbool.h>

#include "dmx/types.h"
#include "rdm/responder.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

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

// TODO: docs, not thread-safe
int rdm_pd_set_definition(dmx_port_t dmx_num, rdm_pid_t pid,
                          const rdm_pd_definition_t *definition);

size_t rdm_pd_format_get_max_size(const char *format);

// TODO: docs, not thread-safe
bool rdm_pd_set_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                         rdm_callback_t callback, void *context);

const rdm_pd_definition_t *rdm_pd_get_definition(dmx_port_t dmx_num,
                                                 rdm_pid_t pid);

// TODO: docs
void rdm_pd_handle_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_header_t *request_header,
                            rdm_header_t *response_header);

// TODO: docs, not thread-safe
const void *rdm_pd_add_variable(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                                rdm_pid_t pid, bool non_volatile,
                                const void *init_value, size_t size);

// TODO: docs, not thread-safe
const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             rdm_pid_t pid, bool non_volatile, rdm_pid_t alias,
                             size_t offset);

// TODO: docs, not thread-safe
const void *rdm_pd_add_const(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             rdm_pid_t pid, void *data);

// TODO: docs, returned pointer is not thread-safe
const void *rdm_pd_get_ptr(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           rdm_pid_t pid);

// TODO: docs
size_t rdm_pd_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                  rdm_pid_t pid, void *destination, size_t size);

// TODO: docs
size_t rdm_pd_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                  rdm_pid_t pid, const void *source, size_t size);

// TODO: docs
size_t rdm_pd_set_and_queue(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, const void *source, size_t size);

// TODO: docs
rdm_pid_t rdm_pd_queue_pop(dmx_port_t dmx_num);

// TODO: docs
uint8_t rdm_pd_queue_get_size(dmx_port_t dmx_num);

// TODO: docs
rdm_pid_t rdm_pd_queue_get_last_message(dmx_port_t dmx_num);

// TODO: docs, not thread-safe
rdm_pid_t rdm_pd_nvs_commit(dmx_port_t dmx_num);

size_t rdm_simple_response_handler(dmx_port_t dmx_num,
                                   const rdm_pd_definition_t *definition,
                                   const rdm_header_t *header);

#ifdef __cplusplus
}
#endif
