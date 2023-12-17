/**
 * @file dmx/include/parameter.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief // TODO
 */
#pragma once

#include "dmx/types.h"
#include "rdm/responder.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// FIXME
#define rdm_format_is_valid(f) \
  ((f) == NULL || dmx_parameter_format_size(f) > 0)

// TODO: docs, not thread-safe
bool dmx_parameter_add_dynamic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               rdm_pid_t pid, bool non_volatile,
                               const void *init, size_t size);

// TODO: docs, not thread-safe
bool dmx_parameter_add_static(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, bool non_volatile, void *data,
                              size_t size);

bool dmx_parameter_add_null(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid);

// TODO: docs
bool dmx_parameter_exists(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid);

// TODO: docs
rdm_pid_t dmx_parameter_at(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           uint32_t index);

// TODO: docs
size_t dmx_parameter_size(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid);

// TODO: docs, returned pointer is not thread-safe
void *dmx_parameter_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                        rdm_pid_t pid);

// TODO: docs
size_t dmx_parameter_copy(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid, void *destination, size_t size);

// TODO: docs
size_t dmx_parameter_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                         rdm_pid_t pid, const void *source, size_t size);

// TODO: docs, not thread-safe
rdm_pid_t dmx_parameter_commit(dmx_port_t dmx_num);

// TODO: make static and move to dmx/io.h?
size_t dmx_parameter_format_size(const char *format);

#ifdef __cplusplus
}
#endif
