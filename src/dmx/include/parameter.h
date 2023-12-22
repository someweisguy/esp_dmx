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

/**
 * @brief Evaluates to true if the parameter format string is valid.
 */
#define rdm_format_is_valid(f) \
  ((f) == NULL || dmx_parameter_rdm_format_size(f) > 0)

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


/**
 * @brief Allocates and adds a parameter to the DMX driver. The parameter is
 * heap-allocated. This function is not thread-safe.
 *
 * Each parameter can be identified using its DMX port number, sub-device
 * number, and PID. Each DMX port and sub-device may possess many parameters but
 * each sub-device parameter's PID must be unique.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID which to add.
 * @param non_volatile True if the parameter should persist after the ESP32 is
 * power-cycled.
 * @param[in] init The value to which the parameter should be initialized or
 * NULL to memset the parameter to zero.
 * @param size The size of the parameter to allocate.
 * @return true if the parameter already existed or was allocated.
 * @return false if the parameter could not be allocated.
 */
bool dmx_parameter_add_dynamic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               rdm_pid_t pid, bool non_volatile,
                               const void *init, size_t size);

/**
 * @brief Allocates and adds a parameter to the DMX driver. The parameter is
 * statically allocated. This function is not thread-safe.
 *
 * Each parameter can be identified using its DMX port number, sub-device
 * number, and PID. Each DMX port and sub-device may possess many parameters but
 * each sub-device parameter's PID must be unique.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID which to add.
 * @param non_volatile True if the parameter should persist after the ESP32 is
 * power-cycled.
 * @param[in] data A pointer to the memory which stores the parameter data.
 * @param size The size of the parameter.
 * @return true if the parameter already existed or was allocated.
 * @return false if the parameter could not be allocated.
 */
bool dmx_parameter_add_static(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, bool non_volatile, void *data,
                              size_t size);

bool dmx_parameter_add_null(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid);

/**
 * @brief Returns true if the parameter exists on the given DMX port and
 * sub-device.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID.
 * @return true if the parameter exists.
 * @return false if the parameter does not exist.
 */
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

bool dmx_parameter_rdm_define(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid,
                              const rdm_parameter_definition_t *definition);

const rdm_parameter_definition_t *dmx_parameter_rdm_lookup(
    dmx_port_t dmx_num, rdm_sub_device_t sub_device, rdm_pid_t pid);

bool dmx_parameter_rdm_set_callback(dmx_port_t dmx_num,
                                    rdm_sub_device_t sub_device, rdm_pid_t pid,
                                    rdm_callback_t callback, void *context);

bool dmx_parameter_rdm_handle_callback(dmx_port_t dmx_num,
                                       rdm_sub_device_t sub_device,
                                       rdm_pid_t pid,
                                       rdm_header_t *request_header,
                                       rdm_header_t *response_header);

// TODO: docs
size_t dmx_parameter_rdm_format_size(const char *format);

bool dmx_parameter_rdm_disable(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               rdm_pid_t pid);

#ifdef __cplusplus
}
#endif
