/**
 * @file rdm/utils/pd.h
 * @author Mitch Weisbrod
 * @brief // TODO
 *
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "dmx/types.h"
#include "rdm/responder.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
const void *rdm_pd_add_new(dmx_port_t dmx_num, rdm_pid_t pid,
                           rdm_sub_device_t sub_device,
                           const rdm_pd_definition_t *def,
                           const void *init_value);

// TODO: docs
const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_pid_t pid,
                             rdm_sub_device_t sub_device,
                             const rdm_pd_definition_t *def, rdm_pid_t alias,
                             size_t offset);

// TODO: docs
bool rdm_pd_add_deterministic(dmx_port_t dmx_num, rdm_pid_t pid,
                              rdm_sub_device_t sub_device,
                              const rdm_pd_definition_t *def);

// TODO: docs
bool rdm_pd_update_response_handler(dmx_port_t dmx_num, rdm_pid_t pid,
                                    rdm_sub_device_t sub_device,
                                    rdm_response_handler_t response_handler);

// TODO: docs
bool rdm_pd_update_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_sub_device_t sub_device,
                            rdm_callback_t callback, void *context);

/**
 * @brief Checks if an RDM parameter has been added to the DMX driver.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to check.
 * @param sub_device The sub-device number which owns the parameter.
 * @return true if the parameter has been added.
 * @return false if the parameter has not been added.
 */
bool rdm_pd_exists(dmx_port_t dmx_num, rdm_pid_t pid,
                   rdm_sub_device_t sub_device);

/**
 * @brief Gets a pointer to the parameter stored in the RDM device, if the
 * parameter exists.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to get.
 * @param sub_device The sub-device number which owns the parameter.
 * @return A pointer to the parameter data or NULL if the parameter does not
 * exist.
 */
const void *rdm_pd_get(dmx_port_t dmx_num, rdm_pid_t pid,
                       rdm_sub_device_t sub_device);

/**
 * @brief Sets the value of a specified RDM parameter.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to set.
 * @param sub_device The sub-device number which owns the parameter.
 * @param[in] param A pointer to the new value to which to set the parameter.
 * @param size The size of the new value of the parameter.
 * @return true on success.
 * @return false on failure.
 */
size_t rdm_pd_set(dmx_port_t dmx_num, rdm_pid_t pid,
                  rdm_sub_device_t sub_device, const void *data, size_t size);

// TODO: docs
size_t rdm_pd_set_and_queue(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_sub_device_t sub_device, const void *data,
                            size_t size);

// TODO: docs
const rdm_pd_schema_t *rdm_pd_get_schema(dmx_port_t dmx_num, rdm_pid_t pid,
                                         rdm_sub_device_t sub_device);

// TODO docs
bool rdm_pd_get_description(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_sub_device_t sub_device,
                            rdm_pid_description_t *description);

// TODO docs
uint32_t rdm_pd_list(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                     uint16_t *pids, uint32_t num);

// TODO: docs
int rdm_pd_call_response_handler(dmx_port_t dmx_num, rdm_header_t *header,
                                 void *pd, uint8_t *pdl_out);

// TODO: docs
int rdm_response_handler_simple(dmx_port_t dmx_num, rdm_header_t *header,
                                void *pd, uint8_t *pdl_out,
                                const rdm_pd_schema_t *schema);
// TODO: docs
size_t rdm_pd_serialize(void *destination, size_t len, const char *format,
                        const void *source);

// TODO: docs
size_t rdm_pd_deserialize(void *destination, size_t len, const char *format,
                          const void *source);

/**
 * @brief Serializes a 16-bit word into a destination. Used as a convenience
 * function for quickly emplacing NACK reasons and timer values.
 *
 * @param[out] destination A pointer to a destination buffer.
 * @param word The word to serialize.
 * @return The size of the word which was written. Is always 2.
 */
size_t rdm_pd_serialize_word(void *destination, uint16_t word);

#ifdef __cplusplus
}
#endif
