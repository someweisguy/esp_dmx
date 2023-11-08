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
const void *rdm_pd_add_new(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           rdm_pid_t pid, const rdm_pd_schema_t *schema,
                           const rdm_pd_dimensions_t *dimensions,
                           const void *init_value);

// TODO: docs
const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             rdm_pid_t pid, const rdm_pd_schema_t *schema,
                             const rdm_pd_dimensions_t *dimensions,
                             rdm_pid_t alias, size_t offset);

// TODO: docs
bool rdm_pd_add_deterministic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, const rdm_pd_schema_t *schema,
                              const rdm_pd_dimensions_t *dimensions);

// TODO: docs
bool rdm_pd_update_response_handler(dmx_port_t dmx_num,
                                    rdm_sub_device_t sub_device, rdm_pid_t pid,
                                    rdm_response_handler_t response_handler);

// TODO: docs
bool rdm_pd_update_callback(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, rdm_callback_t callback,
                            void *context);

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
void *rdm_pd_get(dmx_port_t dmx_num, rdm_pid_t pid,
                 rdm_sub_device_t sub_device);

/**
 * @brief Sets the value of a specified RDM parameter. This function will not
 * set the value of an RDM parameter if the parameter does not support SET
 * requests.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to set.
 * @param sub_device The sub-device number which owns the parameter.
 * @param[in] param A pointer to the new value to which to set the parameter.
 * @param size The size of the new value of the parameter.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_pd_set(dmx_port_t dmx_num, rdm_pid_t pid, rdm_sub_device_t sub_device,
                const void *data, size_t size);

// TODO docs
int rdm_pd_enqueue(dmx_port_t dmx_num, rdm_pid_t pid,
                   rdm_sub_device_t sub_device);

// TODO docs
bool rdm_pd_get_description(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_sub_device_t sub_device,
                            rdm_pid_description_t *description);

// TODO docs
uint32_t rdm_pd_list(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                     uint16_t *pids, uint32_t num);

#ifdef __cplusplus
}
#endif