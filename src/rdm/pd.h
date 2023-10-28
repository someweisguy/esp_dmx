/**
 * @file rdm/pd.h
 * @author Mitch Weisbrod
 * @brief 
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

/**
 * @brief A function type for RDM responder callbacks. This is the type of
 * function that is called when responding to RDM requests.
 */
typedef int (*rdm_response_handler_t)(dmx_port_t dmx_num, rdm_header_t *header,
                               void *pd, uint8_t *pdl, const char *format);

// TODO docs
uint32_t rdm_pd_list(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                     uint16_t *pids, uint32_t num);

/**
 * @brief Gets a pointer to the parameter stored in the RDM device, if the
 * parameter exists.
 *
 * @note This function returns a pointer to the raw parameter data which is
 * stored on the RDM device. It is possible to edit the data directly but this
 * is not recommended for most use cases. The proper way to update RDM parameter
 * data would be to use the function `rdm_pd_set()` because it properly
 * updates NVS and the RDM queue.
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
 * @param add_to_queue True to add this parameter to the RDM message queue.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_pd_set(dmx_port_t dmx_num, rdm_pid_t pid, rdm_sub_device_t sub_device,
                const void *data, size_t size, bool add_to_queue);

// TODO: docs
const void *rdm_pd_add_new(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           const rdm_pid_description_t *definition,
                           const char *format, bool nvs,
                           rdm_response_handler_t response_handler,
                           void *default_value);

// TODO: docs
const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             const rdm_pid_description_t *definition,
                             const char *format, bool nvs,
                             rdm_response_handler_t response_handler,
                             rdm_pid_t alias, size_t offset);

// TODO: docs
bool rdm_pd_add_deterministic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              const rdm_pid_description_t *definition,
                              const char *format,
                              rdm_response_handler_t response_handler);

// TODO: docs
bool rdm_pd_update_response_handler(dmx_port_t dmx_num,
                                    rdm_sub_device_t sub_device,
                                    rdm_pid_t pid,
                                    rdm_response_handler_t response_handler);

// TODO: docs
bool rdm_pd_update_callback(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, rdm_callback_t callback,
                            void *context);

#ifdef __cplusplus
}
#endif