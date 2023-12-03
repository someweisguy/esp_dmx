/**
 * @file rdm/responder/product_info.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers the default response to RDM_PID_DEVICE_INFO requests. This
 * response is required by all RDM-capable devices. It is called when the DMX
 * driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] device_info A pointer to the device info parameter to use in
 * RDM responses.  This value is used to set the parameter to a default value
 * when this function is called for the first time and is ignored (and therefore
 * may be set to NULL) on subsequent calls.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info,
                              rdm_callback_t cb, void *context);

/**
 * @brief Registers the default response to RDM_PID_DEVICE_LABEL requests.
 * It is called when the DMX driver is initially installed.
 * 
 * @param dmx_num The DMX port number.
 * @param device_label A pointer to a null-terminated device label string 
 * to use in RDM responses. This value is used to set the
 * parameter to a default value when this function is called for the first time
 * and is ignored (and therefore may be set to NULL) on subsequent calls.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
*/
bool rdm_register_device_label(dmx_port_t dmx_num,
                               const char *device_label,
                               rdm_callback_t cb, void *context);

/**
 * @brief Registers the default response to RDM_PID_SOFTWARE_VERSION_LABEL
 * requests. This response is required by all RDM-capable devices. It is called
 * when the DMX driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param[in] software_version_label A pointer to a null-terminated software
 * version label string to use in RDM responses. This value is used to set the
 * parameter to a default value when this function is called for the first time
 * and is ignored (and therefore may be set to NULL) on subsequent calls.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         char *software_version_label,
                                         rdm_callback_t cb, void *context);

#ifdef __cplusplus
}
#endif
