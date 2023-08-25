/**
 * @file responder.h
 * @author Mitch Weisbrod
 * @brief This file contains functions used for registering RDM parameters. It
 * must be included by the user.
 */
#pragma once

#include <stdint.h>

#include "dmx_types.h"
#include "rdm/parameters.h"
#include "rdm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The function type for user callbacks in RDM responses.
 */
typedef void (*rdm_responder_cb_t)(dmx_port_t dmx_num,
                                   const rdm_header_t *header, void *context);

/**
 * @brief Registers the default response to RDM_PID_DISC_UNIQUE_BRANCH requests.
 * This response is required by all RDM-capable devices. It is called when the
 * DMX driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_disc_unique_branch(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                     void *context);

/**
 * @brief Registers the default response to RDM_PID_DISC_MUTE requests. This
 * response is required by all RDM-capable devices. It is called when the DMX
 * driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_disc_mute(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                            void *context);

/**
 * @brief Registers the default response to RDM_PID_DISC_UN_MUTE requests. This
 * response is required by all RDM-capable devices. It is called when the DMX
 * driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_disc_un_mute(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                               void *context);

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
                              rdm_responder_cb_t cb, void *context);

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
                                         const char *software_version_label,
                                         rdm_responder_cb_t cb, void *context);

/**
 * @brief Registers the default response to RDM_PID_IDENTIFY_DEVICE requests.
 * This response is required by all RDM-capable devices. It is called when the
 * DMX driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context);


bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context);

/**
 * @brief Registers the default response to RDM_PID_DMX_START_ADDRESS requests.
 * This response is required by all RDM-capable devices which use a DMX address.
 * It is called when the DMX driver is initially installed if the DMX start
 * address is not set to DMX_START_ADDRESS_NONE.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                    void *context);

#ifdef __cplusplus
}
#endif
