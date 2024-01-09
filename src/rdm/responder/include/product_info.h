/**
 * @file rdm/responder/include/product_info.h
 * @author Mitch Weisbrod
 * @brief This file contains product information functions for the RDM
 * responder. The PIDs in product information include RDM_PID_DEVICE_INFO,
 * RDM_PID_PRODUCT_DETAIL_ID_LIST, RDM_PID_DEVICE_MODEL_DESCRIPTION,
 * RDM_PID_MANUFACTURER_LABEL, RDM_PID_DEVICE_LABEL, RDM_PID_FACTORY_DEFAULTS,
 * RDM_PID_LANGUAGE_CAPABILITIES, RDM_PID_LANGUAGE,
 * RDM_PID_SOFTWARE_VERSION_LABEL, RDM_PID_BOOT_SOFTWARE_VERSION_ID,
 * RDM_PID_BOOT_SOFTWARE_VERSION_LABEL. This file also includes getters and
 * setters for these functions as appropriate.
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers the default response to RDM_PID_DEVICE_INFO requests. This
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
bool rdm_register_device_info(dmx_port_t dmx_num, rdm_callback_t cb,
                              void *context);

/**
 * @brief Gets a copy of the RDM device info of this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] device_info A pointer which stores a copy of the device info of
 * this device.
 * @return The number of bytes that were written to device_info.
 */
size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info);

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
bool rdm_register_device_label(dmx_port_t dmx_num, const char *device_label,
                               rdm_callback_t cb, void *context);

/**
 * @brief Gets the device label.
 *
 * @param dmx_num The DMX port number.
 * @param[out] device_label A pointer to a buffer that the device label will be
 * copied into.
 * @param size The size of the device label.
 * @return The number of bytes copied
 */
size_t rdm_get_device_label(dmx_port_t dmx_num, char *device_label,
                            size_t size);

/**
 * @brief Sets the device label.
 *
 * @param dmx_num The DMX port number.
 * @param[in] device_label A pointer to a buffer which contains the value to
 * which the device label should be set.
 * @param size The size of the device label.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_set_device_label(dmx_port_t dmx_num, const char *device_label,
                          size_t size);
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

/**
 * @brief Gets a copy of the RDM software version label of this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] software_version_label A pointer which stores a copy of the
 * software version label of this device.
 * @param size The size of the software_version_label buffer.
 * @return the number of bytes written to software_version_label.
 */
size_t rdm_get_software_version_label(dmx_port_t dmx_num,
                                      char *software_version_label,
                                      size_t size);

#ifdef __cplusplus
}
#endif
