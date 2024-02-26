/**
 * @file dmx/include/parameter.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file contains functions which are used for declaring information
 * about DMX devices and parameters. Parameters can include information about
 * DMX start address, DMX personality, DMX footprints, or other information. DMX
 * parameters are closely related to RDM parameters and use RDM parameter IDs.
 * To register RDM parameter definitions, see the
 * rdm/responder/include/parameter.h header.
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets the number of sub-devices that are supported.
 *
 * @param dmx_num The DMX port number.
 * @return The number of supported sub-devices.
 */
int dmx_sub_device_get_count(dmx_port_t dmx_num);

/**
 * @brief Returns true if the sub-device exists.
 *
 * @param dmx_num The DMX port number.
 * @param device_num The sub-device number.
 * @return true if the sub-device exists.
 * @return false if it does not exist.
 */
bool dmx_sub_device_exists(dmx_port_t dmx_num, dmx_device_num_t device_num);

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

/**
 * @brief Get the PID of the parameter at a given index.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param index The index of the parameter to get.
 * @return The PID of the parameter at the index or 0 if the parameter does not
 * exist.
 */
rdm_pid_t dmx_parameter_at(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           uint32_t index);

/**
 * @brief Get the size of the desired parameter.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @return The size of the parameter.
 */
size_t dmx_parameter_size(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid);

/**
 * @brief Get a pointer to the desired parameter. The returned pointer, if it
 * is valid, is not thread-safe.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @return A pointer to the desired parameter or NULL on failure.
 */
void *dmx_parameter_get_data(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             rdm_pid_t pid);

/**
 * @brief Copies the desired parameter to a destination buffer. Unlike
 * dmx_parameter_get(), this function is thread-safe. Because the parameter is
 * copied to a buffer, updates to the parameter will not affect the copied
 * parameter.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @param[out] destination The destination buffer into which to copy the
 * parameter.
 * @param size The size of the destination buffer.
 * @return The number of bytes that were copied into the destination buffer.
 */
size_t dmx_parameter_copy(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid, void *destination, size_t size);

/**
 * @brief Sets the value of a desired parameter.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @param[in] source The value to which to set the parameter.
 * @param size The size of the source buffer.
 * @return The number of bytes written to the parameter.
 */
size_t dmx_parameter_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                         rdm_pid_t pid, const void *source, size_t size);

/**
 * @brief Commits any updated non-volatile parameters to non-volatile storage.
 * Because committing non-volatile parameters can take some time, this function
 * commits only one parameter per function call. This function is not
 * thread-safe.
 *
 * @param dmx_num The DMX port number.
 * @return The parameter ID of the parameter that was committed or 0 if there
 * are no parameters waiting to be committed.
 */
rdm_pid_t dmx_parameter_commit(dmx_port_t dmx_num);

#ifdef __cplusplus
}
#endif
