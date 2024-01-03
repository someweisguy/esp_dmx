/**
 * @file dmx/include/parameter.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief // TODO
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
int dmx_get_sub_device_count(dmx_port_t dmx_num);

/**
 * @brief Allocates and adds a parameter to the DMX driver. The parameter is
 * heap-allocated. This function is not thread-safe.
 *
 * Each parameter can be identified using its DMX port number, sub-device
 * number, and PID. Each DMX port and sub-device may possess many parameters but
 * sub-devices may only possess one copy of each PID.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID which to add.
 * @param non_volatile True if the parameter should persist after the ESP32 is
 * power-cycled.
 * @param[in] init The value to which the parameter should be initialized or
 * NULL to memset the parameter to zero.
 * @param size The size of the parameter to allocate.
 * @return true if the parameter already existed or was added.
 * @return false if the parameter could not be added.
 */
bool dmx_parameter_add_dynamic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               rdm_pid_t pid, bool non_volatile,
                               const void *init, size_t size);

/**
 * @brief Adds a parameter to the DMX driver. The parameter is statically
 * allocated. This function is not thread-safe.
 *
 * Each parameter can be identified using its DMX port number, sub-device
 * number, and PID. Each DMX port and sub-device may possess many parameters but
 * sub-devices may only possess one copy of each PID.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID which to add.
 * @param non_volatile True if the parameter should persist after the ESP32 is
 * power-cycled.
 * @param[in] data A pointer to the memory which stores the parameter data.
 * @param size The size of the parameter.
 * @return true if the parameter already existed or was added.
 * @return false if the parameter could not be added.
 */
bool dmx_parameter_add_static(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, bool non_volatile, void *data,
                              size_t size);

/**
 * @brief Adds a NULL parameter to the DMX driver. This function is not
 * thread-safe.
 *
 * Each parameter can be identified using its DMX port number, sub-device
 * number, and PID. Each DMX port and sub-device may possess many parameters but
 * sub-devices may only possess one copy of each PID.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID which to add.
 * @return true if the parameter already existed or was added.
 * @return false if the parameter could not be added.
 */
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
void *dmx_parameter_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
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
