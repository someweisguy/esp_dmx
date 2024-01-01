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
 * @brief The RDM parameter definition. Defines the capabilities of the
 * parameter for RDM requests.
 */
typedef struct rdm_parameter_definition_t {
  /** @brief The command class that is supported by the parameter.*/
  uint8_t pid_cc;
  /** @brief The data type of the parameter.*/
  uint8_t ds;
  /** @brief The RDM command information.*/
  struct rdm_command_t {
    /** @brief The command information for the request and the response.*/
    struct {
      /** @brief The format of the command.*/
      const char *format;
    } request, response;
    /** @brief The function that is called to handle the RDM request.*/
    size_t (*handler)(dmx_port_t dmx_num,
                      const struct rdm_parameter_definition_t *definition,
                      const rdm_header_t *header);
  } get, set;
  /** @brief The maximum parameter data length for the parameter.*/
  uint8_t pdl_size;
  /** @brief The maximum value of the parameter. If this value is not
     applicable, it should be left 0.*/
  uint32_t max_value;
  /** @brief The minimum value of the parameter. If this value is not
     applicable, it should be left 0.*/
  uint32_t min_value;
  /** @brief The default value of the parameter. If this value is not
     applicable, it should be left 0.*/
  uint32_t default_value;
  /** @brief The unit type for this parameter, one of rdm_unit_t.*/
  uint8_t units;
  /** @brief The prefix for this parameter, one of rdm_prefix_t.*/
  uint8_t prefix;
  /** @brief The ASCII description of the parameter.*/
  const char *description;
} rdm_parameter_definition_t;

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
 * @brief Gets the number of sub-devices that are supported.
 * 
 * @param dmx_num The DMX port number.
 * @return The number of supported sub-devices.
 */
dmx_device_num_t dmx_get_sub_device_count(dmx_port_t dmx_num);

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

/**
 * @brief Returns true if the RDM format string is valid.
 *
 * @param format The RDM format string.
 * @return true if the RDM format string is valid.
 * @return false if it is not valid.
 */
bool dmx_parameter_rdm_format_is_valid(const char *format);

/**
 * @brief Adds an RDM definition to the desired DMX parameter. RDM definitions
 * are copied by pointer; they must be valid throughout the lifetime of the DMX
 * driver. This function is not thread-safe.
 *
 * Adding an RDM definition to a parameter allows for the device to respond to
 * RDM requests for the parameter. If a request is received for a parameter that
 * does not have an RDM definition, the device will respond with
 * RDM_NR_UNKNOWN_PID.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @param definition A pointer to the RDM definition for the parameter.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_parameter_rdm_define(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid,
                              const rdm_parameter_definition_t *definition);

/**
 * @brief Returns a pointer to the RDM definition for the desired DMX parameter.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @return A pointer to the RDM definition or NULL on failure.
 */
const rdm_parameter_definition_t *dmx_parameter_rdm_lookup(
    dmx_port_t dmx_num, rdm_sub_device_t sub_device, rdm_pid_t pid);

/**
 * @brief Sets the callback function and context for requests to the desired
 * sub-device and parameter ID. The callback function is handled after a
 * response to the RDM request is handled.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @param callback A function to be called after a request for the parameter is
 * received.
 * @param context A pointer to the context for the callback.
 * @return true on success.
 * @return false on failure.
 */
bool dmx_parameter_rdm_set_callback(dmx_port_t dmx_num,
                                    rdm_sub_device_t sub_device, rdm_pid_t pid,
                                    rdm_callback_t callback, void *context);

/**
 * @brief Handles the callback for the desired parameter, if it exists.
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @param[inout] request_header A pointer to the header of the RDM request.
 * @param[inout] response_header A pointer to the header of the RDM response.
 * @return true if the parameter exists.
 * @return false if the parameter does not exist.
 */
bool dmx_parameter_rdm_handle_callback(dmx_port_t dmx_num,
                                       rdm_sub_device_t sub_device,
                                       rdm_pid_t pid,
                                       rdm_header_t *request_header,
                                       rdm_header_t *response_header);

#ifdef __cplusplus
}
#endif
