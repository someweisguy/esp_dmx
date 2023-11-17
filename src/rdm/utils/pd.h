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

/**
 * @brief Adds a new parameter to the RDM parameter manager.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to add.
 * @param sub_device The sub-device to which to add the parameter.
 * @param[in] def A pointer to a struct containing a definition of the
 * parameter. This definition is copied into the RDM parameter manager. It does
 * not need to be valid throughout the lifetime of the DMX driver.
 * @param[in] init_value The initial value of the RDM parameter.
 * @return A pointer to the parameter data within the RDM parameter manager on
 * success, NULL on failure.
 */
const void *rdm_pd_add_new(dmx_port_t dmx_num, rdm_pid_t pid,
                           rdm_sub_device_t sub_device,
                           const rdm_pd_definition_t *def,
                           const void *init_value);

/**
 * @brief Adds a parameter to the RDM parameter manager as an alias of a
 * currently existing RDM parameter. This is used for RDM parameters which are
 * contained within other RDM parameters. For example, RDM_PID_DMX_START_ADDRESS
 * is contained with RDM_PID_DEVICE_INFO. The parameter which is aliased must
 * exist within the parameter manager before the aliasing parameter may be
 * added.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to add.
 * @param sub_device The sub-device to which to add the parameter.
 * @param[in] def A pointer to a struct containing a definition of the
 * parameter. This definition is copied into the RDM parameter manager. It does
 * not need to be valid throughout the lifetime of the DMX driver.
 * @param alias The parameter ID of the parameter which is being aliased.
 * @param offset The relative offset of the aliased data. It is recommended to
 * use the offsetof() macro to define data offsets.
 * @return A pointer to the parameter data within the RDM parameter manager on
 * success, NULL on failure.
 */
const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_pid_t pid,
                             rdm_sub_device_t sub_device,
                             const rdm_pd_definition_t *def, rdm_pid_t alias,
                             size_t offset);

/**
 * @brief Adds a deterministic parameter to the RDM parameter manager. A
 * deterministic parameter does not occupy any space in the RDM parameter
 * manager. Instead, its value is computed when it is requested by an RDM
 * controller.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to add.
 * @param sub_device The sub-device to which to add the parameter.
 * @param[in] def A pointer to a struct containing a definition of the
 * parameter. This definition is copied into the RDM parameter manager. It does
 * not need to be valid throughout the lifetime of the DMX driver.
 * @return true on success.
 * @return false on failure.
 */
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
size_t rdm_pd_list(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                   void *destination, size_t size);

// TODO: docs
int rdm_pd_call_response_handler(dmx_port_t dmx_num, rdm_header_t *header,
                                 void *pd, uint8_t *pdl_out);

// TODO: docs
int rdm_response_handler_simple(dmx_port_t dmx_num, rdm_header_t *header,
                                void *pd, uint8_t *pdl_out,
                                const rdm_pd_schema_t *schema);
/**
 * @brief Converts RDM data from a format which can be accessed in code to a
 * format which is compatible with the RDM bus. This is needed because the RDM
 * bus requires all data to be big-endian. The destination and source buffers
 * may overlap.
 *
 * Parameter fields are serialized using a format string. This provides the
 * instructions on how data is written. Fields are written in the order provided
 * in the format string. The following characters can be used to write parameter
 * data:
 * - 'b' writes an 8-bit byte of data.
 * - 'w' writes a 16-bit word of data.
 * - 'd' writes a 32-bit dword of data.
 * - 'u' writes a 48-bit UID.
 * - 'v' writes an optional 48-bit UID if the UID is not 0000:00000000. Optional
 *   UIDs must be at the end of the format string.
 * - 'a' writes an ASCII string. ASCII strings may be up to 32 characters long
 *   and may or may not be null-terminated. An ASCII string must be at the end
 *   of the format string.
 *
 * Integer literals may be written by beginning the integer with '#' and writing
 * the literal in hexadecimal form. Integer literals must be terminated with an
 * 'h' character. For example, the integer 0xbeef is represented as "#beefh".
 * Integer literals are written regardless of what the underlying value is. This
 * is used for situations such as serializing a rdm_device_info_t wherein the
 * first two bytes are 0x01 and 0x00.
 *
 * Parameters will continue to be serialized as long as the number of bytes
 * written does not exceed the size of the destination buffer, as provided in
 * the num argument. A single parameter may be written instead of multiple by
 * including a '$' character at the end of the format string.
 *
 * To deserialize data, the function rdm_pd_deserialize() should be used.
 *
 * Example format strings and their corresponding PIDs are included below.
 *
 * RDM_PID_DISC_UNIQUE_BRANCH: "uu$"
 * RDM_PID_DISC_MUTE: "wv$"
 * RDM_PID_DEVICE_INFO: "#0100hwwdwbbwwb$"
 * RDM_PID_SOFTWARE_VERSION_LABEL: "a$"
 * RDM_PID_DMX_START_ADDRESS: "w$"
 *
 * @param[out] destination The destination into which to serialize the data.
 * @param len The maximum number of bytes to write.
 * @param[in] format The format string which instructs the function how to
 * format data.
 * @param[in] source The source buffer which is serialized into the destination.
 * @return The size of the data that was written.
 */
size_t rdm_pd_serialize(void *destination, size_t len, const char *format,
                        const void *source);

/**
 * @brief Converts RDM data from a format which can be read from the RDM bus to
 * a format which can be accessed in code. This is needed because the RDM bus
 * requires all data to be big-endian. The destination and source buffers may
 * overlap.
 *
 * Parameter fields are serialized using a format string. This provides the
 * instructions on how data is written. Fields are written in the order provided
 * in the format string. The following characters can be used to write parameter
 * data:
 * - 'b' writes an 8-bit byte of data.
 * - 'w' writes a 16-bit word of data.
 * - 'd' writes a 32-bit dword of data.
 * - 'u' writes a 48-bit UID.
 * - 'v' writes an optional 48-bit UID if the UID is not 0000:00000000. Optional
 *   UIDs must be at the end of the format string.
 * - 'a' writes an ASCII string. ASCII strings may be up to 32 characters long
 *   and may or may not be null-terminated. An ASCII string must be at the end
 *   of the format string.
 *
 * Integer literals may be written by beginning the integer with '#' and writing
 * the literal in hexadecimal form. Integer literals must be terminated with an
 * 'h' character. For example, the integer 0xbeef is represented as "#beefh".
 * Integer literals are written regardless of what the underlying value is. This
 * is used for situations such as serializing a rdm_device_info_t wherein the
 * first two bytes are 0x01 and 0x00.
 *
 * Parameters will continue to be deserialized as long as the number of bytes
 * written does not exceed the size of the destination buffer, as provided in
 * the num argument. A single parameter may be written instead of multiple by
 * including a '$' character at the end of the format string.
 *
 * To serialize data, the function rdm_pd_serialize() should be used.
 *
 * Example format strings and their corresponding PIDs are included below.
 *
 * RDM_PID_DISC_UNIQUE_BRANCH: "uu$"
 * RDM_PID_DISC_MUTE: "wv$"
 * RDM_PID_DEVICE_INFO: "#0100hwwdwbbwwb$"
 * RDM_PID_SOFTWARE_VERSION_LABEL: "a$"
 * RDM_PID_DMX_START_ADDRESS: "w$"
 *
 * @param[out] destination The destination into which to deserialize the data.
 * @param len The maximum number of bytes to write.
 * @param[in] format The format string which instructs the function how to
 * format data.
 * @param[in] source The source buffer which is deserialized into the
 * destination.
 * @return The size of the data that was written.
 */
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
