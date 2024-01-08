/**
 * @file rdm/responder/include/utils.h
 * @author Mitch Weisbrod
 * @brief This header includes utility functions that are needed or may be
 * useful for various purposes pertaining to the RDM responder.
 */
#pragma once

#include "dmx/include/parameter.h"
#include "dmx/include/types.h"
#include "rdm/include/driver.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

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
 * @brief Returns true if the sub-device exists.
 * 
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @return true if the sub-device exists.
 * @return false if it does not exist.
 */
bool rdm_sub_device_exists(dmx_port_t dmx_num, rdm_sub_device_t sub_device);

/**
 * @brief Writes an ACK packet response to a RDM request packet. This function
 * uses the header of an RDM request packet to write a response. The header for
 * the RDM request must be a valid RDM request header.
 * 
 * @param dmx_num The DMX port number.
 * @param[in] header A pointer to the header of the RDM request packet.
 * @param[in] format The format string of the RDM parameter data. 
 * @param[in] pd A pointer to the parameter data for the RDM ACK packet.
 * @param pdl The parameter data length of the RDM ack packet.
 * @return The number of bytes written.
 */
size_t rdm_write_ack(dmx_port_t dmx_num, const rdm_header_t *header,
                     const char *format, const void *pd, size_t pdl);

/**
 * @brief Writes an NACK packet response to a RDM request packet. This function
 * uses the header of an RDM request packet to write a response. The header for
 * the RDM request must be a valid RDM request header.
 * 
 * @param dmx_num The DMX port number.
 * @param[in] header A pointer to the header of the RDM request packet.
 * @param nack_reason The NACK reason for the RDM response packet.
 * @return The number of bytes written.
 */
size_t rdm_write_nack_reason(dmx_port_t dmx_num, const rdm_header_t *header,
                             rdm_nr_t nack_reason);

/*
// TODO: implement rdm_write_ack_timer()
size_t rdm_write_ack_timer(dmx_port_t dmx_num, const rdm_header_t *header,
                           TickType_t ready_ticks);
*/

/*
// TODO: implement rdm_write_ack_overflow()
size_t rdm_write_ack_overflow(dmx_port_t dmx_num, const rdm_header_t *header,
                              const char *format, const void *pd, size_t pdl,
                              int page);
*/

// TODO: docs
void rdm_set_boot_loader(dmx_port_t dmx_num);

size_t rdm_simple_response_handler(dmx_port_t dmx_num,
                                   const rdm_parameter_definition_t *definition,
                                   const rdm_header_t *header);

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
bool rdm_definition_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
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
const rdm_parameter_definition_t *rdm_definition_get(
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
bool rdm_callback_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                      rdm_pid_t pid, rdm_callback_t callback, void *context);

/**
 * @brief Handles the callback for the desired parameter, if it exists. The
 * callback is given passed the request_header argument, which points to the
 * header of the RDM request. Then the current RDM header is read from the DMX
 * data buffer and passed to the callback as the response header. The callback
 * is also passed a pointer to a user context which is declared in
 * rdm_callback_set().
 *
 * @param dmx_num The DMX port number.
 * @param sub_device The sub-device number.
 * @param pid The parameter ID of the desired parameter.
 * @param[inout] request_header A pointer to the header of the RDM request.
 * @return true if the parameter exists.
 * @return false if the parameter does not exist.
 */
bool rdm_callback_handle(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                         rdm_pid_t pid, rdm_header_t *request_header);

#ifdef __cplusplus
}
#endif
