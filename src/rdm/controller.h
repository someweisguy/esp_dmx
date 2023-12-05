/**
 * @file controller.h
 * @author Mitch Weisbrod
 * @brief This file contains functions needed to send requests to RDM
 * responders.
 */
#pragma once

#include <stdint.h>

#include "rdm/controller/discovery.h"
#include "rdm/controller/product_info.h"


#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sends an RDM GET software version label request and reads the
 * response, if any.
 *
 * GET software version label are sent without parameter data. If a response is
 * received, the response parameter data will include a software version label
 * in the form of a string up to 32 bytes long.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param[out] software_version_label A pointer to a parameter which will be
 * received in the response.
 * @param size The size of the software_version_label string. Used to prevent
 * buffer overflows.
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, was improperly formatted, or an
 * RDM_RESPONSE_TYPE_ACK was not received.
 */
bool rdm_send_get_software_version_label(dmx_port_t dmx_num,
                                         rdm_header_t *header,
                                         char *software_version_label,
                                         size_t *size, rdm_ack_t *ack);

/**
 * @brief Sends an RDM GET identify device request and reads the response, if
 * any.
 *
 * GET identify device requests are sent without parameter data. If a response
 * is received, the response parameter data will include the state of the
 * identify mode on the responding device.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param[out] identify A pointer to a parameter which will be received in the
 * response.
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, was improperly formatted, or an
 * RDM_RESPONSE_TYPE_ACK was not received.
 */
bool rdm_send_get_identify_device(dmx_port_t dmx_num, rdm_header_t *header,
                                  uint8_t *identify, rdm_ack_t *ack);

/**
 * @brief Sends an RDM SET identify device request and reads the response, if
 * any.
 *
 * SET identify device requests are sent with a byte indicating whether identify
 * mode should be enabled or disabled on the responding device. Responding
 * devices send a response without any parameter data.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param identify True to enable the identify mode on the responding device(s).
 * Must be between 0 and 1 (inclusive).
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, was improperly formatted, or an
 * RDM_RESPONSE_TYPE_ACK was not received.
 */
bool rdm_send_set_identify_device(dmx_port_t dmx_num, rdm_header_t *header,
                                  const uint8_t identify, rdm_ack_t *ack);

/**
 * @brief Sends an RDM GET DMX start address request and reads the response, if
 * any.
 *
 * GET DMX start address requests are sent without parameter data. If a response
 * is received, the response parameter data will include the value of the DMX
 * start address of the responding device.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param[out] dmx_start_address A pointer to a parameter which will be received
 * in the response.
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, was improperly formatted, or an
 * RDM_RESPONSE_TYPE_ACK was not received.
 */
bool rdm_send_get_dmx_start_address(dmx_port_t dmx_num, rdm_header_t *header,
                                    uint16_t *dmx_start_address,
                                    rdm_ack_t *ack);

/**
 * @brief Sends an RDM SET DMX start address request and reads the response, if
 * any.
 *
 * SET DMX start address requests are sent with a 16-bit word indicating to what
 * value the responding device(s) should set their DMX start address. Responding
 * devices send a response without any parameter data.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param dmx_start_address The value to which responding device(s) should set
 * their DMX start address. Must be between 1 and 512 (inclusive).
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, was improperly formatted, or an
 * RDM_RESPONSE_TYPE_ACK was not received.
 */
bool rdm_send_set_dmx_start_address(dmx_port_t dmx_num, rdm_header_t *header,
                                    const uint16_t dmx_start_address,
                                    rdm_ack_t *ack);

#ifdef __cplusplus
}
#endif
