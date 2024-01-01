/**
 * @file rdm/controller/dmx_setup.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include <stdint.h>

#include "dmx/types.h"
#include "rdm/controller.h"
#include "rdm/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

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
 */  // TODO: update docs
size_t rdm_send_get_dmx_start_address(dmx_port_t dmx_num,
                                      const rdm_uid_t *dest_uid,
                                      rdm_sub_device_t sub_device,
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
 */  // TODO: update docs
bool rdm_send_set_dmx_start_address(dmx_port_t dmx_num,
                                    const rdm_uid_t *dest_uid,
                                    rdm_sub_device_t sub_device,
                                    const uint16_t dmx_start_address,
                                    rdm_ack_t *ack);

#ifdef __cplusplus
}
#endif
