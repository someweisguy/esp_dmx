/**
 * @file rdm/controller/device_control.h
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
 */  // TODO: update docs
size_t rdm_send_get_identify_device(dmx_port_t dmx_num,
                                    const rdm_uid_t *dest_uid,
                                    rdm_sub_device_t sub_device, bool *identify,
                                    rdm_ack_t *ack);

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
 */  // TODO: update docs
bool rdm_send_set_identify_device(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                                  rdm_sub_device_t sub_device,
                                  const uint8_t identify, rdm_ack_t *ack);

#ifdef __cplusplus
}
#endif
