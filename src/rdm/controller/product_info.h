/**
 * @file rdm/controller/disc.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include <stdint.h>

#include "dmx/types.h"
#include "rdm/parameters.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sends an RDM GET device info request and reads the response, if any.
 *
 * GET device info requests are sent without parameter data. If a response is
 * received, the response parameter data will include device information about
 * the responding device.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param[out] device_info A pointer to a parameter which will be received in
 * the response.
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, the response was improperly
 * formatted, or an RDM_RESPONSE_TYPE_ACK was not received.
 */  // TODO: update docs
size_t rdm_send_get_device_info(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                              rdm_sub_device_t sub_device,
                              rdm_device_info_t *device_info, rdm_ack_t *ack);

/**
 * @brief Gets a copy of the RDM device info of this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] device_info A pointer which stores a copy of the device info of
 * this device.
 * @return true on success.
 * @return false on failure.
 */ // TODO: update docs
size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info);


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
 */ // TODO: update docs
size_t rdm_send_get_software_version_label(dmx_port_t dmx_num,
                                           const rdm_uid_t *dest_uid,
                                           rdm_sub_device_t sub_device,
                                           char *software_version_label,
                                           size_t size, rdm_ack_t *ack);

/**
 * @brief Gets a copy of the RDM software version label of this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] software_version_label A pointer which stores a copy of the
 * software version label of this device.
 * @param[inout] size A pointer to the size of the software_version_label
 * buffer. Is set to the size of the software_version_label on success.
 * @return true on success.
 * @return false on failure.
 */ // TODO: update docs
size_t rdm_get_software_version_label(dmx_port_t dmx_num,
                                      char *software_version_label,
                                      size_t size);

#ifdef __cplusplus
}
#endif
