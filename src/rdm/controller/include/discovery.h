/**
 * @file rdm/controller/include/discovery.h
 * @author Mitch Weisbrod
 * @brief This file contains RDM discovery functions for the RDM controller. The
 * PIDs in discovery include RDM_PID_DISC_UNIQUE_BRANCH, RDM_PID_DISC_MUTE, and
 * RDM_PID_DISC_UN_MUTE. This header also includes a default RDM discovery
 * algorithm with a simple caller and a discovery function which provides a
 * custom callback for more complex behavior.
 */
#pragma once

#include <stdint.h>

#include "dmx/include/types.h"
#include "rdm/controller.h"
#include "rdm/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief A callback function type for use with rdm_discover_with_callback().
 *
 * @param dmx_num The DMX port number.
 * @param uid A pointer to a UID of the found device.
 * @param num_found The number of devices that have been found by discovery.
 * @param[out] mute A pointer to the mute parameter received by the device.
 * @param[inout] context A pointer to a user context.
 */
typedef void (*rdm_disc_cb_t)(dmx_port_t dmx_num, rdm_uid_t uid, int num_found,
                              const rdm_disc_mute_t *mute, void *context);

/**
 * @brief Sends an RDM discovery unique branch request and reads the response,
 * if any.
 *
 * Discovery unique branch requests include two UIDs as the parameter data. The
 * UIDs describe an upper and lower bound for an address space to search. Any
 * devices whose UID falls within this address space must respond to the request
 * unless previously muted by discovery mute requests.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param[in] branch A pointer to a parameter which will be sent in the request.
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, was improperly formatted, or an
 * RDM_RESPONSE_TYPE_ACK was not received.
 */
bool rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                 const rdm_disc_unique_branch_t *branch,
                                 rdm_ack_t *ack);

/**
 * @brief Sends an RDM discovery mute request and reads the response, if any.
 *
 * Discovery mute requests are sent without parameter data. If a response is
 * received, the response parameter data will include information about the
 * responding device. Devices which are muted with a discovery mute request will
 * not respond to discovery unique branch requests.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param[out] mute A pointer to a parameter which will be received in the
 * response.
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, the response was improperly
 * formatted, or an RDM_RESPONSE_TYPE_ACK was not received.
 */
bool rdm_send_disc_mute(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                        rdm_disc_mute_t *mute, rdm_ack_t *ack);

/**
 * @brief Sends an RDM discovery un-mute request and reads the response, if any.
 *
 * Discovery un-mute requests are sent without parameter data. If a response is
 * received, the response parameter data will include information about the
 * responding device. Devices which are un-muted with a discovery mute request
 * may respond to discovery unique branch requests.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer to an RDM header which includes information
 * about where to address the request.
 * @param[out] mute A pointer to a parameter which will be received in the
 * response.
 * @param[out] ack A pointer to an ACK struct which contains information about
 * the response, including information if no response is received.
 * @return true if a properly formatted RDM_RESPONSE_TYPE_ACK was received.
 * @return false if no response was received, the response was improperly
 * formatted, or an RDM_RESPONSE_TYPE_ACK was not received.
 */
bool rdm_send_disc_un_mute(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                           rdm_disc_mute_t *mute, rdm_ack_t *ack);

/**
 * @brief Performs the RDM device discovery algorithm and executes a callback
 * function whenever a new device is discovered.
 *
 * @note The discovery algorithm written for this library is based on the
 * discovery algorithm detailed in the RDM standards document. It differs from
 * the standards document algorithm because it is iterative instead of
 * recursive. This significantly reduces the memory needed to perform the
 * discovery algorithm which allows it to be safely performed on an embedded
 * platform. However, the iterative algorithm still requires the allocation of
 * 588 bytes. By default, this is heap allocated but may be allocated on the
 * stack by configuring settings in this library's Kconfig.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback function which is called when a new device is found.
 * @param[inout] context Context which is passed to the callback function when a
 * new device is found.
 * @return The number of devices found.
 */
int rdm_discover_with_callback(dmx_port_t dmx_num, rdm_disc_cb_t cb,
                               void *context);

/**
 * @brief Performs the RDM device discovery algorithm with a default callback
 * function to store the UIDs of found devices in an array.
 *
 * @note The discovery algorithm written for this library is based on the
 * discovery algorithm detailed in the RDM standards document. It differs from
 * the standards document algorithm because it is iterative instead of
 * recursive. This significantly reduces the memory needed to perform the
 * discovery algorithm which allows it to be safely performed on an embedded
 * platform. However, the iterative algorithm still requires the allocation of
 * 588 bytes. By default, this is heap allocated but may be allocated on the
 * stack by configuring settings in this library's Kconfig.
 *
 * @param dmx_num The DMX port number.
 * @param[out] uids An array of UIDs used to store found device UIDs.
 * @param num The number of elements of the provided UID array.
 * @return The number of devices found.
 */
int rdm_discover_devices_simple(dmx_port_t dmx_num, rdm_uid_t *uids,
                                unsigned int num);

#ifdef __cplusplus
}
#endif
