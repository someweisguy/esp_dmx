/**
 * @file rdm/responder/include/queue_status.h
 * @author Mitch Weisbrod
 * @brief This file contains RDM queue and status functions for the RDM
 * responder. The PIDs in RDM queue and status include RDM_PID_QUEUED_MESSAGE,
 * RDM_PID_STATUS_MESSAGES, RDM_PID_STATUS_ID_DESCRIPTION,
 * RDM_PID_CLEAR_STATUS_ID, and RDM_PID_SUB_DEVICE_STATUS_ID_THRESHOLD. This
 * file also includes getters and setters for these functions as appropriate.
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers the default response to RDM_PID_QUEUED_MESSAGE requests.
 * This parameter is used to communicate changes in an RDM responder that were
 * not initiated by the RDM controller. This function is called when the DMX
 * driver is initially installed.
 *
 * @param dmx_num The DMX port number.
 * @param max_count The maximum number of elements that can be stored in the RDM
 * queue.
 * @param cb A callback which is called after receiving a request for this
 * parameter.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the parameter was registered.
 * @return false on failure.
 */
bool rdm_register_queued_message(dmx_port_t dmx_num, uint32_t max_count,
                                 rdm_callback_t cb, void *context);

/**
 * @brief Push a parameter ID to the RDM queue, if it has been instantiated.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to push to the RDM queue.
 * @return true if the PID was pushed to the queue.
 * @return false on failure.
 */
bool rdm_queue_push(dmx_port_t dmx_num, rdm_pid_t pid);

/**
 * @brief Pops a parameter ID from the RDM queue, if it exists.
 *
 * @param dmx_num The DMX port number.
 * @return The PID popped from the RDM queue, or 0 if the queue is empty.
 */
rdm_pid_t rdm_queue_pop(dmx_port_t dmx_num);

/**
 * @brief Gets the number of elements currently in the RDM queue.
 *
 * @param dmx_num The DMX port number.
 * @return The number of elements in the RDM queue or 0 on failure.
 */
uint8_t rdm_queue_size(dmx_port_t dmx_num);

/**
 * @brief Returns the last parameter ID that was successfully popped from the
 * RDM queue. If rdm_queue_pop() has not yet been called, this function returns
 * 0. If rdm_queue_pop() has been called and returned a non-zero value but the
 * most recent call to rdm_queue_pop() returned 0, this function will return the
 * non-zero value.
 *
 * @param dmx_num The DMX port number.
 * @return The last PID returned from rdm_queue_pop() or 0 on failure.
 */
rdm_pid_t rdm_queue_previous(dmx_port_t dmx_num);

#ifdef __cplusplus
}
#endif
