/**
 * @file utils.h
 * @author Mitch Weisbrod
 * @brief // TODO
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
 * @brief Sends an RDM controller request and processes the response. This
 * function writes, sends, receives, and reads a request and response RDM
 * packet. It performs error checking on the written packet to ensure that it
 * adheres to RDM specification and prevents RDM bus errors. An rdm_ack_t is
 * provided to process DMX and RDM errors.
 * - ack.err will evaluate to true if an error occurred during the sending or
 *   receiving of raw DMX data. RDM data will not be processed if an error
 *   occurred. If a response was expected but none was received, ack.err will
 *   evaluate to DMX_ERR_TIMEOUT. If no response was expected, ack.err will be
 *   set to DMX_OK.
 * - ack.size is the size of the received RDM packet, including the RDM
 *   checksum.
 * - ack.src_uid is the UID of the device which responds to the request.
 * - ack.pid is the PID of the RDM response packet. This is typically the same
 *   as the PID which was sent in the RDM request, but may be a different value
 *   in certain responses.
 * - ack.type will evaluate to RDM_RESPONSE_TYPE_INVALID if an invalid
 *   response is received but does not necessarily indicate a DMX error
 *   occurred. If no response is received ack.type will be set to
 *   RDM_RESPONSE_TYPE_NONE whether or not a response was expected. Otherwise,
 *   ack.type will be set to the ack type received in the RDM response.
 * - ack.message_count indicates the number of messages that are waiting to be
 *   retrieved from the responder's message queue.
 * - ack.timer and ack.nack_reason are a union which should be read depending on
 *   the value of ack.type. If ack.type is RDM_RESPONSE_TYPE_ACK_TIMER,
 *   ack.timer should be read. ack.timer is the estimated amount of time in
 *   FreeRTOS ticks until the responder is able to provide a response to the
 *   request. If ack.type is RDM_RESPONSE_TYPE_NACK_REASON, ack.nack_reason
 *   should be read to get the NACK reason.
 *
 * @param dmx_num The DMX port number.
 * @param[inout] header A pointer which stores header information for the RDM
 * request.
 * @param[in] pd_in A pointer which stores parameter data to be written.
 * @param[out] pd_out A pointer which stores parameter data which was read, if a
 * response was received. This can be an alias of pd_in.
 * @param[inout] pdl The size of the pd_out buffer. When receiving data, this is
 * set to the PDL of the received data. Used to prevent buffer overflows.
 * @param[out] ack A pointer to an rdm_ack_t which stores information about the
 * RDM response.
 * @return true if an RDM_RESPONSE_TYPE_ACK response was received.
 * @return false if any other response type was received.
 */ // TODO: docs
size_t rdm_send_generic(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                        rdm_sub_device_t sub_device, rdm_pid_t pid, rdm_cc_t cc,
                        const char *format, const void *pd, size_t pdl,
                        rdm_ack_t *ack);

// TODO: docs
size_t rdm_get_transaction_num(dmx_port_t dmx_num);

#ifdef __cplusplus
}
#endif