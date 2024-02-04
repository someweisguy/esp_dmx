/**
 * @file rdm/controller/include/utils.h
 * @author Mitch Weisbrod
 * @brief This header includes utility functions that are needed or may be
 * useful for various purposes pertaining to the RDM controller.
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
 * @brief Type for constructing an RDM request. Contains all the necessary
 * information needed to address a request on the RDM bus.
 */
typedef struct rdm_request_t {
  const rdm_uid_t *dest_uid;    // The destination UID of the request.
  rdm_sub_device_t sub_device;  // The target sub-device of the request.
  rdm_cc_t cc;                  // The command class of the request.
  rdm_pid_t pid;                // The parameter ID.
  const char *format;           // The format string for the parameter data.
  const void *pd;  // A pointer to the parameter data of the request.
  size_t pdl;      // The parameter data length of the request.
} rdm_request_t;

/**
 * @brief Sends an RDM controller request and processes the response. This
 * function writes, sends, receives, and reads a request and response RDM
 * packet. It performs error checking on the written packet to ensure that it
 * adheres to RDM specification and prevents RDM bus errors. Any parameter data
 * received in the RDM response must be read by using rdm_read_pd(). An
 * rdm_ack_t is provided to process DMX and RDM errors.
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
 * - ack.pdl, ack.timer and ack.nack_reason are a union which should be read
 * depending on the value of ack.type. If ack.type is RDM_RESPONSE_TYPE_ACK,
 * ack.pdl should be read. ack.pdl is the parameter data length of the RDM
 * response. If ack.type is RDM_RESPONSE_TYPE_ACK_TIMER, ack.timer should be
 * read. ack.timer is the estimated amount of time in FreeRTOS ticks until the
 * responder is able to provide a response to the request. If ack.type is
 * RDM_RESPONSE_TYPE_NACK_REASON, ack.nack_reason should be read to get the NACK
 * reason.
 *
 * @param dmx_num The DMX port number.
 * @param[in] request A pointer to a request constructor.
 * @param[in] format The RDM parameter format string for the response data. More
 * information about RDM parameter format strings can be found in the
 * documentation on the rdm_read_pd() and rdm_write() functions.
 * @param[out] pd A pointer to an array which will store the parameter data
 * received in the response. This value may be NULL if no data is expected.
 * @param size The size of the pd array.
 * @param[out] ack A pointer to an rdm_ack_t which stores information about the
 * RDM response.
 * @return When an RDM_RESPONSE_TYPE_ACK response is received, the response PDL
 * is returned or true if there is no parameter data received. 0 on failure.
 */
size_t rdm_send_request(dmx_port_t dmx_num, const rdm_request_t *request,
                        const char *format, void *pd, size_t size,
                        rdm_ack_t *ack);

/**
 * @brief Get the transaction number of the RDM controller. This number is
 * included in every RDM controller request. It is incremented after every RDM
 * message is sent. This function should only be used when the port is being
 * used as an RDM controller.
 *
 * @param dmx_num The DMX port number.
 * @return The current RDM transaction number or 0 on failure.
 */
uint32_t rdm_get_transaction_num(dmx_port_t dmx_num);

#ifdef __cplusplus
}
#endif