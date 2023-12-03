/**
 * @file rdm/utils/bus_ctl.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief // TODO
 * 
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*

size_t rdm_write(dmx_num, *header, *format, *pd, size_t n);
size_t rdm_write_ack(dmx_num, *header, *format, *pd, n);
size_t rdm_write_ack_timer(dmx_num, *header, timer);
size_t rdm_write_nack_reason(dmx_num, *header, nr, n)
size_t rdm_write_ack_overflow(dmx_num, *header, *format, *pd, n, page)

size_t rdm_read_pd(dmx_num, *format, *pd, n);

*/




/**
 * @brief Reads an RDM packet from the DMX driver buffer. Header information is
 * emplaced into a header pointer so that it may be read by the caller.
 * Parameter data information needs to be emplaced before it can be properly
 * read by the caller. This function does not perform any data error checking to
 * ensure that the RDM packet is within specification.
 *
 * @param dmx_num The DMX port number.
 * @param[out] header A pointer which stores RDM header information.
 * @param[out] pd A pointer to store parameter data from the RDM packet.
 * @param num The size of the pd pointer. Used to prevent buffer overflows.
 * @return The size of the RDM packet that was read or 0 on error.
 */
bool rdm_read_header(dmx_port_t dmx_num, rdm_header_t *header);

// TODO: docs
size_t rdm_read_pd(dmx_port_t dmx_num, const char *format, void *destination,
                   size_t size);

/**
 * @brief Writes an RDM packet into the DMX driver buffer so it may be sent with
 * dmx_send(). Header information is emplaced into the DMX driver buffer but
 * parameter data information must be emplaced before calling this function to
 * ensure that the RDM packet is properly formatted. This function does not
 * perform any data error checking to ensure that the RDM packet is within
 * specification.
 *
 * @param dmx_num The DMX port number.
 * @param[in] header A pointer which stores RDM header information.
 * @param[in] pd A pointer which stores parameter data to be written.
 * @return The size of the RDM packet that was written or 0 on error.
 */ // TODO: docs update
size_t rdm_write(dmx_port_t dmx_num, const rdm_header_t *header,
                 const char *format, const void *pd);

// TODO: docs
size_t rdm_write_ack(dmx_port_t dmx_num, const rdm_header_t *header,
                     const char *format, const void *pd, size_t pdl);

// TODO: docs
size_t rdm_write_nack_reason(dmx_port_t dmx_num, const rdm_header_t *header, 
                             rdm_nr_t nack_reason);

size_t rdm_send_get(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                    rdm_sub_device_t sub_device, rdm_pid_t pid,
                    const char *format, const void *pd, size_t pdl,
                    rdm_ack_t *ack);

bool rdm_send_set(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                  rdm_sub_device_t sub_device, rdm_pid_t pid,
                  const char *format, const void *pd, size_t pdl,
                  rdm_ack_t *ack);
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
 */
// bool rdm_send_request(dmx_port_t dmx_num, rdm_header_t *header,
//                       const void *pd_in, void *pd_out, size_t *pdl,
//                       rdm_ack_t *ack);

// TODO: docs
void rdm_set_boot_loader(dmx_port_t dmx_num);

// TODO: docs
size_t rdm_get_transaction_num(dmx_port_t dmx_num);

// // TODO: docs
// bool rdm_status_push(dmx_port_t dmx_num, const rdm_status_message_t *message);

// // TODO: docs
// bool rdm_status_pop(dmx_port_t dmx_num, rdm_status_t status,
//                    rdm_status_message_t *message);

// // TODO docs
// void rdm_status_clear(dmx_port_t dmx_num);

// // TODO: docs
// rdm_status_t rdm_status_get_threshold(dmx_port_t dmx_num);

// // TODO: docs
// void rdm_status_set_threshold(dmx_port_t dmx_num, rdm_status_t status);

#ifdef __cplusplus
}
#endif