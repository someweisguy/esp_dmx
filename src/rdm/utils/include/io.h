/**
 * @file rdm/utils/bus_ctl.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief // TODO
 * 
 */
#pragma once

#include "dmx/types.h"
#include "rdm/controller.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

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