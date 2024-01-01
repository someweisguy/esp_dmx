/**
 * @file controller.h
 * @author Mitch Weisbrod
 * @brief This file contains functions needed to send requests to RDM
 * responders.
 */
#pragma once

#include "rdm/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Provides information about RDM responses.*/
typedef struct rdm_ack_t {
  /** @brief Evaluates to true if an error occurred reading DMX data.*/
  dmx_err_t err;
  /** @brief The size of the packet received.*/
  size_t size;
  /** @brief The UID of the device originating the response packet.*/
  rdm_uid_t src_uid;
  /** @brief The PID of the response packet. It is typically the same PID as the
   * RDM request. */
  rdm_pid_t pid;
  /** @brief The type of the RDM response received.*/
  rdm_response_type_t type;
  /** @brief The message count field is used by a responder to indicate that
       additional data is now available for collection by a controller.*/
  int message_count;
  union {
    /** @brief The parameter data length (PDL) is the number of slots included
     in the parameter data area that it precedes.*/
    size_t pdl;
    /** @brief The amount of time in FreeRTOS ticks until the responder device
       will be ready to respond to the request. This field should be read when
       the response type received is RDM_RESPONSE_TYPE_ACK_TIMER.*/
    TickType_t timer;
    /** @brief The reason that the request was unable to be fulfilled. This
       field should be read when the response type received is
       RDM_RESPONSE_TYPE_NACK_REASON.*/
    rdm_nr_t nack_reason;
  };
} rdm_ack_t;

#ifdef __cplusplus
}
#endif

#include "rdm/controller/include/device_control.h"
#include "rdm/controller/include/discovery.h"
#include "rdm/controller/include/dmx_setup.h"
#include "rdm/controller/include/product_info.h"
