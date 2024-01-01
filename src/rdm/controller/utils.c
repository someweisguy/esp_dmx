#include "rdm/controller/include/utils.h"

#include <string.h>

#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "rdm/include/driver.h"
#include "rdm/uid.h"

size_t rdm_send_generic(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                        rdm_sub_device_t sub_device, rdm_pid_t pid, rdm_cc_t cc,
                        const char *format, const void *pd, size_t pdl,
                        rdm_ack_t *ack) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dest_uid != NULL);
  assert(sub_device < RDM_SUB_DEVICE_MAX || sub_device == RDM_SUB_DEVICE_ALL);
  assert(pid > 0);
  assert(rdm_cc_is_valid(cc) && rdm_cc_is_request(cc));
  assert(sub_device != RDM_SUB_DEVICE_ALL || cc == RDM_CC_SET_COMMAND);
  assert(dmx_parameter_rdm_format_is_valid(format));
  assert(format != NULL || pd == NULL);
  assert(pd != NULL || pdl == 0);
  assert(pdl < RDM_PD_SIZE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Attempt to take the mutex and wait until the driver is done sending
  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return 0;
  }
  if (!dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23))) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Write the header using the default arguments and the caller's arguments
  rdm_header_t header = {
      .message_len = 24 + pdl,
      .tn = rdm_get_transaction_num(dmx_num),
      .port_id = dmx_num + 1,
      .message_count = 0,
      .sub_device = sub_device,
      .cc = cc,
      .pid = pid,
      .pdl = pdl,
  };
  memcpy(&header.dest_uid, dest_uid, sizeof(header.dest_uid));
  memcpy(&header.src_uid, rdm_uid_get(dmx_num), sizeof(header.src_uid));

  // Write and send the RDM request
  rdm_write(dmx_num, &header, format, pd);
  if (!dmx_send(dmx_num)) {
    xSemaphoreGiveRecursive(driver->mux);
    if (ack != NULL) {
      ack->err = DMX_OK;
      ack->size = 0;
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->pid = 0;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->message_count = 0;
      ack->pdl = 0;
    }
    return 0;
  }

  // Return early if no response is expected
  if (rdm_uid_is_broadcast(dest_uid) && pid != RDM_PID_DISC_UNIQUE_BRANCH) {
    dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23));
    xSemaphoreGiveRecursive(driver->mux);
    if (ack != NULL) {
      ack->err = DMX_OK;
      ack->size = 0;
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->pid = 0;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->message_count = 0;
      ack->pdl = 0;
    }
    return 0;
  }

  // Attempt to receive the RDM response
  dmx_packet_t packet;
  size_t size = dmx_receive(dmx_num, &packet, pdDMX_MS_TO_TICKS(23));
  if (ack != NULL) {
    ack->err = packet.err;
    ack->size = size;
  }

  // Return early if no response was received
  if (size == 0) {
    xSemaphoreGiveRecursive(driver->mux);
    if (ack != NULL) {
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->pid = 0;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->message_count = 0;
      ack->pdl = 0;
    }
    return 0;
  }

  // Return early if the response checksum was invalid
  if (!rdm_read_header(dmx_num, &header)) {
    xSemaphoreGiveRecursive(driver->mux);
    if (ack != NULL) {
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->pid = 0;
      ack->type = RDM_RESPONSE_TYPE_INVALID;
      ack->message_count = 0;
      ack->pdl = 0;
    }
    return 0;
  }

  // Copy the results into the ack struct
  if (ack != NULL) {
    memcpy(&ack->src_uid, &header.src_uid, sizeof(rdm_uid_t));
    ack->pid = header.pid;
    if (!rdm_response_type_is_valid(header.response_type)) {
      ack->type = RDM_RESPONSE_TYPE_INVALID;
      ack->pdl = header.pdl;
    } else {
      const char *word_format = "w";
      ack->type = header.response_type;
      if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
        uint16_t timer;
        rdm_read_pd(dmx_num, word_format, &timer, sizeof(timer));
        ack->timer = pdDMX_MS_TO_TICKS(timer * 10);
      } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
        uint16_t nack_reason;
        rdm_read_pd(dmx_num, word_format, &nack_reason, sizeof(nack_reason));
        ack->nack_reason = nack_reason;
      } else {
        ack->pdl = header.pdl;
      }
    }
    ack->message_count = header.message_count;
  }

  xSemaphoreGiveRecursive(driver->mux);
  return header.pdl;
}

size_t rdm_get_transaction_num(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  size_t tn;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  tn = dmx_driver[dmx_num]->rdm.tn;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return tn;
}
