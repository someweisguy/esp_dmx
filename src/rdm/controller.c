#include "controller.h"

#include <string.h>

#include "dmx/types.h"
#include "endian.h"
#include "esp_log.h"
#include "dmx/hal.h"
#include "dmx/driver.h"
#include "read_write.h"
#include "parameters.h"
#include "utils.h"

static const char *TAG = "rdm_controller";

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

size_t rdm_send(dmx_port_t dmx_num, rdm_header_t *header,
                const rdm_encode_t *encode, rdm_decode_t *decode,
                rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Validate required header information
  if (header->dest_uid == 0 ||
      (header->dest_uid > RDM_MAX_UID && !uid_is_broadcast(header->dest_uid))) {
    ESP_LOGE(TAG, "dest_uid is invalid");
    return 0;
  }
  if (header->cc != RDM_CC_DISC_COMMAND && header->cc != RDM_CC_GET_COMMAND &&
      header->cc != RDM_CC_SET_COMMAND) {
    ESP_LOGE(TAG, "cc is invalid");
    return 0;
  }
  if (header->pid == 0 || header->pid > 0xffff) {
    ESP_LOGE(TAG, "pid is invalid");
    return 0;
  }
  if (header->sub_device > 512 && header->sub_device != RDM_ALL_SUB_DEVICES) {
    ESP_LOGE(TAG, "sub_device is invalid");
    return 0;
  } else if (header->sub_device == RDM_ALL_SUB_DEVICES &&
             header->cc == RDM_CC_GET_COMMAND) {
    ESP_LOGE(TAG, "cannot send RDM_CC_GET_COMMAND to RDM_ALL_SUB_DEVICES");
    return 0;
  }

  // Validate header values that the user doesn't need to include
  if (header->src_uid > RDM_MAX_UID || uid_is_broadcast(header->src_uid)) {
    ESP_LOGE(TAG, "src_uid is invalid");
    return 0;
  } else if (header->src_uid == 0) {
    header->src_uid = rdm_get_uid(dmx_num);
  }
  if (header->port_id < 0 || header->port_id > 255) {
    ESP_LOGE(TAG, "port_id is invalid");
    return 0;
  } else if (header->port_id == 0) {
    header->port_id = dmx_num + 1;
  }

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Set header values that the user cannot set themselves
  taskENTER_CRITICAL(spinlock);
  header->tn = driver->rdm.tn;
  taskEXIT_CRITICAL(spinlock);
  header->message_count = 0;

  // Encode parameter data
  rdm_mdb_t mdb;
  if (encode && encode->function && encode->params && encode->num) {
    encode->function(&mdb, encode->params, encode->num);
  } else {
    mdb.pdl = 0;
  }

  // Take mutex so driver values may be accessed
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Send the request and await the response
  size_t packet_size = rdm_write(dmx_num, header, &mdb);
  dmx_send(dmx_num, packet_size);
  dmx_packet_t packet = {};
  if (!uid_is_broadcast(header->dest_uid) ||
      (header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
       header->cc == RDM_CC_DISC_COMMAND)) {
    packet_size = dmx_receive(dmx_num, &packet, 2);
  }

  // Return early if an error occurred
  if (packet.err) {
    if (ack != NULL) {
      ack->err = packet.err;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->num = 0;
    }
    return packet_size;
  }

  // Process the response data
  if (packet.size > 0) {
    esp_err_t err;
    const rdm_header_t req = *header;
    if (packet.err) {
      err = packet.err;  // Error pass-through
    } else if (!packet.is_rdm) {
      err = ESP_ERR_INVALID_RESPONSE;  // Packet is not RDM
    } else if (!rdm_read(dmx_num, header, &mdb)) {
      err = ESP_ERR_INVALID_RESPONSE;  // Packet is invalid
    } else if (header->response_type != RDM_RESPONSE_TYPE_ACK &&
               header->response_type != RDM_RESPONSE_TYPE_ACK_TIMER &&
               header->response_type != RDM_RESPONSE_TYPE_NACK_REASON &&
               header->response_type != RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
      err = ESP_ERR_INVALID_RESPONSE;  // Response type is invalid
    } else if (!(req.cc == RDM_CC_DISC_COMMAND &&
                 req.pid == RDM_PID_DISC_UNIQUE_BRANCH) &&
               (req.cc != (header->cc & 0x1) || req.pid != header->pid ||
                req.sub_device != header->sub_device || req.tn != header->tn ||
                req.dest_uid != header->src_uid ||
                req.src_uid != header->src_uid)) {
      err = ESP_ERR_INVALID_RESPONSE;  // Response is invalid
    } else {
      err = ESP_OK;
    }

    uint32_t decoded = 0;
    rdm_response_type_t response_type;
    if (!err) {
      response_type = header->response_type;
      if (response_type == RDM_RESPONSE_TYPE_ACK) {
        // Decode the parameter data if requested
        if (mdb.pdl > 0) {
          if (decode && decode->function && decode->params && decode->num) {
            decoded = decode->function(&mdb, decode->params, decode->num);
          } else {
            ESP_LOGW(TAG, "received parameter data but decoder is null");
          }
        }
      } else if (response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
        // Get the estimated response time and convert it to FreeRTOS ticks
        rdm_decode_16bit(&mdb, &decoded, 1);
        decoded = pdMS_TO_TICKS(decoded * 10);
      } else if (response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
        // Get the reported NACK reason
        rdm_decode_16bit(&mdb, &decoded, 1);
      } else {
        // Received RDM_RESPONSE_TYPE_ACK_OVERFLOW
        err = ESP_ERR_NOT_SUPPORTED;  // TODO: implement overflow support
      }
    } else {
      response_type = RDM_RESPONSE_TYPE_NONE;  // Report no response on errors
    }

    // Report the ACK back to the user
    if (ack != NULL) {
      ack->err = err;
      ack->type = response_type;
      ack->num = decoded;
    }

  } else {
    // Wait for request to finish sending if no response is expected
    if (ack != NULL) {
      ack->err = ESP_OK;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->num = 0;
    }
    dmx_wait_sent(dmx_num, 2);
  }

  xSemaphoreGiveRecursive(driver->mux);
  return packet_size;
}

size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num, rdm_header_t *header,
                                   const rdm_disc_unique_branch_t *param,
                                   rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(param != NULL, 0, "param is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->dest_uid = RDM_BROADCAST_ALL_UID;
  header->sub_device = RDM_ROOT_DEVICE;
  header->cc = RDM_CC_DISC_COMMAND;
  header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
  header->src_uid = rdm_get_uid(dmx_num);
  header->port_id = dmx_num + 1;
  
  const rdm_encode_t encode = {
    .function = rdm_encode_uids,
    .params = param,
    .num = 2
  };
  
  return rdm_send(dmx_num, header, &encode, NULL, ack);
}

size_t rdm_send_disc_mute(dmx_port_t dmx_num, rdm_header_t *header,
                          rdm_ack_t *ack, rdm_disc_mute_t *param) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_DISC_COMMAND;
  header->pid = RDM_PID_DISC_MUTE;
  header->src_uid = rdm_get_uid(dmx_num);
  header->port_id = dmx_num + 1;
  
  rdm_decode_t decode = {
    .function = rdm_decode_mute,
    .params = param,
    .num = 1,
  };
  
  return rdm_send(dmx_num, header, NULL, &decode, ack);
}

size_t rdm_send_disc_un_mute(dmx_port_t dmx_num, rdm_header_t *header,
                             rdm_ack_t *ack, rdm_disc_mute_t *param) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_DISC_COMMAND;
  header->pid = RDM_PID_DISC_UN_MUTE;
  header->src_uid = rdm_get_uid(dmx_num);
  header->port_id = dmx_num + 1;
  
  rdm_decode_t decode = {
    .function = rdm_decode_mute,
    .params = param,
    .num = 1,
  };
  
  return rdm_send(dmx_num, header, NULL, &decode, ack);
}
