#include "esp_rdm.h"

#include <stdint.h>
#include <string.h>

#include "dmx_constants.h"
#include "dmx_types.h"
#include "endian.h"
#include "esp_check.h"
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "private/driver.h"
#include "private/rdm_encode/types.h"
#include "private/rdm_encode/functions.h"
#include "rdm_constants.h"

// Used for argument checking at the beginning of each function.
#define RDM_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

static const char *TAG = "rdm";  // The log tagline for the file.

rdm_uid_t rdm_get_uid(dmx_port_t dmx_num) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // TODO: enter spinlock

  // Initialize the RDM UID 
  if (driver->rdm.uid == 0) {
    struct __attribute__((__packed__)) {
      uint16_t manufacturer;
      uint64_t device;
    } mac;
    esp_efuse_mac_get_default((void *)&mac);
    driver->rdm.uid = (bswap32(mac.device) + dmx_num) & 0xffffffff;
    driver->rdm.uid |= (rdm_uid_t)RDM_DEFAULT_MAN_ID << 32;
  }
  rdm_uid_t uid = driver->rdm.uid;

  // TODO: exit spinlock

  return uid;
}

void rdm_set_uid(dmx_port_t dmx_num, rdm_uid_t uid) { 
  RDM_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  RDM_CHECK(uid <= RDM_MAX_UID, , "uid error");
  
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // TODO: enter spinlock
  driver->rdm.uid = uid;  
  // TODO: exit spinlock
}

bool rdm_is_muted() { 
  // FIXME
  return false;
}

size_t rdm_send_disc_response(dmx_port_t dmx_num, rdm_uid_t uid) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");


  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Write and send the response
  size_t written = rdm_encode_disc_response(driver->data.buffer, 24, uid);
  dmx_send(dmx_num, written);

  xSemaphoreGiveRecursive(driver->mux);
  return written;
}

size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                   rdm_disc_unique_branch_t *params,
                                   rdm_response_t *response, rdm_uid_t *uid) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(params != NULL, 0, "params is null");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Prepare the RDM message
  rdm_data_t *rdm = (rdm_data_t *)driver->data.buffer;
  size_t written = rdm_encode_uids(&rdm->pd, (rdm_uid_t *)params, 2);
  rdm_header_t header = {
    .destination_uid = RDM_BROADCAST_ALL_UID,
    .source_uid = rdm_get_uid(dmx_num),
    .tn = 0,  // TODO: get up-to-date transaction number
    .port_id = dmx_num + 1,
    .message_count = 0,
    .sub_device = 0,
    .cc = RDM_CC_DISC_COMMAND, 
    .pid = RDM_PID_DISC_UNIQUE_BRANCH, 
    .pdl = written,
  };
  written += rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Initialize the response to the default values
  if (response != NULL) {
    response->err = ESP_OK;
    response->type = RDM_RESPONSE_TYPE_NONE;
    response->num_params = 0;
  }

  // Wait for a response
  size_t num_params = 0;
  dmx_event_t packet;
  const size_t read = dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK);
  if (packet.err) {
    response->err = packet.err;
  } else if (read) {
    // Check the packet for errors
    if (!rdm_decode_disc_response(driver->data.buffer, uid)) {
      response->err = ESP_ERR_INVALID_RESPONSE;
    }

    num_params = 1;
    response->num_params = num_params;
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
}

size_t rdm_send_disc_mute(dmx_port_t dmx_num, rdm_uid_t uid, bool mute,
                          rdm_response_t *response,
                          rdm_disc_mute_t *params) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Determine which PID to use (mute and un-mute are different PIDs)
  const rdm_pid_t pid = mute ? RDM_PID_DISC_MUTE : RDM_PID_DISC_UN_MUTE;

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Write and send the RDM message
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  rdm_header_t header = {
    .destination_uid = uid,
    .source_uid = rdm_get_uid(dmx_num),
    .tn = 0, // TODO: get up-to-date transaction number
    .port_id = dmx_num + 1,
    .message_count = 0,
    .sub_device = 0,
    .cc = RDM_CC_DISC_COMMAND, 
    .pid = pid,
    .pdl = 0 
  };
  size_t written = rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Initialize the response to the default values
  if (response != NULL) {
    response->err = ESP_OK;
    response->type = RDM_RESPONSE_TYPE_NONE;
    response->num_params = 0;
  }

  // Determine if a response is expected
  size_t num_params = 0;
  if (!RDM_UID_IS_BROADCAST(uid)) {
    // Receive the response
    dmx_event_t packet;
    const size_t read = dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK);
    if (packet.err) {
      response->err = packet.err;
    } else if (read) {
      // Check the packet for errors
      if (!rdm_decode_header(driver->data.buffer, &header)) {
        response->err = ESP_ERR_INVALID_RESPONSE;
      } else if (!header.checksum_is_valid) {
        response->err = ESP_ERR_INVALID_CRC;
      }
      // TODO: error checking of packet -- check pid, cc, and ACK?
      
      // Decode the response
      if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
        num_params = rdm_decode_mute(&rdm->pd, params, header.pdl);
        response->num_params = num_params;
      }     
    }
  } else {
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
}

size_t rdm_discover_with_callback(dmx_port_t dmx_num, rdm_discovery_cb_t cb,
                                  void *context) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Allocate the instruction stack. The max binary tree depth is 49
#ifndef CONFIG_RDM_STATIC_DEVICE_DISCOVERY
  rdm_disc_unique_branch_t *stack;
  stack = malloc(sizeof(rdm_disc_unique_branch_t) * 49);
  if (stack == NULL) {
    ESP_LOGE(TAG, "Discovery malloc error");
    return 0;
  }
#else
  rdm_disc_unique_branch_t stack[49];  // 784B - use with caution!
#endif

  dmx_driver_t *restrict const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Un-mute all devices
  rdm_send_disc_mute(dmx_num, RDM_BROADCAST_ALL_UID, false, NULL, NULL);
  // Initialize the stack with the initial branch instruction
  size_t stack_size = 1;
  stack[0].lower_bound = 0;
  stack[0].upper_bound = RDM_MAX_UID;

  size_t num_found = 0;
  while (stack_size > 0) {
    rdm_disc_unique_branch_t *branch = &stack[--stack_size];
    size_t attempts = 0;
    rdm_response_t response;
    rdm_uid_t uid;

    if (branch->lower_bound == branch->upper_bound) {
      // Can't branch further so attempt to mute the device
      uid = branch->lower_bound;
      rdm_disc_mute_t mute;
      do {
        rdm_send_disc_mute(dmx_num, uid, true, &response, &mute);
      } while (response.num_params == 0 && ++attempts < 3);

      // Attempt to fix possible error where responder is flipping its own UID
      if (response.num_params == 0) {
        uid = bswap64(uid) >> 16;  // Flip UID
        rdm_send_disc_mute(dmx_num, uid, true, &response, &mute);
      }

      // Add the UID to the list
      if (response.num_params > 0 && !response.err) {
        if (mute.binding_uid) {
          uid = mute.binding_uid;
        }
        cb(dmx_num, uid, num_found, context);
        ++num_found;
      }
    } else {
      // Search the current branch in the RDM address space
      do {
        rdm_send_disc_unique_branch(dmx_num, branch, &response, &uid);
      } while (response.num_params == 0 && ++attempts < 3);
      if (response.num_params > 0) {
        bool devices_remaining = true;

#ifndef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
        /*
        Stop the RDM controller from branching all the way down to the
        individual address if it is not necessary. When debugging, this code
        should not be called as it can hide bugs in the discovery algorithm. 
        Users can use the sdkconfig to enable or disable discovery debugging.
        */
        if (!response.err) {
          for (int quick_finds = 0; quick_finds < 3; ++quick_finds) {
            // Attempt to mute the device
            attempts = 0;
            rdm_disc_mute_t mute;
            do {
              rdm_send_disc_mute(dmx_num, uid, true, &response, &mute);
            } while (response.num_params == 0 && ++attempts < 3);

            // Add the UID to the list
            if (response.num_params > 0) {
              if (mute.binding_uid) {
                uid = mute.binding_uid;
              }
              cb(dmx_num, uid, num_found, context);
              ++num_found;
            }

            // Check if there are more devices in this branch
            attempts = 0;
            do {
              rdm_send_disc_unique_branch(dmx_num, branch, &response, &uid);
            } while (response.num_params == 0 && ++attempts < 3);
            if (response.num_params > 0 && response.err) {
              // There are more devices in this branch - branch further
              devices_remaining = true;
              break;
            } else {
              // There are no more devices in this branch
              devices_remaining = false;
              break;
            }
          }
        }
#endif

        // Recursively search the next two RDM address spaces
        if (devices_remaining) {
          const rdm_uid_t lower_bound = branch->lower_bound;
          const rdm_uid_t mid = (lower_bound + branch->upper_bound) / 2;

          // Add the upper branch so that it gets handled second
          stack[stack_size].lower_bound = mid + 1;
          ++stack_size;

          // Add the lower branch so it gets handled first
          stack[stack_size].lower_bound = lower_bound;
          stack[stack_size].upper_bound = mid;
          ++stack_size;
        }
      }
    }
  }

  xSemaphoreGiveRecursive(driver->mux);

#ifndef CONFIG_RDM_STATIC_DEVICE_DISCOVERY
  free(stack);
#endif

  return num_found;
}

struct rdm_disc_default_ctx {
  size_t size;
  rdm_uid_t *uids;
};

static void rdm_disc_cb(dmx_port_t dmx_num, rdm_uid_t uid, size_t num_found,
                        void *context) {
  struct rdm_disc_default_ctx *c = (struct rdm_disc_default_ctx *)context;
  if (num_found < c->size && c->uids != NULL) {
    c->uids[num_found] = uid;
  }
}

size_t rdm_discover_devices(dmx_port_t dmx_num, rdm_uid_t *uids,
                            const size_t size) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  struct rdm_disc_default_ctx context = {.size = size, .uids = uids};
  size_t found = rdm_discover_with_callback(dmx_num, &rdm_disc_cb, &context);

  return found;
}

size_t rdm_get_supported_parameters(dmx_port_t dmx_num, rdm_uid_t uid,
                                    uint16_t sub_device,
                                    rdm_response_t *response, rdm_pid_t *pids,
                                    size_t size) {
  // TODO
  return 0;
}

size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_uid_t uid,
                           uint16_t sub_device, rdm_response_t *response,
                           rdm_device_info_t *param) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(!RDM_UID_IS_BROADCAST(uid), 0, "uid cannot be broadcast");
  // TODO: more arg checks

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  // No parameter data to send
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_GET_COMMAND,
                         .pid = RDM_PID_DEVICE_INFO,
                         .pdl = 0};
  size_t written = rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  size_t num_params = 0;
  dmx_event_t event;
  const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
  if (event.err) {
    response->err = event.err;
    response->num_params = 0;
  } else if (read) {
    // Parse the response to ensure it is valid
    if (!rdm_decode_header(driver->data.buffer, &header)) {
      response->err = ESP_ERR_INVALID_RESPONSE;
    } else if (!header.checksum_is_valid) {
      response->err = ESP_ERR_INVALID_CRC;
    } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
      response->err = ESP_ERR_INVALID_ARG;
    } else {
      response->err = ESP_OK;
    }

    // Handle the parameter data
    response->type = header.response_type;
    if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
      // Decode the parameter data
      num_params = rdm_decode_device_info(&rdm->pd, param);
      response->num_params = num_params;
    } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
      // Get the estimated response time and convert it to FreeRTOS ticks
      uint32_t estimated_response_time;
      rdm_decode_16bit(&rdm->pd, &estimated_response_time, 1);
      response->timer = pdMS_TO_TICKS(estimated_response_time * 10);
    } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
      // Report the NACK reason
      rdm_decode_16bit(&rdm->pd, &response->nack_reason, 1);
    } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
      // This code should never run
      response->err = ESP_ERR_INVALID_RESPONSE;
    } else {
      // An unknown response type was received
      response->err = ESP_ERR_INVALID_RESPONSE;
    }
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
}

size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_uid_t uid,
                                      uint16_t sub_device,
                                      rdm_response_t *response, char *param,
                                      size_t size) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(!RDM_UID_IS_BROADCAST(uid), 0, "uid cannot be broadcast");
  // TODO: more arg checks

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  // No parameter data to send
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_GET_COMMAND,
                         .pid = RDM_PID_SOFTWARE_VERSION_LABEL,
                         .pdl = 0};
  size_t written = rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  size_t num_params = 0;
  dmx_event_t event;
  const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
  if (event.err) {
    response->err = event.err;
    response->num_params = 0;
  } else if (read) {
    // Parse the response to ensure it is valid
    if (!rdm_decode_header(driver->data.buffer, &header)) {
      response->err = ESP_ERR_INVALID_RESPONSE;
    } else if (!header.checksum_is_valid) {
      response->err = ESP_ERR_INVALID_CRC;
    } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
      response->err = ESP_ERR_INVALID_ARG;
    } else {
      response->err = ESP_OK;
    }

    // Handle the parameter data
    response->type = header.response_type;
    if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
      // Decode the parameter data
      strncpy(param, &rdm->pd, header.pdl);
      num_params = header.pdl;
      response->num_params = header.pdl;
    } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
      // Get the estimated response time and convert it to FreeRTOS ticks
      uint32_t estimated_response_time;
      rdm_decode_16bit(&rdm->pd, &estimated_response_time, 1);
      response->timer = pdMS_TO_TICKS(estimated_response_time * 10);
    } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
      // Report the NACK reason
      rdm_decode_16bit(&rdm->pd, &response->nack_reason, 1);
    } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
      // This code should never run
      response->err = ESP_ERR_INVALID_RESPONSE;
    } else {
      // An unknown response type was received
      response->err = ESP_ERR_INVALID_RESPONSE;
    }
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
}

size_t rdm_get_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                               uint16_t sub_device, rdm_response_t *response,
                               bool *identify_state) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(!RDM_UID_IS_BROADCAST(uid), 0, "uid cannot be broadcast");
  // TODO: more arg checks

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  // No parameter data to send
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_GET_COMMAND,
                         .pid = RDM_PID_IDENTIFY_DEVICE,
                         .pdl = 0};
  size_t written = rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  size_t num_params = 0;
  dmx_event_t event;
  const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
  if (event.err) {
    response->err = event.err;
    response->num_params = 0;
  } else if (read) {
    // Parse the response to ensure it is valid
    if (!rdm_decode_header(driver->data.buffer, &header)) {
      response->err = ESP_ERR_INVALID_RESPONSE;
    } else if (!header.checksum_is_valid) {
      response->err = ESP_ERR_INVALID_CRC;
    } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
      response->err = ESP_ERR_INVALID_ARG;
    } else {
      response->err = ESP_OK;
    }

    // Handle the parameter data
    response->type = header.response_type;
    if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
      // Decode the parameter data
      num_params = rdm_decode_8bit(&rdm->pd, identify_state, 1);
      response->num_params = num_params;
    } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
      // Get the estimated response time and convert it to FreeRTOS ticks
      uint32_t estimated_response_time;
      rdm_decode_16bit(&rdm->pd, &estimated_response_time, 1);
      response->timer = pdMS_TO_TICKS(estimated_response_time * 10);
    } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
      // Report the NACK reason
      rdm_decode_16bit(&rdm->pd, &response->nack_reason, 1);
    } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
      // This code should never run
      response->err = ESP_ERR_INVALID_RESPONSE;
    } else {
      // An unknown response type was received
      response->err = ESP_ERR_INVALID_RESPONSE;
    }
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
  
  // TODO
  return 0;
}

// TODO: implement, docs
size_t rdm_set_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                               uint16_t sub_device, rdm_response_t *response,
                               const bool identify_state) {
  
  // TODO
  return 0;
}

// TODO: implement, docs
size_t rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                                 uint16_t sub_device, rdm_response_t *response,
                                 int *start_address) {
  // TODO
  return 0;
}

// TODO: implement, docs
size_t rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                                 uint16_t sub_device, rdm_response_t *response,
                                 const int start_address) {
  // TODO
  return 0;
}