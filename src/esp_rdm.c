#include "esp_rdm.h"

#include <stdint.h>
#include <string.h>

#include "dmx_types.h"
#include "endian.h"
#include "esp_check.h"
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "private/driver.h"
#include "private/rdm_encode/types.h"
#include "private/rdm_encode/functions.h"


// Used for argument checking at the beginning of each function.
#define RDM_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

static const char *TAG = "rdm";  // The log tagline for the file.

rdm_uid_t rdm_get_uid(dmx_port_t dmx_num) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Initialize the RDM UID 
  taskENTER_CRITICAL(spinlock);
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
  taskEXIT_CRITICAL(spinlock);

  return uid;
}

void rdm_set_uid(dmx_port_t dmx_num, rdm_uid_t uid) { 
  RDM_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  RDM_CHECK(uid <= RDM_MAX_UID, , "uid error");
  
  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);
  driver->rdm.uid = uid;  
  taskEXIT_CRITICAL(spinlock);
}

bool rdm_is_muted(dmx_port_t dmx_num) { 
  RDM_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  
  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  bool is_muted;
  taskENTER_CRITICAL(spinlock);
  is_muted = driver->rdm.discovery_is_muted;
  taskEXIT_CRITICAL(spinlock);

  return is_muted;
}

size_t rdm_send_disc_response(dmx_port_t dmx_num, size_t preamble_len,
                              rdm_uid_t uid) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(preamble_len <= 7, 0, "preamble_len error");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Write and send the response
  size_t written =
      rdm_encode_disc_response(driver->data.buffer, preamble_len, uid);
  dmx_send(dmx_num, written);

  xSemaphoreGiveRecursive(driver->mux);
  return written;
}

size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                   rdm_disc_unique_branch_t *params,
                                   rdm_response_t *response, rdm_uid_t *uid) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(params != NULL, 0, "params is null");
  RDM_CHECK(uid != NULL, 0, "uid is null");

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

  // Wait for a response
  size_t num_params = 0;
  dmx_event_t event;
  const size_t read = dmx_receive(dmx_num, &event, DMX_TIMEOUT_TICK);
  if (!read) {
    if (response != NULL) {
      response->err = event.err;
      response->type = RDM_RESPONSE_TYPE_NONE;
      response->num_params = 0;
    }
  } else {
    // Check the packet for errors
    esp_err_t err;
    rdm_response_type_t response_type;
    if (!rdm_decode_disc_response(driver->data.buffer, uid)) {
      err = ESP_ERR_INVALID_CRC;
      response_type = RDM_RESPONSE_TYPE_NONE;
      *uid = 0;
    } else {
      err = ESP_OK;
      response_type = RDM_RESPONSE_TYPE_ACK;
      num_params = 1;
    }

    // Report response back to user
    if (response != NULL) {
      response->err = err; 
      response->type = response_type;
      response->num_params = num_params;
    }
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

  // Determine if a response is expected
  esp_err_t err = ESP_OK;
  rdm_response_type_t response_type = RDM_RESPONSE_TYPE_NONE;
  size_t num_params = 0;
  if (!RDM_UID_IS_BROADCAST(uid)) {
    // Receive the response
    dmx_event_t event;
    const size_t read = dmx_receive(dmx_num, &event, DMX_TIMEOUT_TICK);
    if (!read) {
      if (response != NULL) {
        response->err = event.err;
        response->type = RDM_RESPONSE_TYPE_NONE;
        response->num_params = 0;
      }
    } else {
      // Check the packet for errors
      esp_err_t err;
      if (!rdm_decode_header(driver->data.buffer, &header)) {
        err = ESP_ERR_INVALID_RESPONSE;
      } else if (!header.checksum_is_valid) {
        err = ESP_ERR_INVALID_CRC;
      } else {
        err = ESP_OK;
      }
      // TODO: error checking of packet -- check pid, cc
      
      // Decode the response
      if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
        if (params != NULL) {
          rdm_decode_mute(&rdm->pd, params, header.pdl);
        }
        num_params = 1;
      } else {
        // Discovery commands do not accept any other response type
        err = ESP_ERR_INVALID_RESPONSE;
      }
      
      // Report response back to user
      if (response != NULL) {
        response->err = err;
        response->type = header.response_type;
        response->num_params = num_params;
      }
    }
  } else {
    if (response != NULL) {
      response->err = ESP_OK;
      response->type = RDM_RESPONSE_TYPE_NONE;
      response->num_params = 0;
    }
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

      // Call the callback function and report a device has been found
      if (response.num_params > 0 && !response.err) {
        cb(dmx_num, uid, num_found, &mute, context);
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

            // Call the callback function and report a device has been found
            if (response.num_params > 0) {
              cb(dmx_num, uid, num_found, &mute, context);
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
                        rdm_disc_mute_t *mute, void *context) {
  struct rdm_disc_default_ctx *c = (struct rdm_disc_default_ctx *)context;
  if (num_found < c->size && c->uids != NULL) {
    c->uids[num_found] = uid;
  }
}

size_t rdm_discover_devices_simple(dmx_port_t dmx_num, rdm_uid_t *uids,
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
  RDM_CHECK(param != NULL, 0, "param is null");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  const rdm_pid_t pid = RDM_PID_DEVICE_INFO;
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  // No parameter data to send
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_GET_COMMAND,
                         .pid = pid,
                         .pdl = 0};
  size_t written = rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  uint32_t num_params = 0;
  dmx_event_t event;
  const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
  if (!read) {
    if (response != NULL) {
      response->err = event.err;
      response->type = RDM_RESPONSE_TYPE_NONE;
      response->num_params = 0;
    }
  } else {
    // Parse the response to ensure it is valid
    esp_err_t err;
    if (!rdm_decode_header(driver->data.buffer, &header)) {
      err = ESP_ERR_INVALID_RESPONSE;
    } else if (!header.checksum_is_valid) {
      err = ESP_ERR_INVALID_CRC;
    } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
      err = ESP_ERR_INVALID_ARG;
    } else {
      err = ESP_OK;
    }

    // Handle the parameter data
    uint32_t response_val;
    if (header.cc == RDM_CC_GET_COMMAND_RESPONSE && header.pid == pid) {
      if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
        // Decode the parameter data
        num_params = rdm_decode_device_info(&rdm->pd, param);
        response_val = num_params;
      } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
        // Get the estimated response time and convert it to FreeRTOS ticks
        rdm_decode_16bit(&rdm->pd, &response_val, 1);
        response_val = pdMS_TO_TICKS(response_val * 10);
      } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
        // Report the NACK reason
        rdm_decode_16bit(&rdm->pd, &response_val, 1);
      } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
        // This code should never run
        err = ESP_ERR_INVALID_RESPONSE;
      } else {
        // An unknown response type was received
        err = ESP_ERR_INVALID_RESPONSE;
      }
    } else {
      // Received data is not for this request
      err = ESP_ERR_INVALID_RESPONSE;
    }

    // Report response back to user
    if (response != NULL) {
      response->err = err;
      response->type = header.response_type;
      response->num_params = response_val;
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
  RDM_CHECK(param != NULL, 0, "param is null");
  RDM_CHECK(size > 0, 0, "size is 0");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  const rdm_pid_t pid = RDM_PID_SOFTWARE_VERSION_LABEL;
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  // No parameter data to send
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_GET_COMMAND,
                         .pid = pid,
                         .pdl = 0};
  size_t written = rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  size_t num_params = 0;
  dmx_event_t event;
  const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
  if (!read) {
    if (response != NULL) {
      response->err = event.err;
      response->type = RDM_RESPONSE_TYPE_NONE;
      response->num_params = 0;
    }
  } else {
    // Parse the response to ensure it is valid
    esp_err_t err;
    if (!rdm_decode_header(driver->data.buffer, &header)) {
      err = ESP_ERR_INVALID_RESPONSE;
    } else if (!header.checksum_is_valid) {
      err = ESP_ERR_INVALID_CRC;
    } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
      err = ESP_ERR_INVALID_ARG;
    } else {
      err = ESP_OK;
    }

    // Handle the parameter data
    uint32_t response_val;
    if (header.cc == RDM_CC_GET_COMMAND_RESPONSE && header.pid == pid) {
      if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
        // Decode the parameter data
        size = size > header.pdl ? header.pdl : size;
        strncpy(param, (void *)&rdm->pd, size);
        param[size] = '\0';  // Null terminate string
        response_val = header.pdl;
        num_params = size;
      } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
        // Get the estimated response time and convert it to FreeRTOS ticks
        rdm_decode_16bit(&rdm->pd, &response_val, 1);
        response_val = pdMS_TO_TICKS(response_val * 10);
      } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
        // Report the NACK reason
        rdm_decode_16bit(&rdm->pd, &response_val, 1);
      } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
        // This code should never run
        err = ESP_ERR_INVALID_RESPONSE;
      } else {
        // An unknown response type was received
        err = ESP_ERR_INVALID_RESPONSE;
      }
    } else {
      err = ESP_ERR_INVALID_RESPONSE;
    }

    // Report response back to user
    if (response != NULL) {
      response->err = err;
      response->type = header.response_type;
      response->num_params = response_val;
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
  RDM_CHECK(identify_state != NULL, 0, "identify_state is null");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  // No parameter data to send
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_GET_COMMAND,
                         .pid = pid,
                         .pdl = 0};
  size_t written = rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  size_t num_params = 0;
  dmx_event_t event;
  const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
  if (!read) {
    if (response != NULL) {
      response->err = event.err;
      response->type = RDM_RESPONSE_TYPE_NONE;
      response->num_params = 0;
    }
  } else  {
    // Parse the response to ensure it is valid
    esp_err_t err;
    if (!rdm_decode_header(rdm, &header)) {
      err = ESP_ERR_INVALID_RESPONSE;
    } else if (!header.checksum_is_valid) {
      err = ESP_ERR_INVALID_CRC;
    } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
      err = ESP_ERR_INVALID_ARG;
    } else {
      err = ESP_OK;
    }

    // Handle the parameter data
    uint32_t response_val;
    if (header.cc == RDM_CC_GET_COMMAND_RESPONSE && header.pid == pid) {
      if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
        // Decode the parameter data
        num_params = rdm_decode_8bit(&rdm->pd, (uint32_t *)identify_state, 1);
        response_val = num_params;
      } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
        // Get the estimated response time and convert it to FreeRTOS ticks
        rdm_decode_16bit(&rdm->pd, &response_val, 1);
        response_val = pdMS_TO_TICKS(response_val * 10);
      } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
        // Report the NACK reason
        rdm_decode_16bit(&rdm->pd, &response_val, 1);
      } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
        // This code should never run
        err = ESP_ERR_INVALID_RESPONSE;
      } else {
        // An unknown response type was received
        err = ESP_ERR_INVALID_RESPONSE;
      }
    } else {
      err = ESP_ERR_INVALID_RESPONSE;
    }

    // Report response back to user
    if (response != NULL) {
      response->err = err;
      response->type = header.response_type;
      response->num_params = response_val;
    }
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
}

// FIXME: make sub_device type
size_t rdm_set_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                               uint16_t sub_device, rdm_response_t *response,
                               const bool identify_state) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  size_t written = rdm_encode_8bit(&rdm->pd, (uint32_t *)&identify_state, 1);
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_SET_COMMAND,
                         .pid = pid,
                         .pdl = written};
  written += rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  size_t num_params = 0;
  if (!RDM_UID_IS_BROADCAST(uid)) {
    dmx_event_t event;
    const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
    if (!read) {
      if (response != NULL) {
        response->err = event.err;
        response->type = RDM_RESPONSE_TYPE_NONE;
        response->num_params = 0;
      }
    } else {
      // Parse the response to ensure it is valid
      esp_err_t err;
      if (!rdm_decode_header(driver->data.buffer, &header)) {
        err = ESP_ERR_INVALID_RESPONSE;
      } else if (!header.checksum_is_valid) {
        err = ESP_ERR_INVALID_CRC;
      } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
        err = ESP_ERR_INVALID_ARG;
      } else {
        err = ESP_OK;
      }

      // Handle the parameter data
      uint32_t response_val;
      if (header.cc == RDM_CC_SET_COMMAND_RESPONSE && header.pid == pid) {
        if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
          // Decode the parameter data
          // No params to decode
          response_val = 0;
        } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
          // Get the estimated response time and convert it to FreeRTOS ticks
          rdm_decode_16bit(&rdm->pd, &response_val, 1);
          response_val = pdMS_TO_TICKS(response_val * 10);
        } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
          // Report the NACK reason
          rdm_decode_16bit(&rdm->pd, &response_val, 1);
        } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
          // This code should never run
          err = ESP_ERR_INVALID_RESPONSE;
        } else {
          // An unknown response type was received
          err = ESP_ERR_INVALID_RESPONSE;
        }
      } else {
        err = ESP_ERR_INVALID_RESPONSE;
      }

      // Report response back to user
      if (response != NULL) {
        response->err = err;
        response->type = header.response_type;
        response->num_params = response_val;
      }
    }
  } else {
    if (response != NULL) {
      response->err = ESP_OK;
      response->type = RDM_RESPONSE_TYPE_NONE;
      response->num_params = 0;
    }
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(20));
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
}

size_t rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                                 uint16_t sub_device, rdm_response_t *response,
                                 int *start_address) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(!RDM_UID_IS_BROADCAST(uid), 0, "uid cannot be broadcast");
  RDM_CHECK(start_address != NULL, 0, "start_address is null");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  const rdm_pid_t pid = RDM_PID_DMX_START_ADDRESS;
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  // No parameter data to send
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_GET_COMMAND,
                         .pid = pid,
                         .pdl = 0};
  size_t written = rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  size_t num_params = 0;
  dmx_event_t event;
  const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
  if (!read) {
    if (response != NULL) {
      response->err = event.err;
      response->type = RDM_RESPONSE_TYPE_NONE;
      response->num_params = 0;
    }
  } else {
    // Parse the response to ensure it is valid
    esp_err_t err;
    if (!rdm_decode_header(rdm, &header)) {
      err = ESP_ERR_INVALID_RESPONSE;
    } else if (!header.checksum_is_valid) {
      err = ESP_ERR_INVALID_CRC;
    } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
      err = ESP_ERR_INVALID_ARG;
    } else {
      err = ESP_OK;
    }

    // Handle the parameter data
    uint32_t response_val;
    if (header.cc == RDM_CC_GET_COMMAND_RESPONSE && header.pid == pid) {
      if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
        // Decode the parameter data
        num_params = rdm_decode_16bit(&rdm->pd, (uint32_t *)&start_address, 1);
        response_val = num_params;
      } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
        // Get the estimated response time and convert it to FreeRTOS ticks
        rdm_decode_16bit(&rdm->pd, &response_val, 1);
        response_val = pdMS_TO_TICKS(response_val * 10);
      } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
        // Report the NACK reason
        rdm_decode_16bit(&rdm->pd, &response_val, 1);
      } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
        // This code should never run
        err = ESP_ERR_INVALID_RESPONSE;
      } else {
        // An unknown response type was received
        err = ESP_ERR_INVALID_RESPONSE;
      }
    } else {
      err = ESP_ERR_INVALID_RESPONSE;
    }

    // Report response back to user
    if (response != NULL) {
      response->err = err;
      response->type = header.response_type;
      response->num_params = response_val;
    }
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
}

size_t rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                                 uint16_t sub_device, rdm_response_t *response,
                                 const int start_address) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  const rdm_pid_t pid = RDM_PID_DMX_START_ADDRESS;
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  size_t written = rdm_encode_16bit(&rdm->pd, (uint32_t *)&start_address, 1);
  rdm_header_t header = {.destination_uid = uid,
                         .source_uid = rdm_get_uid(dmx_num),
                         .tn = 0,  // TODO: get up-to-date TN
                         .port_id = dmx_num + 1,
                         .message_count = 0,
                         .sub_device = sub_device,
                         .cc = RDM_CC_SET_COMMAND,
                         .pid = pid,
                         .pdl = written};
  written += rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  size_t num_params = 0;
  if (!RDM_UID_IS_BROADCAST(uid)) {
    dmx_event_t event;
    const size_t read = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(20));
    if (!read) {
      if (response != NULL) {
        response->err = event.err;
        response->type = RDM_RESPONSE_TYPE_NONE;
        response->num_params = 0;
      }
    } else {
      // Parse the response to ensure it is valid
      esp_err_t err;
      if (!rdm_decode_header(driver->data.buffer, &header)) {
        err = ESP_ERR_INVALID_RESPONSE;
      } else if (!header.checksum_is_valid) {
        err = ESP_ERR_INVALID_CRC;
      } else if (header.destination_uid != rdm_get_uid(dmx_num)) {
        err = ESP_ERR_INVALID_ARG;
      } else {
        err = ESP_OK;
      }

      // Handle the parameter data
      uint32_t response_val;
      if (header.cc == RDM_CC_SET_COMMAND_RESPONSE && header.pid == pid) {
        if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
          // Decode the parameter data
          // No params to decode
          response_val = 0;
        } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
          // Get the estimated response time and convert it to FreeRTOS ticks
          rdm_decode_16bit(&rdm->pd, &response_val, 1);
          response_val = pdMS_TO_TICKS(response_val * 10);
        } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
          // Report the NACK reason
          rdm_decode_16bit(&rdm->pd, &response_val, 1);
        } else if (header.response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
          // This code should never run
          err = ESP_ERR_INVALID_RESPONSE;
        } else {
          // An unknown response type was received
          err = ESP_ERR_INVALID_RESPONSE;
        }
      } else {
        err = ESP_ERR_INVALID_RESPONSE;
      }

      // Report response back to user
      if (response != NULL) {
        response->err = err;
        response->type = header.response_type;
        response->num_params = response_val;
      }
    }
  } else {
    if (response != NULL) {
      response->err = ESP_OK;
      response->type = RDM_RESPONSE_TYPE_NONE;
      response->num_params = 0;
    }
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(20));
  }

  xSemaphoreGiveRecursive(driver->mux);
  return num_params;
}
