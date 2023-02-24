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
#include "private/rdm_encode/functions.h"
#include "private/rdm_encode/types.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_mac.h"
#endif

// Used for argument checking at the beginning of each function.
#define RDM_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

static const char *TAG = "rdm";  // The log tagline for the file.

bool rdm_uid_array_is_broadcast(uint8_t *uid) {
  // bool is_broadcast = false;
  for (int i = 2; i < 6; ++i) {
    if (0xff != uid[i]) {
      return false;
    }
  }
  return true;
}

// UNDER CONSTRUCTION !!!!!
bool rdm_uid_array_is_addressed_to(uint8_t *uid, uint8_t *addressee) {
  // uid &= 0xffffffffffff;
  // addressee &= 0xffffffffffff;
  // bool temp = addressee == uid ||
  //        ((uid >> 32 == 0xffff || uid >> 32 == addressee >> 32) &&
  //         (uint32_t)uid == 0xffffffff);
  for (int i = 0; i < 6; ++i) {
    if ((addressee[i] != uid[i]) && 
        (0xff != uid[i])) {
      return false;
    }
  }
  return true;
}

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

rdm_uid_t rdm_get_uid_for_mfr(dmx_port_t dmx_num, uint16_t mfrID) {
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
    driver->rdm.uid |= (rdm_uid_t)mfrID << 32;
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

bool set_rdm_muted(dmx_port_t dmx_num, bool mute) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  bool was_muted;
  taskENTER_CRITICAL(spinlock);
  was_muted = driver->rdm.discovery_is_muted;
  driver->rdm.discovery_is_muted = mute;
  taskEXIT_CRITICAL(spinlock);

  return was_muted;
}

size_t rdm_send_response(dmx_port_t dmx_num, rdm_data_t *data, const void *payload, size_t pd_len,
                         rdm_response_type_t response, uint8_t *packet) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(pd_len <= 231, 0, "pd_len error");
  RDM_CHECK(response <= RDM_RESPONSE_TYPE_ACK_OVERFLOW, 0, "invalid response type error");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Fill in the fields of the response over those of the command
  data->sc = RDM_SC;
  data->sub_sc = RDM_SUB_SC;
  data->message_len = 24 + pd_len;
  data->cc++;  // turn cmd code into response? 1 higher 
  // data->pid = data->pid;   // Let it be
  data->pdl = pd_len;
  uint8_t *pPD = ((uint8_t*)&(data->pd));
  // Fill the payload Parameter Data, if any, into the packet
  if (NULL != payload) {
    for (int i = 0; i < pd_len; ++i) {
      pPD[i] = ((uint8_t *)payload)[i];
    }
  }

  uint8_t temp[6];
  memcpy(temp, data->destination_uid, 6);
  memcpy(data->destination_uid, data->source_uid, 6);
  memcpy(data->source_uid, temp, 6);
  
  data->response_type = response;  // was RDM_RESPONSE_TYPE_ACK;
  data->sub_device = RDM_ROOT_DEVICE;

  // Put the packet as it is so far into the driver's buffer for transmission
  memcpy(driver->data.buffer, data, 24 + pd_len);

  // Write and send the response
  size_t written =
      rdm_encode_response(driver->data.buffer, 24 + pd_len);
  // dmx_send(dmx_num, written);

  // For diagnostics (rcd)
  if (NULL != packet) {
    memcpy(packet, driver->data.buffer, written);
  }
  dmx_send(dmx_num, written);

  xSemaphoreGiveRecursive(driver->mux);
  return written;
}

size_t rdm_send_disc_response(dmx_port_t dmx_num, size_t preamble_len,
                              rdm_uid_t uid, uint8_t *packet) {
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

  // For diagnostics (rcd)
  if (NULL != packet) {
    memcpy(packet, driver->data.buffer, written);
  }

  xSemaphoreGiveRecursive(driver->mux);
  return written;
}

rdm_uid_t rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                      rdm_disc_unique_branch_t *params,
                                      rdm_response_t *response) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(params != NULL, 0, "params is null");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Prepare the RDM message
  rdm_data_t *const restrict rdm = (rdm_data_t *)driver->data.buffer;
  size_t written = rdm_encode_uids(&rdm->pd, (rdm_uid_t *)params, 2);
  const rdm_header_t header = {.destination_uid = RDM_BROADCAST_ALL_UID,
                               .source_uid = rdm_get_uid(dmx_num),
                               .tn = driver->rdm.tn,
                               .port_id = dmx_num + 1,
                               .message_count = 0,
                               .sub_device = RDM_ROOT_DEVICE,
                               .cc = RDM_CC_DISC_COMMAND,
                               .pid = RDM_PID_DISC_UNIQUE_BRANCH,
                               .pdl = written};
  written += rdm_encode_header(rdm, &header);
  dmx_send(dmx_num, written);

  // Wait for a response
  rdm_uid_t uid = 0;
  dmx_packet_t event;
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
    size_t num_params;
    if (!rdm_decode_disc_response((uint8_t *)rdm, &uid)) {
      err = ESP_ERR_INVALID_CRC;
      response_type = RDM_RESPONSE_TYPE_NONE;
      num_params = 0;
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
  return uid;
}

bool rdm_send_disc_mute(dmx_port_t dmx_num, rdm_uid_t uid, bool mute,
                        rdm_response_t *response, rdm_disc_mute_t *params) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Determine which PID to use (mute and un-mute are different PIDs)
  const rdm_pid_t pid = mute ? RDM_PID_DISC_MUTE : RDM_PID_DISC_UN_MUTE;

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Write and send the RDM message
  const uint8_t tn = driver->rdm.tn;
  rdm_data_t *const restrict rdm = (rdm_data_t *)driver->data.buffer;
  const rdm_header_t req_header = {.destination_uid = uid,
                                       .source_uid = rdm_get_uid(dmx_num),
                                       .tn = tn,
                                       .port_id = dmx_num + 1,
                                       .message_count = 0,
                                       .sub_device = RDM_ROOT_DEVICE,
                                       .cc = RDM_CC_DISC_COMMAND,
                                       .pid = pid,
                                       .pdl = 0};
  size_t written = rdm_encode_header(rdm, &req_header);
  dmx_send(dmx_num, written);

  // Determine if a response is expected
  size_t num_params = 0;
  if (!rdm_uid_is_broadcast(uid)) {
    // Receive the response
    dmx_packet_t event;
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
      rdm_header_t resp_header;
      if (!rdm_decode_header(rdm, &resp_header)) {
        err = ESP_ERR_INVALID_RESPONSE;
      } else if (!resp_header.checksum_is_valid) {
        err = ESP_ERR_INVALID_CRC;
      } else if (resp_header.cc != RDM_CC_DISC_COMMAND_RESPONSE ||
                 resp_header.pid != pid ||
                 resp_header.destination_uid != req_header.source_uid ||
                 resp_header.source_uid != req_header.destination_uid ||
                 resp_header.tn != tn ||
                 resp_header.sub_device != RDM_ROOT_DEVICE) {
        err = ESP_ERR_INVALID_RESPONSE;
      } else {
        err = ESP_OK;

        // Decode the response
        if (resp_header.response_type == RDM_RESPONSE_TYPE_ACK &&
            resp_header.pdl >= 2) {
          if (params != NULL) {
            rdm_decode_mute(&rdm->pd, params, 1, resp_header.pdl);
          }
          num_params = 1;
        } else {
          // Discovery commands do not accept any other response type
          err = ESP_ERR_INVALID_RESPONSE;
        }
      }

      // Report response back to user
      if (response != NULL) {
        response->err = err;
        response->type = resp_header.response_type;
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
  return num_params > 0;
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

  rdm_disc_mute_t mute;     // Mute parameters returned from devices.
  rdm_response_t response;  // Request response information.
  bool dev_muted;           // Is true if the responding device was muted.
  rdm_uid_t uid;            // The UID of the responding device.

  size_t num_found = 0;
  while (stack_size > 0) {
    rdm_disc_unique_branch_t *branch = &stack[--stack_size];
    size_t attempts = 0;

    if (branch->lower_bound == branch->upper_bound) {
      // Can't branch further so attempt to mute the device
      uid = branch->lower_bound;
      do {
        dev_muted = rdm_send_disc_mute(dmx_num, uid, true, &response, &mute);
      } while (!dev_muted && ++attempts < 3);

      // Attempt to fix possible error where responder is flipping its own UID
      if (!dev_muted) {
        uid = bswap64(uid) >> 16;  // Flip UID
        dev_muted = rdm_send_disc_mute(dmx_num, uid, true, NULL, &mute);
      }

      // Call the callback function and report a device has been found
      if (dev_muted && !response.err) {
        cb(dmx_num, uid, num_found, &mute, context);
        ++num_found;
      }
    } else {
      // Search the current branch in the RDM address space
      do {
        uid = rdm_send_disc_unique_branch(dmx_num, branch, &response);
      } while (uid == 0 && ++attempts < 3);
      if (uid != 0) {
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
            do {
              dev_muted = rdm_send_disc_mute(dmx_num, uid, true, NULL, &mute);
            } while (!dev_muted && ++attempts < 3);

            // Call the callback function and report a device has been found
            if (dev_muted) {
              cb(dmx_num, uid, num_found, &mute, context);
              ++num_found;
            }

            // Check if there are more devices in this branch
            attempts = 0;
            do {
              uid = rdm_send_disc_unique_branch(dmx_num, branch, &response);
            } while (uid == 0 && ++attempts < 3);
            if (uid != 0 && response.err) {
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

static size_t rdm_send_generic_request(
    dmx_port_t dmx_num, rdm_uid_t uid, rdm_sub_device_t sub_device,
    const rdm_cc_t cc, const rdm_pid_t pid,
    size_t (*encode)(void *, const void *, size_t), void *encode_params,
    size_t num_encode_params,
    size_t (*decode)(const void *, void *, size_t, size_t), void *decode_params,
    size_t num_decode_params, rdm_response_t *response) {
  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the initial RDM request
  const uint8_t tn = driver->rdm.tn;
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  size_t written;
  if (encode && encode_params && num_encode_params) {
    written = encode(&rdm->pd, encode_params, num_encode_params);
  } else {
    written = 0;
  }
  rdm_header_t req_header = {.destination_uid = uid,
                             .source_uid = rdm_get_uid(dmx_num),
                             .tn = tn,
                             .port_id = dmx_num + 1,
                             .message_count = 0,
                             .sub_device = sub_device,
                             .cc = cc,
                             .pid = pid,
                             .pdl = written};
  written += rdm_encode_header(rdm, &req_header);
  dmx_send(dmx_num, written);

  // Receive and decode the RDM response
  uint32_t return_val = 0;
  if (!rdm_uid_is_broadcast(uid)) {
    dmx_packet_t event;
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
      rdm_header_t resp_header;
      if (!rdm_decode_header(driver->data.buffer, &resp_header)) {
        err = ESP_ERR_INVALID_RESPONSE;
      } else if (!resp_header.checksum_is_valid) {
        err = ESP_ERR_INVALID_CRC;
      } else if (resp_header.cc != req_header.cc + 1 ||
                 resp_header.pid != req_header.pid ||
                 resp_header.destination_uid != req_header.source_uid ||
                 resp_header.source_uid != req_header.destination_uid ||
                 resp_header.sub_device != req_header.sub_device ||
                 resp_header.tn != req_header.tn) {
        err = ESP_ERR_INVALID_RESPONSE;
      } else {
        err = ESP_OK;

        // Handle the parameter data
        uint32_t response_val;
        if (resp_header.response_type == RDM_RESPONSE_TYPE_ACK) {
          // Decode the parameter data
          if (decode) {
            // Return the number of params available when response is received
            return_val = decode(&rdm->pd, decode_params, num_decode_params,
                                resp_header.pdl);
            response_val = return_val;
          } else {
            // Return true when no response parameters are expected
            return_val = true;
            response_val = 0;
          }
        } else if (resp_header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
          // Get the estimated response time and convert it to FreeRTOS ticks
          rdm_decode_16bit(&rdm->pd, &response_val, 1, resp_header.pdl);
          response_val = pdMS_TO_TICKS(response_val * 10);
        } else if (resp_header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
          // Report the NACK reason
          rdm_decode_16bit(&rdm->pd, &response_val, 1, resp_header.pdl);
        } else if (resp_header.response_type ==
                   RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
          // TODO: implement overflow support
          err = ESP_ERR_NOT_SUPPORTED;
          response_val = 0;
        } else {
          // An unknown response type was received
          err = ESP_ERR_INVALID_RESPONSE;
          response_val = 0;
        }

        // Report response back to user
        if (response != NULL) {
          response->err = err;
          response->type = resp_header.response_type;
          response->num_params = response_val;
        }
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
  return return_val;
}

size_t rdm_get_supported_parameters(dmx_port_t dmx_num, rdm_uid_t uid,
                                    rdm_sub_device_t sub_device,
                                    rdm_response_t *response, rdm_pid_t *pids,
                                    size_t size) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(uid <= RDM_MAX_UID, 0, "uid error");
  RDM_CHECK(sub_device != RDM_ALL_SUB_DEVICES, 0, "sub_device error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_send_generic_request(dmx_num, uid, sub_device, RDM_CC_GET_COMMAND,
                                  RDM_PID_SUPPORTED_PARAMETERS, NULL, NULL, 0,
                                  rdm_decode_16bit, pids, size, response);
}

size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_uid_t uid,
                           rdm_sub_device_t sub_device,
                           rdm_response_t *response,
                           rdm_device_info_t *device_info) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(uid <= RDM_MAX_UID, 0, "uid error");
  RDM_CHECK(sub_device != RDM_ALL_SUB_DEVICES, 0, "sub_device error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_send_generic_request(
      dmx_num, uid, sub_device, RDM_CC_GET_COMMAND, RDM_PID_DEVICE_INFO, NULL,
      NULL, 0, rdm_decode_device_info, device_info, 1, response);
}

size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_uid_t uid,
                                      rdm_sub_device_t sub_device,
                                      rdm_response_t *response, char *label,
                                      size_t size) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(uid <= RDM_MAX_UID, 0, "uid error");
  RDM_CHECK(sub_device != RDM_ALL_SUB_DEVICES, 0, "sub_device error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_send_generic_request(dmx_num, uid, sub_device, RDM_CC_GET_COMMAND,
                                  RDM_PID_SOFTWARE_VERSION_LABEL, NULL, NULL, 0,
                                  rdm_decode_string, label, size, response);
}

size_t rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                                 rdm_sub_device_t sub_device,
                                 rdm_response_t *response, int *start_address) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(uid <= RDM_MAX_UID, 0, "uid error");
  RDM_CHECK(sub_device != RDM_ALL_SUB_DEVICES, 0, "sub_device error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_send_generic_request(dmx_num, uid, sub_device, RDM_CC_GET_COMMAND,
                                  RDM_PID_DMX_START_ADDRESS, NULL, NULL, 0,
                                  rdm_decode_16bit, start_address, 1, response);
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_uid_t uid,
                               rdm_sub_device_t sub_device,
                               rdm_response_t *response, int start_address) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(uid <= RDM_MAX_UID || uid == RDM_BROADCAST_ALL_UID, 0, "uid error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(start_address > 0 && start_address < DMX_MAX_PACKET_SIZE, 0,
            "start_address must be >0 and <513");

  return rdm_send_generic_request(dmx_num, uid, sub_device, RDM_CC_SET_COMMAND,
                                  RDM_PID_DMX_START_ADDRESS, rdm_encode_16bit,
                                  &start_address, 1, NULL, 0, 0, response);
}

size_t rdm_get_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                               rdm_sub_device_t sub_device,
                               rdm_response_t *response, bool *identify) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(uid <= RDM_MAX_UID, 0, "uid error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  RDM_CHECK(!rdm_uid_is_broadcast(uid), 0, "uid cannot be broadcast");
  RDM_CHECK(sub_device != RDM_ALL_SUB_DEVICES, 0,
            "cannot send to all sub-devices");

  return rdm_send_generic_request(dmx_num, uid, sub_device, RDM_CC_GET_COMMAND,
                                  RDM_PID_IDENTIFY_DEVICE, NULL, NULL, 0,
                                  rdm_decode_8bit, identify, 1, response);
}

bool rdm_set_identify_device(dmx_port_t dmx_num, rdm_uid_t uid,
                             rdm_sub_device_t sub_device,
                             rdm_response_t *response, bool identify) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(uid <= RDM_MAX_UID || uid == RDM_BROADCAST_ALL_UID, 0, "uid error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_send_generic_request(dmx_num, uid, sub_device, RDM_CC_SET_COMMAND,
                                  RDM_PID_IDENTIFY_DEVICE, rdm_encode_8bit,
                                  &identify, 1, NULL, NULL, 0, response);
}
