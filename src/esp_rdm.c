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
#include "impl/driver.h"
#include "impl/rdm_encode_types.h"
#include "impl/rdm_encode_funcs.h"
#include "rdm_constants.h"

// Used for argument checking at the beginning of each function.
#define RDM_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

static const char *TAG = "rdm";  // The log tagline for the file.

static rdm_uid_t rdm_uid = 0;  // The 48-bit unique ID of this device.
static bool rdm_disc_is_muted = false;  // True if RDM discovery is muted.

rdm_uid_t rdm_get_uid() {
  // Initialize the RDM UID
  if (rdm_uid == 0) {
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    rdm_uid = (rdm_uid_t)RDM_DEFAULT_MANUFACTURER_ID << 32;
    rdm_uid |= bswap32(*(uint32_t *)(mac + 2));
  }

  return rdm_uid;
}

void rdm_set_uid(rdm_uid_t uid) { rdm_uid = uid; }

bool rdm_is_muted() { return rdm_disc_is_muted; }

size_t rdm_encode(void *destination, size_t size, const rdm_header_t *header,
                  const void *params, size_t num_params, size_t message_num) {
  RDM_CHECK(destination != NULL, 0, "destination is null");
  RDM_CHECK(size > 0, 0, "size is 0");
  RDM_CHECK(header != NULL, 0, "header is null");

  size_t bytes_encoded = 0;
  const rdm_cc_t cc = header->cc;
  const rdm_pid_t pid = header->pid;

  if (cc == RDM_CC_DISC_COMMAND_RESPONSE && pid == RDM_PID_DISC_UNIQUE_BRANCH &&
      size >= 17) {
    // Encode DISC_UNIQUE_BRANCH response
    
    uint8_t *data = destination;
    const rdm_uid_t uid = header->source_uid;

    // Encode the RDM preamble and delimiter
    const size_t preamble_len = size - 17 <= 7 ? size - 17 : 7;
    for (int i = 0; i < preamble_len; ++i) {
      data[i] = RDM_PREAMBLE;
    }
    data[7] = RDM_DELIMITER;

    // Encode the UID and calculate the checksum
    uint16_t checksum = 0;
    for (int i = 8, j = 5; i < 20; i += 2, --j) {
      data[i] = ((uint8_t *)&uid)[j] | 0xaa;
      data[i + 1] = ((uint8_t *)&uid)[j] | 0x55;
      checksum += ((uint8_t *)&uid)[j] + (0xaa + 0x55);
    }

    // Encode the checksum
    data[20] = (checksum >> 8) | 0xaa;
    data[21] = (checksum >> 8) | 0x55;
    data[22] = checksum | 0xaa;
    data[23] = checksum | 0x55;

    bytes_encoded = preamble_len + 17;
  } else if (size >= RDM_BASE_PACKET_SIZE) {
    // Encode standard RDM message

    // Encode most of the RDM message header
    rdm_data_t *buf = destination;
    buf->sc = RDM_SC;
    buf->sub_sc = RDM_SUB_SC;
    buf->message_len = RDM_BASE_PACKET_SIZE - 2;  // Exclude checksum
    uid_to_buf(buf->destination_uid, header->destination_uid);
    uid_to_buf(buf->source_uid, header->source_uid);
    buf->tn = header->tn;
    buf->port_id = header->port_id;
    buf->message_count = header->message_count;
    buf->sub_device = bswap16(header->sub_device);
    buf->cc = header->cc;
    buf->pid = bswap16(header->pid);

    // Encode PDL and Parameter Data
    size_t pdl = 0;
    if (pid >= 0x0000 && pid < 0x0100) {
      if (pid == RDM_PID_DISC_MUTE || pid == RDM_PID_DISC_UN_MUTE) {
        if (cc == RDM_CC_DISC_COMMAND_RESPONSE) {
          pdl = rdm_encode_disc_mute(buf, size, params);
        }
      } else if (pid == RDM_PID_SUPPORTED_PARAMETERS) {
        // TODO
      } else if (pid == RDM_PID_PARAMETER_DESCRIPTION) {
        // TODO
      } else if (pid == RDM_PID_DEVICE_INFO) {
        if (cc == RDM_CC_GET_COMMAND_RESPONSE) {
          pdl = rdm_encode_device_info(buf, size, params);
        }
      } else if (pid == RDM_PID_SOFTWARE_VERSION_LABEL) {
        // TODO
      } else if (pid == RDM_PID_DMX_START_ADDRESS) {
        // TODO
      }
    } else if (pid >= 0x1000 && pid < 0x1100) {
      if (pid == RDM_PID_IDENTIFY_DEVICE) {
        // TODO
      }
    }

    // Update PDL and message length
    buf->message_len += pdl;
    buf->pdl = pdl;

    // Calculate checksum
    uint16_t checksum = 0;
    for (int i = 0; i < buf->message_len; ++i) {
      checksum += *(uint8_t *)(destination + i);
    }
    *(uint16_t *)(destination + buf->message_len) = bswap16(checksum);

    bytes_encoded = buf->message_len + 2;
  }

  return bytes_encoded;
}

size_t rdm_send_disc_response(dmx_port_t dmx_num, rdm_uid_t uid) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Prepare and encode the response
  uint8_t data[24];
  const rdm_header_t header = {
    .cc = RDM_CC_DISC_COMMAND_RESPONSE,
    .pid = RDM_PID_DISC_UNIQUE_BRANCH,
    .source_uid = uid
  };
  const size_t written = rdm_encode(data, sizeof(data), &header, NULL, 0, 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Write and send the response
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, data, written);
  const size_t sent = dmx_send(dmx_num, 0);

  xSemaphoreGiveRecursive(driver->mux);
  return sent;
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

  // Initialize the response to the default values
  if (response != NULL) {
    response->err = ESP_OK;
    response->type = RDM_RESPONSE_TYPE_NONE;
    response->num_params = 0;
  }

  // Prepare the RDM message
  const rdm_header_t header = {
    .destination_uid = RDM_BROADCAST_UID,
    .source_uid = rdm_get_uid(),
    .tn = 0,  // TODO: get up-to-date transaction number
    .port_id = dmx_num + 1,
    .message_count = 0,
    .sub_device = 0,
    .cc = RDM_CC_DISC_COMMAND, 
    .pid = RDM_PID_DISC_UNIQUE_BRANCH, 
  };
  const size_t written = rdm_encode(driver->data.buffer, DMX_MAX_PACKET_SIZE,
                                    &header, params, 1, 0);
  dmx_send(dmx_num, written);

  // Wait for a response
  size_t num_params = 0;
  dmx_event_t packet;
  const size_t read = dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK);
  if (packet.err) {
    response->err = packet.err;
  } else if (read) {
    // Check the packet for errors
    rdm_header_t header;
    if (!rdm_decode_header(driver->data.buffer, read, &header)) {
      response->err = ESP_ERR_INVALID_RESPONSE;
    } else if (!header.checksum_is_valid) {
      response->err = ESP_ERR_INVALID_CRC;
    } // TODO: more error checking?

    // This is a special case in which no params are decoded
    *uid = header.source_uid;
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
  const rdm_header_t header = {
    .destination_uid = uid,
    .source_uid = rdm_get_uid(),
    .tn = 0, // TODO: get up-to-date transaction number
    .port_id = dmx_num + 1,
    .message_count = 0,
    .sub_device = 0,
    .cc = RDM_CC_DISC_COMMAND, 
    .pid = pid, 
  };
  const size_t bytes_encoded =
      rdm_encode(driver->data.buffer, DMX_MAX_PACKET_SIZE, &header, NULL, 0, 0);
  dmx_send(dmx_num, bytes_encoded);

  // Initialize the response to the default values
  if (response != NULL) {
    response->err = ESP_OK;
    response->type = RDM_RESPONSE_TYPE_NONE;
    response->num_params = 0;
  }

  // Determine if a response is expected
  size_t num_params = 0;
  if (uid != RDM_BROADCAST_UID) {
    // Receive the response
    dmx_event_t packet;
    const size_t read = dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK);
    if (packet.err) {
      response->err = packet.err;
    } else if (read) {
      // Check the packet for errors
      rdm_header_t header;
      if (!rdm_decode_header(driver->data.buffer, read, &header)) {
        response->err = ESP_ERR_INVALID_RESPONSE;
      } else if (!header.checksum_is_valid) {
        response->err = ESP_ERR_INVALID_CRC;
      }
      // TODO: error checking of packet -- check pid, cc, and ACK?
      
      // Decode the response
      if (header.response_type == RDM_RESPONSE_TYPE_ACK) {
        num_params = rdm_decode_disc_mute((rdm_data_t *)driver->data.buffer,
                                          read, params);
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

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Un-mute all devices
  rdm_send_disc_mute(dmx_num, RDM_BROADCAST_UID, false, NULL, NULL);

  // Initialize the stack with the initial branch instruction
  // The max depth of the binary tree is 49 nodes
  size_t stack_size = 1;
#ifndef CONFIG_RDM_STATIC_DEVICE_DISCOVERY
  rdm_disc_unique_branch_t *stack;
  stack = malloc(sizeof(rdm_disc_unique_branch_t) * 49);
#else
  rdm_disc_unique_branch_t stack[49];  // 784B - use with caution!
#endif
  stack[0].lower_bound = 0;
  stack[0].upper_bound = RDM_MAX_UID;

  size_t num_found = 0;
  while (stack_size > 0) {
    rdm_disc_unique_branch_t *params = &stack[--stack_size];
    size_t attempts = 0;
    rdm_response_t response;
    rdm_uid_t uid;

    if (params->lower_bound == params->upper_bound) {
      // Can't branch further so attempt to mute the device
      uid = params->lower_bound;
      rdm_disc_mute_t mute_params;
      do {
        rdm_send_disc_mute(dmx_num, uid, true, &response, &mute_params);
      } while (response.num_params == 0 && ++attempts < 3);

      // Attempt to fix possible error where responder is flipping its own UID
      if (response.num_params == 0) {
        uid = bswap64(uid) >> 16;  // Flip UID
        rdm_send_disc_mute(dmx_num, uid, true, &response, &mute_params);
      }

      // Add the UID to the list
      if (response.num_params > 0 && !response.err) {
        if (mute_params.binding_uid) {
          uid = mute_params.binding_uid;
        }
        cb(dmx_num, uid, num_found, context);
        ++num_found;
      }
    } else {
      // Search the current branch in the RDM address space
      do {
        rdm_send_disc_unique_branch(dmx_num, params, &response, &uid);
      } while (response.num_params == 0 && ++attempts < 3);
      if (response.num_params > 0) {
        bool devices_remaining = true;

#ifndef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
        /*
        Stop the RDM controller from branching all the way down to the
        individual address if it is not necessary. When debugging, this function
        should not be used as it can hide bugs in the discovery algorithm. Users
        can use the sdkconfig to enable or disable discovery debugging.
        */
        if (!response.err) {
          for (int quick_finds = 0; quick_finds < 3; ++quick_finds) {
            // Attempt to mute the device
            attempts = 0;
            rdm_disc_mute_t mute_params;
            do {
              rdm_send_disc_mute(dmx_num, uid, true, &response, &mute_params);
            } while (response.num_params == 0 && ++attempts < 3);

            // Add the UID to the list
            if (response.num_params > 0) {
              if (mute_params.binding_uid) {
                uid = mute_params.binding_uid;
              }
              cb(dmx_num, uid, num_found, context);
              ++num_found;
            }

            // Check if there are more devices in this branch
            attempts = 0;
            do {
              rdm_send_disc_unique_branch(dmx_num, params, &response, &uid);
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
          const rdm_uid_t lower_bound = params->lower_bound;
          const rdm_uid_t mid = (lower_bound + params->upper_bound) / 2;

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

size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_uid_t uid,
                           uint16_t sub_device, rdm_response_t *response,
                           rdm_device_info_t *device_info) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Encode and send the RDM message
  const rdm_header_t header = {
      .destination_uid = uid,
      .source_uid = rdm_get_uid(),
      .tn = 0,  // TODO: get up-to-date transaction number
      .port_id = dmx_num + 1,
      .message_count = 0,
      .sub_device = 0,
      .cc = RDM_CC_DISC_COMMAND,
      .pid = RDM_PID_DEVICE_INFO,
  };
  const size_t written = rdm_encode(driver->data.buffer, DMX_MAX_PACKET_SIZE,
                                    &header, NULL, 0, 0);
  dmx_send(dmx_num, written);

  // Initialize the response to the default values
  if (response != NULL) {
    response->err = ESP_OK;
    response->type = RDM_RESPONSE_TYPE_NONE;
    response->num_params = 0;
  }

  // Wait for a response if necessary
  size_t num_params = 0;
  if (uid != RDM_BROADCAST_UID) {
    // Wait for a response 
    dmx_event_t packet;
    const size_t read = dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK);
    if (packet.err) {
      response->err = packet.err;
    } else if (read) {
      rdm_header_t header;
      if (!rdm_decode_header(driver->data.buffer, read, &header)) {
        response->err = ESP_ERR_INVALID_RESPONSE;
      } else if (!header.checksum_is_valid) {
        response->err = ESP_ERR_INVALID_CRC;
      } // TODO: more error checking

      // Read the data into a buffer
      response->type = header.response_type;
      if (response->type == RDM_RESPONSE_TYPE_ACK) {
        num_params = rdm_decode_device_info((rdm_data_t *)driver->data.buffer,
                                            read, device_info);
        response->num_params = num_params;
      }

    }
  } else {
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  }

  xSemaphoreGiveRecursive(driver->mux);

  return num_params;
}

/*
size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_uid_t uid,
                                      uint16_t sub_device,
                                      rdm_response_t *response,
                                      rdm_software_version_label_t *param) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Take mutex so driver values may be accessed
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Prepare the RDM request
  uint8_t request[RDM_BASE_PACKET_SIZE];
  rdm_data_t *rdm = (rdm_data_t *)request;
  rdm->sc = RDM_SC;
  rdm->sub_sc = RDM_SUB_SC;
  rdm->message_len = RDM_BASE_PACKET_SIZE - 2;  // exclude checksum
  uid_to_buf(rdm->destination_uid, uid);
  uid_to_buf(rdm->source_uid, rdm_get_uid());
  rdm->tn = driver->rdm_tn;
  rdm->port_id = dmx_num + 1;
  rdm->message_count = 0;
  rdm->sub_device = bswap16(sub_device);
  rdm->cc = RDM_CC_GET_COMMAND;
  rdm->pid = bswap16(RDM_PID_BOOT_SOFTWARE_VERSION_LABEL);
  rdm->pdl = 0;

  // Calculate the checksum
  uint16_t checksum = 0;
  for (int i = 0; i < rdm->message_len; ++i) {
    checksum += request[i];
  }
  *(uint16_t *)(&request[rdm->message_len]) = bswap16(checksum);

  // Send the RDM request
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, request, sizeof(request));
  dmx_send(dmx_num, 0);

  size_t response_size = 0;
  if (uid != RDM_BROADCAST_UID) {
    dmx_event_t dmx_event;
    response_size = dmx_receive(dmx_num, &dmx_event, DMX_TIMEOUT_TICK);
    if (response != NULL) {
      response->size = response_size;
    }
    if (dmx_event.err) {
      if (response != NULL) {
        response->checksum_is_valid = false;
      }
    } else if (response_size) {
      rdm_header_t rdm_header;
      rdm_decode_header(driver->data.buffer, response_size, &rdm_header);
      if (response != NULL) {
        response->checksum_is_valid = rdm_header.checksum_is_valid;
        response->type = rdm_header.response_type;
        // TODO: check for NACK
      }

      // Read the data into a buffer
      uint8_t response[RDM_BASE_PACKET_SIZE + 32];
      dmx_read(dmx_num, response, response_size);
      rdm = (rdm_data_t *)response;

      if (param != NULL) {
        if (rdm_header.pdl > 0) {
          strncpy(param->software_version_label, (char *)&rdm->pd, 32);
        } else {
          param->software_version_label[0] = 0;
        }
      }

    }
  } else {
    if (response != NULL) {
      response->size = 0;
    }
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  }
  xSemaphoreGiveRecursive(driver->mux);

  return response_size;
}
*/
