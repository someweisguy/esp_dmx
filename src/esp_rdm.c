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

bool rdm_decode_header(const void *source, size_t size, rdm_header_t *header) {
  RDM_CHECK(source != NULL, false, "source is null");
  RDM_CHECK(header != NULL, false, "header is null");

  bool is_rdm = false;
  rdm_data_t *rdm = (rdm_data_t *)source;

  if ((rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER) && size > 17) {
    // Decode a DISC_UNIQUE_BRANCH response

    // Find the length of the discovery response preamble (0-7 bytes)
    int preamble_len = 0;
    const uint8_t *data = source;
    for (; preamble_len < 7; ++preamble_len) {
      if (data[preamble_len] == RDM_DELIMITER) {
        break;
      }
    }
    if (data[preamble_len] != RDM_DELIMITER || size < preamble_len + 17) {
      return is_rdm;  // Not a valid discovery response
    }

    // Decode the 6-byte UID and get the packet sum
    rdm_uid_t uid = 0;
    uint16_t sum = 0;
    data = &((uint8_t *)source)[preamble_len + 1];
    for (int i = 5, j = 0; i >= 0; --i, j += 2) {
      ((uint8_t *)&uid)[i] = data[j] & 0x55;
      ((uint8_t *)&uid)[i] |= data[j + 1] & 0xaa;
      sum += ((uint8_t *)&uid)[i] + 0xff;
    }

    // Decode the checksum received in the response
    uint16_t checksum;
    for (int i = 1, j = 12; i >= 0; --i, j += 2) {
      ((uint8_t *)&checksum)[i] = data[j] & 0x55;
      ((uint8_t *)&checksum)[i] |= data[j + 1] & 0xaa;
    }

    // Return RDM data to the caller
    header->destination_uid = RDM_BROADCAST_UID;
    header->source_uid = uid;
    header->tn = 0;
    header->response_type = RDM_RESPONSE_TYPE_ACK;
    header->message_count = 0;
    header->sub_device = 0;
    header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
    header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    header->pdl = 0;
    header->checksum_is_valid = (sum == checksum);
    is_rdm = true;

  } else if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC && size >= 26) {
    // Decode a standard RDM message

    // Calculate the checksum
    uint16_t sum = 0;
    const uint16_t checksum = bswap16(*(uint16_t *)(source + rdm->message_len));
    for (int i = 0; i < rdm->message_len; ++i) {
      sum += *(uint8_t *)(source + i);
    }

    // Return RDM data to the caller
    header->destination_uid = buf_to_uid(rdm->destination_uid);
    header->source_uid = buf_to_uid(rdm->source_uid);
    header->tn = rdm->tn;
    header->response_type = rdm->response_type;
    header->message_count = rdm->message_count;
    header->sub_device = bswap16(rdm->sub_device);
    header->cc = rdm->cc;
    header->pid = bswap16(rdm->pid);
    header->pdl = rdm->pdl;
    header->checksum_is_valid = (sum == checksum);
    is_rdm = true;
  }

  return is_rdm;
}

size_t rdm_decode_params(const void *source, size_t size, void *params,
                         size_t num_params, size_t message_num) {
  RDM_CHECK(source != NULL, 0, "source is null");
  RDM_CHECK(params != NULL, 0, "params is null");
  RDM_CHECK(num_params == 0, 0, "num_params must be greater than 0");

  size_t params_available = 0;
  const rdm_data_t *data = source;

  // Guard against null pointer errors
  if (size < 26 || data->pdl == 0 || data->sc != RDM_SC ||
      data->sub_sc != RDM_SUB_SC) {
    return params_available;
  }

  // Search for the correct PID decoder list
  const rdm_pid_t pid = bswap16(data->pid);
  if (pid >= 0x0000 && pid < 0x0100) {
    if (pid == RDM_PID_DISC_MUTE || pid == RDM_PID_DISC_UN_MUTE) {
      params_available = rdm_decode_disc_mute(data, size, data->cc, params,
                                              num_params, message_num);
    } else if (pid == RDM_PID_SUPPORTED_PARAMETERS) {
      // TODO
    } else if (pid == RDM_PID_PARAMETER_DESCRIPTION) {
      // TODO
    } else if (pid == RDM_PID_DEVICE_INFO) {
      // TODO
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

  return params_available;
}

size_t rdm_encode(void *destination, size_t size, const rdm_header_t *header,
                  const void *params, size_t num_params, size_t message_num) {
  // TODO: arg check

  // Guard against null pointer errors
  if (size < 26) {
    return 0;
  }

  size_t bytes_encoded = 0;
  const rdm_cc_t cc = header->cc;
  const rdm_pid_t pid = header->pid;

  if (cc == RDM_CC_DISC_COMMAND_RESPONSE && pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // Encode DISC_UNIQUE_BRANCH response
    bytes_encoded = rdm_encode_disc_unique_branch_response(destination, size,
                                                           header->source_uid);
  } else {
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
        pdl = rdm_encode_disc_mute(buf, size, header->cc, params, num_params,
                                   message_num);
      } else if (pid == RDM_PID_SUPPORTED_PARAMETERS) {
        // TODO
      } else if (pid == RDM_PID_PARAMETER_DESCRIPTION) {
        // TODO
      } else if (pid == RDM_PID_DEVICE_INFO) {
        // TODO
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

  // Prepare the RDM message
  uint8_t data[RDM_BASE_PACKET_SIZE + 12];
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
  const size_t written = rdm_encode(data, sizeof(data), &header, params, 1, 0);

  // Send the RDM message
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, data, written);
  dmx_send(dmx_num, 0);

  // Wait for a response
  dmx_event_t dmx_event;
  size_t response_size = dmx_receive(dmx_num, &dmx_event, DMX_TIMEOUT_TICK);
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
      if (rdm_header.checksum_is_valid) {
        response->type = RDM_RESPONSE_TYPE_ACK;
      }
    }

    if (uid != NULL) {
      *uid = rdm_header.source_uid;
    }
  }
  xSemaphoreGiveRecursive(driver->mux);

  return response_size;
}

size_t rdm_send_disc_mute(dmx_port_t dmx_num, rdm_uid_t uid, bool mute,
                          rdm_response_t *response,
                          rdm_disc_mute_t *mute_params) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_pid_t pid = mute ? RDM_PID_DISC_MUTE : RDM_PID_DISC_UN_MUTE;

  // Take mutex so driver values may be accessed
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Prepare the RDM message
  uint8_t data[RDM_BASE_PACKET_SIZE];
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
  const size_t written = rdm_encode(data, sizeof(data), &header, NULL, 0, 0);

  // Send the RDM request
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, data, written);
  dmx_send(dmx_num, 0);

  // Determine if a response is expected
  size_t response_size = 0;
  if (uid != RDM_BROADCAST_UID) {
    // Receive the response
    dmx_event_t dmx_event;
    response_size = dmx_receive(dmx_num, &dmx_event, DMX_TIMEOUT_TICK);
    if (response != NULL) {
      response->size = response_size;
    }

    // Determine if checksum is valid
    if (dmx_event.err) {
      // Report invalid checksum on DMX error
      if (response != NULL) {
        response->checksum_is_valid = false;
      }
    } else if (response_size) {
      // Parse the RDM response
      rdm_header_t rdm_header;
      rdm_decode_header(driver->data.buffer, response_size, &rdm_header);
      if (response != NULL) {
        response->checksum_is_valid = rdm_header.checksum_is_valid;
        if (rdm_header.checksum_is_valid) {
          response->type = RDM_RESPONSE_TYPE_ACK;
        }
      }

      // Read the data into a buffer
      uint8_t response[RDM_BASE_PACKET_SIZE + 8];
      dmx_read(dmx_num, response, response_size);
      rdm_data_t *rdm = (rdm_data_t *)response;

      // Copy RDM packet parameters
      if (mute_params != NULL && rdm_header.pdl >= 2) {
        struct __attribute__((__packed__)) disc_mute_data_t {
          union {
            struct {
              uint8_t managed_proxy : 1;
              uint8_t sub_device : 1;
              uint8_t boot_loader : 1;
              uint8_t proxied_device : 1;
            };
            uint16_t control_field;
          };
          uint8_t binding_uid[6];
        } *p = (struct disc_mute_data_t *)&rdm->pd;

        mute_params->managed_proxy = p->managed_proxy;
        mute_params->sub_device = p->sub_device;
        mute_params->boot_loader = p->boot_loader;
        mute_params->proxied_device = p->proxied_device;

        if (rdm_header.pdl >= 8) {
          mute_params->binding_uid = buf_to_uid(p->binding_uid);
        } else {
          mute_params->binding_uid = 0;
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
      } while (response.size == 0 && ++attempts < 3);

      // Attempt to fix possible error where responder is flipping its own UID
      if (response.size == 0) {
        uid = bswap64(uid) >> 16;  // Flip UID
        rdm_send_disc_mute(dmx_num, uid, true, &response, &mute_params);
      }

      // Add the UID to the list
      if (response.size > 0 && response.checksum_is_valid) {
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
      } while (response.size == 0 && ++attempts < 3);
      if (response.size > 0) {
        bool devices_remaining = true;

#ifndef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
        /*
        Stop the RDM controller from branching all the way down to the
        individual address if it is not necessary. When debugging, this function
        should not be used as it can hide bugs in the discovery algorithm. Users
        can use the sdkconfig to enable or disable discovery debugging.
        */
        if (response.checksum_is_valid) {
          for (int quick_finds = 0; quick_finds < 3; ++quick_finds) {
            // Attempt to mute the device
            attempts = 0;
            rdm_disc_mute_t mute_params;
            do {
              rdm_send_disc_mute(dmx_num, uid, true, &response, &mute_params);
            } while (response.size == 0 && ++attempts < 3);

            // Add the UID to the list
            if (response.size > 0) {
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
            } while (response.size == 0 && ++attempts < 3);
            if (response.size > 0 && !response.checksum_is_valid) {
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
  rdm->pid = bswap16(RDM_PID_DEVICE_INFO);
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
      uint8_t response[RDM_BASE_PACKET_SIZE + 19];
      dmx_read(dmx_num, response, response_size);
      rdm = (rdm_data_t *)response;

      if (device_info != NULL && rdm_header.pdl >= 19) {
        struct __attribute__((__packed__)) dev_info_data_t {
          uint16_t rdm_version;
          uint16_t model_id;
          uint16_t product_category;
          uint32_t software_version;
          uint16_t footprint;
          uint8_t current_personality;
          uint8_t personality_count;
          uint16_t start_address;
          uint16_t sub_device_count;
          uint8_t sensor_count;
        } *p = (struct dev_info_data_t *)&rdm->pd;

        device_info->rdm_version = bswap16(p->rdm_version);
        device_info->model_id = bswap16(p->model_id);
        device_info->product_category = bswap16(p->product_category);
        device_info->software_version = bswap32(p->software_version);
        device_info->footprint = bswap16(p->footprint);
        device_info->current_personality = p->current_personality;
        device_info->personality_count = p->personality_count;
        device_info->start_address = bswap16(p->start_address);
        device_info->sub_device_count = bswap16(p->sub_device_count);
        device_info->sensor_count = p->sensor_count;
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
