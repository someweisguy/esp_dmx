#include "rdm_tools.h"

#include <stdint.h>
#include <string.h>

#include "dmx_constants.h"
#include "endian.h"
#include "esp_check.h"
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "impl/driver.h"
#include "rdm_constants.h"

// Used for argument checking at the beginning of each function.
#define RDM_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

static const char *TAG = "rdm";  // The log tagline for the file.

static int64_t rdm_uid = 0;  // The 48-bit unique ID of this device.

IRAM_ATTR int64_t buf_to_uid(const void *buf) {
  uint64_t val = 0;
  ((uint8_t *)&val)[5] = ((uint8_t *)buf)[0];
  ((uint8_t *)&val)[4] = ((uint8_t *)buf)[1];
  ((uint8_t *)&val)[3] = ((uint8_t *)buf)[2];
  ((uint8_t *)&val)[2] = ((uint8_t *)buf)[3];
  ((uint8_t *)&val)[1] = ((uint8_t *)buf)[4];
  ((uint8_t *)&val)[0] = ((uint8_t *)buf)[5];
  return val;
}

void *uid_to_buf(void *buf, int64_t uid) {
  ((uint8_t *)buf)[0] = ((uint8_t *)&uid)[5];
  ((uint8_t *)buf)[1] = ((uint8_t *)&uid)[4];
  ((uint8_t *)buf)[2] = ((uint8_t *)&uid)[3];
  ((uint8_t *)buf)[3] = ((uint8_t *)&uid)[2];
  ((uint8_t *)buf)[4] = ((uint8_t *)&uid)[1];
  ((uint8_t *)buf)[5] = ((uint8_t *)&uid)[0];
  return buf;
}

int64_t rdm_get_uid() {
  // Initialize the RDM UID
  if (rdm_uid == 0) {
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    rdm_uid = (int64_t)RDM_DEFAULT_MANUFACTURER_ID << 32;
    rdm_uid |= bswap32(*(uint32_t *)(mac + 2));
  }

  return rdm_uid;
}

void rdm_set_uid(int64_t uid) { rdm_uid = uid; }

bool rdm_parse(void *data, size_t size, rdm_event_t *event) {
  // TODO: check args

  const rdm_data_t *const rdm = (rdm_data_t *)data;

  if ((rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER) && size > 17) {
    // Find the length of the discovery response preamble (0-7 bytes)
    int preamble_len = 0;
    const uint8_t *response = data;
    for (; preamble_len < 7; ++preamble_len) {
      if (response[preamble_len] == RDM_DELIMITER) {
        break;
      }
    }
    if (response[preamble_len] != RDM_DELIMITER || size < preamble_len + 17) {
      return false;  // Not a valid discovery response
    }

    // Decode the 6-byte UID and get the packet sum
    uint64_t uid = 0;
    uint16_t sum = 0;
    response = &((uint8_t *)data)[preamble_len + 1];
    for (int i = 5, j = 0; i >= 0; --i, j += 2) {
      ((uint8_t *)&uid)[i] = response[j] & 0x55;
      ((uint8_t *)&uid)[i] |= response[j + 1] & 0xaa;
      sum += ((uint8_t *)&uid)[i] + 0xff;
    }

    // Decode the checksum received in the response
    uint16_t checksum;
    for (int i = 1, j = 12; i >= 0; --i, j += 2) {
      ((uint8_t *)&checksum)[i] = response[j] & 0x55;
      ((uint8_t *)&checksum)[i] |= response[j + 1] & 0xaa;
    }

    // Return DMX data to the caller
    event->cc = RDM_CC_DISC_COMMAND_RESPONSE;
    event->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    event->source_uid = uid;
    event->checksum_is_valid = (sum == checksum);
    return (sum == checksum);

  } else if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC) {
    // Verify the packet checksum
    uint16_t sum = 0;
    for (int i = 0; i < rdm->message_len; ++i) {
      sum += ((uint8_t *)data)[i];
    }
    const uint16_t checksum = bswap16(*(uint16_t *)(data + rdm->message_len));
    event->checksum_is_valid = (sum == checksum);

    // Copy the packet data to the event if the checksum is valid
    event->destination_uid = buf_to_uid(rdm->destination_uid);
    event->source_uid = buf_to_uid(rdm->source_uid);
    event->tn = rdm->tn;
    event->port_id = rdm->port_id;  // Also copies response_type
    event->message_count = rdm->message_count;
    event->sub_device = bswap16(rdm->sub_device);
    event->cc = rdm->cc;
    event->pid = bswap16(rdm->pid);
    event->pdl = rdm->pdl;
    return (sum == checksum);
  }

  return false;
}

size_t rdm_send_disc_response(dmx_port_t dmx_num) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // TODO: Return 0 if the driver is muted

  // Prepare and encode the response
  uint8_t response[24] = {RDM_PREAMBLE, RDM_PREAMBLE, RDM_PREAMBLE,
                          RDM_PREAMBLE, RDM_PREAMBLE, RDM_PREAMBLE,
                          RDM_PREAMBLE, RDM_DELIMITER};
  const uint64_t uid = rdm_get_uid();
  uint16_t checksum = 0;
  for (int i = 8, j = 5; i < 20; i += 2, --j) {
    response[i] = ((uint8_t *)&uid)[j] | 0xaa;
    response[i + 1] = ((uint8_t *)&uid)[j] | 0x55;
    checksum += ((uint8_t *)&uid)[j] + (0xaa + 0x55);
  }
  response[20] = (checksum >> 8) | 0xaa;
  response[21] = (checksum >> 8) | 0x55;
  response[22] = checksum | 0xaa;
  response[23] = checksum | 0x55;

  // Write and send the response
  dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  dmx_write(dmx_num, response, sizeof(response));
  return dmx_send(dmx_num, 0);
}

size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num, int64_t lower_bound,
                                   int64_t upper_bound, int64_t *response_uid,
                                   bool *response_is_valid) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Take mutex so driver values may be accessed
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Prepare the RDM request
  uint8_t request[RDM_BASE_PACKET_SIZE + 12];
  rdm_data_t *rdm = (rdm_data_t *)request;
  rdm->sc = RDM_SC;
  rdm->sub_sc = RDM_SUB_SC;
  rdm->message_len = RDM_BASE_PACKET_SIZE + 12 - 2;
  uid_to_buf(rdm->destination_uid, RDM_BROADCAST_UID);
  uid_to_buf(rdm->source_uid, rdm_get_uid());
  rdm->tn = driver->rdm_tn;
  rdm->port_id = dmx_num + 1;
  rdm->message_count = 0;
  rdm->sub_device = bswap16(0);
  rdm->cc = RDM_CC_DISC_COMMAND;
  rdm->pid = bswap16(RDM_PID_DISC_UNIQUE_BRANCH);
  rdm->pdl = 12;

  // Prepare the RDM request parameters
  uid_to_buf((void *)&rdm->pd, lower_bound);
  uid_to_buf((void *)&rdm->pd + 6, upper_bound);

  // Calculate the checksum
  uint16_t checksum = 0;
  for (int i = 0; i < rdm->message_len; ++i) {
    checksum += request[i];
  }
  *(uint16_t *)(&request[rdm->message_len]) = bswap16(checksum);

  // Send the RDM request
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, request, rdm->message_len + 2);
  dmx_send(dmx_num, 0);

  // Wait for a response
  dmx_event_t event;
  const size_t response_size = dmx_receive(dmx_num, &event, pdMS_TO_TICKS(10));
  if (response_size) {
    if (response_is_valid != NULL) {
      if (!event.err && event.is_rdm && event.rdm.checksum_is_valid &&
          event.rdm.cc == RDM_CC_DISC_COMMAND_RESPONSE &&
          event.rdm.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
        *response_is_valid = true;
      } else {
        *response_is_valid = false;
      }
    }
    
    if (response_uid != NULL) {
      *response_uid = event.rdm.source_uid;
    }
  }
  xSemaphoreGiveRecursive(driver->mux);

  return response_size;
}

size_t rdm_send_disc_mute(dmx_port_t dmx_num, int64_t uid, bool mute,
                          uint16_t *control_field, int64_t *binding_uid,
                          bool *response_is_valid) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  const uint16_t request_pid = mute ? RDM_PID_DISC_MUTE : RDM_PID_DISC_UN_MUTE;

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
  rdm->sub_device = bswap16(0);
  rdm->cc = RDM_CC_DISC_COMMAND;
  rdm->pid = bswap16(request_pid);
  rdm->pdl = 0;

  // Calculate the checksum
  uint16_t checksum = 0;
  for (int i = 0; i < rdm->message_len; ++i) {
    checksum += request[i];
  }
  *(uint16_t *)(&request[rdm->message_len]) = bswap16(checksum);

  // Send the RDM request
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, request, rdm->message_len + 2);
  dmx_send(dmx_num, 0);

  // Determine if a response is expected
  dmx_event_t event;
  size_t response_size = 0;
  if (uid != RDM_BROADCAST_UID) {
    response_size = dmx_receive(dmx_num, &event, DMX_TIMEOUT_TICK);
    if (response_size) {
      if (response_is_valid != NULL) {
        if (!event.err && event.is_rdm && event.rdm.checksum_is_valid &&
            event.rdm.cc == RDM_CC_DISC_COMMAND_RESPONSE &&
            event.rdm.pid == request_pid && event.rdm.source_uid == uid &&
            event.rdm.destination_uid == rdm_get_uid()) {
          *response_is_valid = true;
        } else {
          *response_is_valid = false;
        }
      }

      // Read the data into a buffer
      uint8_t response[RDM_BASE_PACKET_SIZE + 8];
      dmx_read(dmx_num, response, event.size);
      rdm = (rdm_data_t *)response;

      /*
       * Number of Discovery Mute/Un-Mute RDM Parameters: 1
       *   control_field:  2 bytes
       *   binding_uid:    6 bytes, optional
       */

      // Copy RDM packet parameters
      if (control_field != NULL) {
        if (event.rdm.pdl >= 2) {
          *control_field = bswap16(*(uint16_t *)(&rdm->pd));
        } else {
          *control_field = 0;
        }
      }
      if (binding_uid != NULL) {
        if (event.rdm.pdl >= 8) {
          *binding_uid = buf_to_uid((void *)&rdm->pd + 2);
        } else {
          *binding_uid = 0;
        }
      }
    }
  } else {
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  }
  xSemaphoreGiveRecursive(driver->mux);

  return response_size;
}

static bool rdm_quick_find(dmx_port_t dmx_num, int64_t lower_bound, 
                           int64_t upper_bound, int64_t uid, const size_t size, 
                           int64_t *const uids, size_t *const found) {
  bool response_is_valid;
  size_t response;
  int attempts = 0;

  // Attempt to mute the device
  int64_t binding_uid;
  do {
    response = rdm_send_disc_mute(dmx_num, uid, true, NULL, &binding_uid,
                                  &response_is_valid);
  } while (!response && attempts++ < 3);

  // Add the UID to the list
  if (response && response_is_valid) {
    if (*found < size && uids != NULL) {
      uids[*found] = binding_uid ? binding_uid : uid;
    }
    ++(*found);
  }

  // Check if there are more devices in this branch
  attempts = 0;
  do {
    response = rdm_send_disc_unique_branch(dmx_num, lower_bound, upper_bound,
                                           &uid, &response_is_valid);
  } while (!response && attempts++ < 3);
  if (response && response_is_valid) {
    // There is another single device in this branch
    return rdm_quick_find(dmx_num, lower_bound, upper_bound, uid, size, uids,
                          found);
  } else if (response && !response_is_valid) {
    // There are more devices in this branch - branch further
    return true;
  } else {
    // There are no more devices in this branch
    return false;
  }
}

static void rdm_find_devices(dmx_port_t dmx_num, int64_t lower_bound,
                             int64_t upper_bound, const size_t size,
                             int64_t *const uids, size_t *const found) {
  bool response_is_valid;
  size_t response;
  int attempts = 0;

  if (lower_bound == upper_bound) {
    // Can't branch further so attempt to mute the device
    int64_t binding_uid = 0;
    do {
      response = rdm_send_disc_mute(dmx_num, lower_bound, true, NULL,
                                    &binding_uid, &response_is_valid);
    } while (!response && attempts++ < 3);

    // Attempt to fix possible error where responder is flipping its own UID
    if (!(response && response_is_valid)) {
      lower_bound = bswap64(lower_bound) >> 16;  // Flip UID
      response = rdm_send_disc_mute(dmx_num, lower_bound, true, NULL,
                                    &binding_uid, &response_is_valid);
    }

    // Add the UID to the list
    if (response && response_is_valid) {
      if (*found < size && uids != NULL) {
        uids[*found] = binding_uid ? binding_uid : lower_bound;
      }
      ++(*found);
    }
  } else {  // lower_bound != upper_bound
    // Search the current branch in the RDM address space
    int64_t uid;
    do {
      response = rdm_send_disc_unique_branch(dmx_num, lower_bound, upper_bound,
                                             &uid, &response_is_valid);
    } while (!response && attempts++ < 3);
    if (response) {
      bool devices_remaining = true;

#ifndef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
      /*
      Stop the RDM controller from branching all the way down to the individual
      address if it is not necessary. When debugging, this function should not 
      be used as it can hide bugs in the discovery algorithm. Users can use the 
      sdkconfig to enable or disable discovery debugging.
      */
      if (response_is_valid) {
        devices_remaining = rdm_quick_find(dmx_num, lower_bound, upper_bound,
                                           uid, size, uids, found);
      }
#endif

      // Recursively search the next two RDM address spaces
      if (devices_remaining) {
        const int64_t mid = (lower_bound + upper_bound) / 2;
        rdm_find_devices(dmx_num, lower_bound, mid, size, uids, found);
        rdm_find_devices(dmx_num, mid + 1, upper_bound, size, uids, found);
      }
    }
  }
}

#ifndef CONFIG_RDM_STATIC_DEVICE_DISCOVERY
struct rdm_disc_args_t {
  dmx_port_t dmx_num;
  int64_t *uids;
  size_t *found;
  size_t size;
  SemaphoreHandle_t sem;
};

static void rdm_dev_disc_task(void *args) {
  struct rdm_disc_args_t *disc = (struct rdm_disc_args_t *)args;
  dmx_driver_t *const driver = dmx_driver[disc->dmx_num];

  // Mutex must be taken in the same task as the discovery
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  rdm_send_disc_mute(disc->dmx_num, RDM_BROADCAST_UID, false, NULL, NULL, NULL);
  rdm_find_devices(disc->dmx_num, 0, RDM_MAX_UID, disc->size, disc->uids,
                   disc->found);
  xSemaphoreGiveRecursive(driver->mux);

  // Signal task complete
  xSemaphoreGive(disc->sem);
  vTaskDelete(NULL);
}
#endif

size_t rdm_discover_devices(dmx_port_t dmx_num, size_t size, int64_t *uids) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t devices_found = 0;

#ifndef CONFIG_RDM_STATIC_DEVICE_DISCOVERY
  /*
  By default, the ESP32 main task does not have enough stack space to execute
  the RDM discovery algorithm all the way down to the bottom branch. Running the
  RDM discovery algorithm on the main task using the default stack size results
  in a stack overflow crash. To fix this error, a new task with an appropriately
  sized stack is needed.
  */
  UBaseType_t priority = 1;
#if (INCLUDE_uxTaskPriorityGet == 1)
  priority = uxTaskPriorityGet(NULL);
#endif
  struct rdm_disc_args_t disc;
  disc.dmx_num = dmx_num;
  disc.uids = uids;
  disc.found = &devices_found;
  disc.size = size;
  StaticSemaphore_t buffer;
  disc.sem = xSemaphoreCreateBinaryStatic(&buffer);
  const size_t stack_size = 5120;  // 20.5KB - use with caution!
  xTaskCreate(&rdm_dev_disc_task, "RDM Device Discovery", stack_size, &disc,
              priority, NULL);
  xSemaphoreTake(disc.sem, portMAX_DELAY);
  vSemaphoreDelete(disc.sem);
#else
  /*
  Users may enable discovery without allocating extra memory by enabling this 
  option in the ESP32 sdkconfig. This must be done with caution as it may result
  in stack overflows!
  */
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  rdm_send_disc_mute(dmx_num, RDM_BROADCAST_UID, false, NULL, NULL, NULL);
  rdm_find_devices(dmx_num, 0, RDM_MAX_UID, size, uids, &devices_found);

  xSemaphoreGiveRecursive(driver->mux);
#endif

  return devices_found;
}

size_t rdm_get_device_info(dmx_port_t dmx_num, int64_t uid,
                           rdm_device_info_t *device_info) {
  // TODO
  return 0;
}
