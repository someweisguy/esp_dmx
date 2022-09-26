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
#include "rdm_constants.h"

// Used for argument checking at the beginning of each function.
#define RDM_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

static const char *TAG = "rdm";  // The log tagline for the file.

static rdm_uid_t rdm_uid = 0;  // The 48-bit unique ID of this device.

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

bool rdm_parse(void *data, size_t size, rdm_event_t *event) {
  RDM_CHECK(data != NULL, false, "data is null");
  RDM_CHECK(event != NULL, false, "event is null");

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
    rdm_uid_t uid = 0;
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

  } else if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC &&
             size >= RDM_BASE_PACKET_SIZE) {
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
  const rdm_uid_t uid = rdm_get_uid();
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
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, response, sizeof(response));
  return dmx_send(dmx_num, 0);
}

size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                   rdm_disc_unique_branch_t *params,
                                   rdm_response_t *response, rdm_uid_t *uid) {
  RDM_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  RDM_CHECK(params != NULL, 0, "params is null");
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
  uid_to_buf((void *)&rdm->pd, params->lower_bound);
  uid_to_buf((void *)&rdm->pd + 6, params->upper_bound);

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

  // Wait for a response
  dmx_event_t dmx_event;
  size_t response_size = dmx_receive(dmx_num, &dmx_event, DMX_TIMEOUT_TICK);
  if (response != NULL) {
    response->size = response_size;
  }
  if (dmx_event.err && dmx_event.err != DMX_ERR_DATA_COLLISION) {
    if (response != NULL) {
      response->err = RDM_FAIL;
    }
  } else if (response_size) {
    rdm_event_t rdm_event;
    rdm_parse(driver->data.buffer, response_size, &rdm_event);
    if (response != NULL) {
      if (rdm_event.cc != RDM_CC_DISC_COMMAND_RESPONSE ||
          rdm_event.pid != RDM_PID_DISC_UNIQUE_BRANCH) {
        response->err = RDM_INVALID_RESPONSE;
      } else if (!rdm_event.checksum_is_valid) {
        response->err = RDM_INVALID_CHECKSUM;
      } else {
        response->err = RDM_OK;
        response->type = RDM_RESPONSE_TYPE_ACK;
      }
    }

    if (uid != NULL) {
      *uid = rdm_event.source_uid;
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
  dmx_write(dmx_num, request, sizeof(request));
  dmx_send(dmx_num, 0);

  // Determine if a response is expected
  size_t response_size = 0;
  if (uid != RDM_BROADCAST_UID) {
    dmx_event_t dmx_event;
    response_size = dmx_receive(dmx_num, &dmx_event, DMX_TIMEOUT_TICK);
    if (response != NULL) {
      response->size = response_size;
    }
    if (dmx_event.err) {
      if (response != NULL) {
        response->err = RDM_FAIL;
      }
    } else if (response_size) {
      rdm_event_t rdm_event;
      rdm_parse(driver->data.buffer, response_size, &rdm_event);
      if (response != NULL) {
        if (rdm_event.cc != RDM_CC_DISC_COMMAND_RESPONSE ||
            rdm_event.pid != request_pid) {
          response->err = RDM_INVALID_RESPONSE;
        } else if (!rdm_event.checksum_is_valid) {
          response->err = RDM_INVALID_CHECKSUM;
        } else {
          response->err = RDM_OK;
          response->type = RDM_RESPONSE_TYPE_ACK;
        }
      }

      // Read the data into a buffer
      uint8_t response[RDM_BASE_PACKET_SIZE + 8];
      dmx_read(dmx_num, response, dmx_event.size);
      rdm = (rdm_data_t *)response;

      // Copy RDM packet parameters
      if (mute_params != NULL && rdm_event.pdl >= 2) {
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

        if (rdm_event.pdl >= 8) {
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

bool rdm_quick_find(dmx_port_t dmx_num, rdm_disc_unique_branch_t *params,
                    rdm_uid_t uid, const size_t size, rdm_uid_t *const uids,
                    size_t *const found) {
  rdm_response_t response;
  int attempts = 0;

  // Attempt to mute the device
  rdm_disc_mute_t mute_params;
  do {
    rdm_send_disc_mute(dmx_num, uid, true, &response, &mute_params);
  } while (response.size == 0 && attempts++ < 3);

  // Add the UID to the list
  if (response.size > 0) {
    if (*found < size && uids != NULL) {
      uids[*found] = mute_params.binding_uid ? mute_params.binding_uid : uid;
    }
    ++(*found);
  }

  // Check if there are more devices in this branch
  attempts = 0;
  do {
    rdm_send_disc_unique_branch(dmx_num, params, &response, &uid);
  } while (response.size == 0 && attempts++ < 3);
  if (response.size > 0 && !response.err) {
    // There is another single device in this branch
    return rdm_quick_find(dmx_num, params, uid, size, uids, found);
  } else if (response.size > 0 && response.err) {
    // There are more devices in this branch - branch further
    return true;
  } else {
    // There are no more devices in this branch
    return false;
  }
}

void rdm_find_devices(dmx_port_t dmx_num, rdm_disc_unique_branch_t *bounds,
                      const size_t size, rdm_uid_t *const uids,
                      size_t *const found) {
  rdm_response_t response;
  rdm_uid_t uid;
  int attempts = 0;

  if (bounds->lower_bound == bounds->upper_bound) {
    // Can't branch further so attempt to mute the device
    uid = bounds->lower_bound;
    
    rdm_disc_mute_t mute_params;
    do {
      rdm_send_disc_mute(dmx_num, uid, true, &response, &mute_params);
    } while (response.size == 0 && attempts++ < 3);

    // Attempt to fix possible error where responder is flipping its own UID
    if (response.size == 0) {
      uid = bswap64(uid) >> 16;  // Flip UID
      rdm_send_disc_mute(dmx_num, uid, true, &response, &mute_params);
    }

    // Add the UID to the list
    if (response.size > 0 && !response.err) {
      if (*found < size && uids != NULL) {
        uids[*found] = mute_params.binding_uid ? mute_params.binding_uid : uid;
      }
      ++(*found);
    }
  } else {  // lower_bound != upper_bound
    // Search the current branch in the RDM address space
    do {
      rdm_send_disc_unique_branch(dmx_num, bounds, &response, &uid);
    } while (response.size == 0 && attempts++ < 3);
    if (response.size > 0 && !response.err) {
      bool devices_remaining = true;

#ifndef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
      /*
      Stop the RDM controller from branching all the way down to the individual
      address if it is not necessary. When debugging, this function should not 
      be used as it can hide bugs in the discovery algorithm. Users can use the 
      sdkconfig to enable or disable discovery debugging.
      */
      if (!response.err) {
        rdm_disc_unique_branch_t new_params = *bounds;
        devices_remaining =
            rdm_quick_find(dmx_num, &new_params, uid, size, uids, found);
      }
#endif

      // Recursively search the next two RDM address spaces
      if (devices_remaining) {
        // The following variables MUST be declared volatile
        volatile const rdm_uid_t temp_upper = bounds->upper_bound;
        volatile const rdm_uid_t mid = (bounds->lower_bound + temp_upper) / 2;

        bounds->upper_bound = mid;
        rdm_find_devices(dmx_num, bounds, size, uids, found);

        bounds->lower_bound = mid + 1;
        bounds->upper_bound = temp_upper;
        rdm_find_devices(dmx_num, bounds, size, uids, found);
      }
    }
  }
}

#ifndef CONFIG_RDM_STATIC_DEVICE_DISCOVERY
struct rdm_disc_args_t {
  dmx_port_t dmx_num;
  rdm_uid_t *uids;
  size_t *found;
  size_t size;
  SemaphoreHandle_t sem;
};

static void rdm_dev_disc_task(void *args) {
  struct rdm_disc_args_t *disc = (struct rdm_disc_args_t *)args;
  dmx_driver_t *const driver = dmx_driver[disc->dmx_num];

  // Mutex must be taken in the same task as the discovery
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  rdm_send_disc_mute(disc->dmx_num, RDM_BROADCAST_UID, false, NULL, NULL);
  rdm_disc_unique_branch_t disc_params = {
    .upper_bound = RDM_MAX_UID,
    .lower_bound = 0
  };
  rdm_find_devices(disc->dmx_num, &disc_params, disc->size, disc->uids,
                   disc->found);
  xSemaphoreGiveRecursive(driver->mux);

#ifdef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
  const size_t hwm = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGI(TAG, "Discovery task high water mark is %i words.", hwm);
#endif

  // Signal task complete
  xSemaphoreGive(disc->sem);
  vTaskDelete(NULL);
}
#endif

size_t rdm_discover_devices(dmx_port_t dmx_num, size_t size, rdm_uid_t *uids) {
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
  StaticSemaphore_t buffer;
  struct rdm_disc_args_t disc = {
      .dmx_num = dmx_num,
      .uids = uids,
      .found = &devices_found,
      .size = size,
      .sem = xSemaphoreCreateBinaryStatic(&buffer)};
  const size_t stack_size = 5632;  // 22KB - use with caution!
  // TODO: number the discovery by the DMX port number
  xTaskCreate(&rdm_dev_disc_task, "RDM Discovery", stack_size, &disc,
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

#ifdef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
  const size_t hwm = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGI(TAG, "Discovery high water mark is %i words.", hwm);
#endif

#endif

  return devices_found;
}

/*
size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_uid_t uid,
                           uint16_t sub_device, rdm_device_info_t *device_info,
                           bool *response_is_valid) {
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

  dmx_event_t event;
  size_t response_size = 0;
  if (uid != RDM_BROADCAST_UID) {
    response_size = dmx_receive(dmx_num, &event, DMX_TIMEOUT_TICK);
    if (response_size) {
      if (response_is_valid != NULL) {
        if (!event.err && event.is_rdm && event.rdm.checksum_is_valid &&
            event.rdm.cc == RDM_CC_GET_COMMAND_RESPONSE &&
            event.rdm.pid == RDM_PID_DEVICE_INFO &&
            event.rdm.source_uid == uid &&
            event.rdm.destination_uid == rdm_get_uid()) {
          *response_is_valid = true;
        } else {
          *response_is_valid = false;
        }
      }

      // Read the data into a buffer
      uint8_t response[RDM_BASE_PACKET_SIZE + 19];
      dmx_read(dmx_num, response, event.size);
      rdm = (rdm_data_t *)response;

      if (device_info != NULL && event.rdm.pdl >= 19) {
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
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  }
  xSemaphoreGiveRecursive(driver->mux);

  return response_size;
}
*/

/*
size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_uid_t uid,
                                      uint16_t sub_device,
                                      rdm_software_version_label_t *param,
                                      bool *response_is_valid) {
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

  dmx_event_t event;
  size_t response_size = 0;
  if (uid != RDM_BROADCAST_UID) {
    response_size = dmx_receive(dmx_num, &event, DMX_TIMEOUT_TICK);
    if (response_size) {
      if (response_is_valid != NULL) {
        if (!event.err && event.is_rdm && event.rdm.checksum_is_valid &&
            event.rdm.cc == RDM_CC_GET_COMMAND_RESPONSE &&
            event.rdm.pid == RDM_PID_BOOT_SOFTWARE_VERSION_LABEL &&
            event.rdm.response_type == RDM_RESPONSE_TYPE_ACK && 
            event.rdm.source_uid == uid &&
            event.rdm.destination_uid == rdm_get_uid()) {
          *response_is_valid = true;
        } else {
          *response_is_valid = false;
        }
      }

      // Read the data into a buffer
      uint8_t response[RDM_BASE_PACKET_SIZE + 32];
      dmx_read(dmx_num, response, event.size);
      rdm = (rdm_data_t *)response;

      if (param != NULL) {
        if (event.rdm.pdl > 0) {
          strncpy(param->software_version_label, (char *)&rdm->pd, 32);
        } else {
          param->software_version_label[0] = 0;
        }
      }

    }
  } else {
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  }
  xSemaphoreGiveRecursive(driver->mux);

  return response_size;
}
*/