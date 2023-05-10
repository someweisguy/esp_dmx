#include "controller.h"

#include <string.h>

#include "dmx/driver.h"
#include "dmx/hal.h"
#include "dmx/types.h"
#include "endian.h"
#include "esp_log.h"
#include "rdm/parameters.h"
#include "rdm/read_write.h"
#include "rdm/utils.h"

static const char *TAG = "rdm_controller";

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

/**
 * @brief A packed struct which can be used to help process raw RDM packets
 * instead of reading slots by index alone. RDM sends data in most-significant
 * byte first - endianness must be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_raw_t {
  uint8_t sc;           // The RDM start code.
  uint8_t sub_sc;       // The RDM sub-start code.
  uint8_t message_len;  // Number of slots in the RDM packet excluding checksum.
  uint8_t dest_uid[6];  // The UID of the target device(s).
  uint8_t src_uid[6];   // The UID of the device originating this packet.
  uint8_t tn;           // The transaction number.
  union {
    uint8_t port_id;        // The requesting device's port ID.
    uint8_t response_type;  // The responding device's response type.
  };
  uint8_t message_count;  // The number of packets in the device's packet queue.
  uint16_t sub_device;    // The destination sub-device.
  uint8_t cc;             // The command class.
  uint16_t pid;           // The parameter ID.
  uint8_t pdl;            // The parameter data length.
  struct {
  } pd;  // The RDM parameter data. It can be variable length.
} rdm_raw_t;

bool rdm_read(dmx_port_t dmx_num, rdm_header_t *header, rdm_mdb_t *mdb) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(header, false, "header is null");
  DMX_CHECK(mdb, false, "mdb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // FIXME: make this function thread-safe

  // Verify that the checksum is correct
  uint16_t sum = 0;
  uint16_t checksum;
  size_t preamble_len;
  const uint8_t sc = driver->data.buffer[0];
  if (sc == RDM_SC) {
    // Calculate sum and decode checksum normally
    const size_t message_len = ((rdm_raw_t *)driver->data.buffer)->message_len;
    for (int i = 0; i < message_len; ++i) {
      sum += driver->data.buffer[i];
    }
    checksum = bswap16(*(uint16_t *)(&driver->data.buffer[message_len]));
  } else {
    // Decode checksum from encoded DISC_UNIQUE_BRANCH response
    preamble_len = get_preamble_len(driver->data.buffer);
    const uint8_t *d = &driver->data.buffer[preamble_len + 1];
    for (int i = 0; i < 12; ++i) {
      sum += d[i];
    }
    checksum = (d[12] & d[13]) << 8;
    checksum += (d[14] & d[15]);
  }
  bool checksum_is_valid = (sum == checksum);
  if (!checksum_is_valid) {
    return checksum_is_valid;
  }

  // Decode the packet
  if (sc == RDM_SC) {
    const rdm_raw_t *const rdm = (void *)driver->data.buffer;

    // Assign or ignore the parameter data
    const size_t pdl = rdm->pdl;
    if (pdl > 231) {
      return false;  // PDL must be <= 231
    } else if (pdl > 0) {
      memcpy(mdb->pd, &rdm->pd, pdl);
    }
    mdb->pdl = pdl;

    // Copy the remaining header data
    header->dest_uid = bswap48(rdm->dest_uid);
    header->src_uid = bswap48(rdm->src_uid);
    header->tn = rdm->tn;
    header->port_id = rdm->port_id;  // Also copies response_type
    header->message_count = rdm->message_count;
    header->sub_device = bswap16(rdm->sub_device);
    header->cc = rdm->cc;
    header->pid = bswap16(rdm->pid);

  } else {
    // Decode the EUID
    uint8_t buf[6];
    const uint8_t *d = &driver->data.buffer[preamble_len + 1];
    for (int i = 0, j = 0; i < 6; ++i, j += 2) {
      buf[i] = d[j] & d[j + 1];
    }
    header->src_uid = bswap48(buf);

    // Fill out the remaining header and MDB data
    header->dest_uid = 0;
    header->tn = -1;
    header->response_type = RDM_RESPONSE_TYPE_ACK;  // Also copies port_id
    header->message_count = -1;
    header->sub_device = -1;
    header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
    header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    mdb->pdl = 0;
    mdb->preamble_len = preamble_len;
  }

  return checksum_is_valid;
}

size_t rdm_write(dmx_port_t dmx_num, const rdm_header_t *header,
                 const rdm_mdb_t *mdb) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  if (mdb != NULL && !(header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
                       header->cc == RDM_CC_DISC_COMMAND_RESPONSE)) {
    if (mdb->pdl > 231) {
      ESP_LOGE(TAG, "pdl is invalid");
      return 0;
    } else if (mdb->pdl > 0 && mdb->pd == NULL) {
      ESP_LOGE(TAG, "pd is null");
      return 0;
    }
  }

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  uart_dev_t *const restrict uart = driver->uart;

  taskENTER_CRITICAL(spinlock);
  if (driver->is_sending) {
    // Do not allow asynchronous writes when sending an any packet
    taskEXIT_CRITICAL(spinlock);
    return 0;
  } else if (dmx_uart_get_rts(uart) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_uart_set_rts(uart, 0);
  }
  taskEXIT_CRITICAL(spinlock);

  // TODO: make this function and the other `_write_` functions thread-safe

  size_t encoded;
  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
      header->cc == RDM_CC_DISC_COMMAND_RESPONSE) {
    // Encode the preamble
    const size_t preamble_len = mdb->preamble_len > 7 ? 7 : mdb->preamble_len;
    for (int i = 0; i < preamble_len; ++i) {
      driver->data.buffer[i] = RDM_PREAMBLE;
    }
    driver->data.buffer[preamble_len] = RDM_DELIMITER;

    // Encode the EUID and calculate the checksum
    uint16_t checksum = 0;
    uint8_t *d = &(driver->data.buffer[mdb->preamble_len + 1]);
    for (int i = 0, j = 5; i < 12; i += 2, --j) {
      d[i] = ((uint8_t *)&(header->src_uid))[j] | 0xaa;
      d[i + 1] = ((uint8_t *)&(header->src_uid))[j] | 0x55;
      checksum += ((uint8_t *)&(header->src_uid))[j] + 0xaa + 0x55;
    }

    // Encode the checksum
    d[12] = (checksum >> 8) | 0xaa;
    d[13] = (checksum >> 8) | 0x55;
    d[14] = (checksum & 0xff) | 0xaa;
    d[15] = (checksum & 0xff) | 0x55;

    encoded = mdb->preamble_len + 1 + 16;
  } else {
    rdm_raw_t *const rdm = (void *)driver->data.buffer;

    // Encode the parameter data
    if (mdb != NULL) {
      if (mdb->pdl > 0) {
        memcpy(&rdm->pd, mdb->pd, mdb->pdl);
      }
      rdm->pdl = mdb->pdl;
    } else {
      rdm->pdl = 0;
    }

    // Encode the packet header
    const size_t message_len = 24 + mdb->pdl;
    rdm->sc = RDM_SC;
    rdm->sub_sc = RDM_SUB_SC;
    rdm->message_len = message_len;
    uidcpy(rdm->dest_uid, &(header->dest_uid));
    uidcpy(rdm->src_uid, &(header->src_uid));
    rdm->tn = header->tn;
    rdm->port_id = header->port_id;  // Also copies response_type
    rdm->message_count = header->message_count;
    rdm->sub_device = bswap16(header->sub_device);
    rdm->cc = header->cc;
    rdm->pid = bswap16(header->pid);

    // Encode the checksum
    uint16_t checksum = 0;
    for (int i = 0; i < message_len; ++i) {
      checksum += driver->data.buffer[i];
    }
    *(uint16_t *)&(driver->data.buffer)[message_len] = bswap16(checksum);

    encoded = message_len + 2;
  }

  taskENTER_CRITICAL(spinlock);
  driver->data.tx_size = encoded;  // Update driver transmit size
  taskEXIT_CRITICAL(spinlock);

  return encoded;
}

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
  const size_t write_size = rdm_write(dmx_num, header, &mdb);
  dmx_send(dmx_num, write_size);
  dmx_packet_t packet = {};  // Initialize values to 0
  if (!uid_is_broadcast(header->dest_uid) ||
      (header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
       header->cc == RDM_CC_DISC_COMMAND)) {
    dmx_receive(dmx_num, &packet, 2);
  }

  // Process the response data
  if (packet.size > 0) {
    uint32_t decoded = 0;
    const rdm_header_t req = *header;
    rdm_response_type_t response_type;
    if (packet.err || !rdm_read(dmx_num, header, &mdb)) {
      response_type = RDM_RESPONSE_TYPE_INVALID;  // Data or checksum error
    } else {
      // Decode the response type
      response_type = header->response_type;
      if (response_type < RDM_RESPONSE_TYPE_ACK ||
          response_type > RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
        response_type = RDM_RESPONSE_TYPE_INVALID;  // Invalid response type
      } else if (!(req.cc == RDM_CC_DISC_COMMAND &&
                   req.pid == RDM_PID_DISC_UNIQUE_BRANCH) &&
                 (req.cc != (header->cc - 1) || req.pid != header->pid ||
                  req.tn != header->tn || req.src_uid != header->dest_uid ||
                  req.dest_uid != header->src_uid)) {
        response_type = RDM_RESPONSE_TYPE_INVALID;  // Invalid packet format
      }

      // Handle the response based on the response type
      if (response_type == RDM_RESPONSE_TYPE_ACK) {
        // Decode the MDB if there is parameter data
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
      } else if (response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
        // Received RDM_RESPONSE_TYPE_ACK_OVERFLOW
        packet.err = ESP_ERR_NOT_SUPPORTED;  // TODO: implement overflow support
      }
    }

    // Report the ACK back to the user
    if (ack != NULL) {
      ack->err = packet.err;
      ack->type = response_type;
      ack->num = decoded;
    }
  } else {
    // Wait for request to finish sending if no response is expected
    if (ack != NULL) {
      ack->err = packet.err;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->num = 0;
    }
    dmx_wait_sent(dmx_num, 2);
  }

  xSemaphoreGiveRecursive(driver->mux);
  return packet.size;
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

size_t rdm_discover_with_callback(dmx_port_t dmx_num, rdm_discovery_cb_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(cb != NULL, 0, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

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

  // Initialize the stack with the initial branch instruction
  size_t stack_size = 1;
  stack[0].lower_bound = 0;
  stack[0].upper_bound = RDM_MAX_UID;

  rdm_header_t header;   // Send and receive header information.
  rdm_disc_mute_t mute;  // Mute parameters returned from devices.
  rdm_ack_t ack;         // Request response information.
  size_t num_found = 0;

  dmx_driver_t *restrict const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Un-mute all devices
  header.dest_uid = RDM_BROADCAST_ALL_UID;
  header.sub_device = 0;
  rdm_send_disc_un_mute(dmx_num, &header, NULL, NULL);

  while (stack_size > 0) {
    // Pop a DISC_UNIQUE_BRANCH instruction parameter from the stack
    const rdm_disc_unique_branch_t *branch = &stack[--stack_size];

    size_t attempts = 0;
    if (branch->lower_bound == branch->upper_bound) {
      // Can't branch further so attempt to mute the device
      do {
        header.src_uid = 0;
        header.sub_device = 0;
        header.dest_uid = branch->lower_bound;
        rdm_send_disc_mute(dmx_num, &header, &ack, &mute);
      } while (ack.type != RDM_RESPONSE_TYPE_ACK && ++attempts < 3);
      
      // TODO: remove this workaround?
      // Attempt to fix possible error where responder is flipping its own UID
      if (ack.type != RDM_RESPONSE_TYPE_ACK) {
        header.dest_uid = bswap64(branch->lower_bound) >> 16;  // Flip UID
        rdm_send_disc_mute(dmx_num, &header, &ack, &mute);
      }

      // Call the callback function and report a device has been found
      if (ack.type == RDM_RESPONSE_TYPE_ACK) {
        cb(dmx_num, header.src_uid, num_found, &mute, context);
        ++num_found;
      }
    } else {
      // Search the current branch in the RDM address space
      do {
        header.src_uid = 0;
        rdm_send_disc_unique_branch(dmx_num, &header, branch, &ack);
      } while (ack.err == ESP_ERR_TIMEOUT && ++attempts < 3);
      if (ack.err != ESP_ERR_TIMEOUT) {
        bool devices_remaining = true;

#ifndef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
        /*
        Stop the RDM controller from branching all the way down to the
        individual address if it is not necessary. When debugging, this code 
        should not be called as it can hide bugs in the discovery algorithm. 
        Users can use the sdkconfig to enable or disable discovery debugging if 
        it is desired, but it isn't necessary unless the user makes changes to
        this function.
        */
        if (!ack.err) {
          const rdm_uid_t uid = header.src_uid;
          do {
            // Attempt to mute the device
            attempts = 0;
            do {
              header.src_uid = 0;
              header.sub_device = 0;
              header.dest_uid = uid;
              rdm_send_disc_mute(dmx_num, &header, &ack, &mute);
            } while (ack.err == ESP_ERR_TIMEOUT && ++attempts < 3);

            // Call the callback function and report a device has been found
            if (ack.type == RDM_RESPONSE_TYPE_ACK) {
              cb(dmx_num, uid, num_found, &mute, context);
              ++num_found;
            }

            // Check if there are more devices in this branch
            attempts = 0;
            do {
              header.dest_uid = RDM_BROADCAST_ALL_UID;
              rdm_send_disc_unique_branch(dmx_num, &header, branch, &ack);
            } while (ack.err == ESP_ERR_TIMEOUT && ++attempts < 3);
          } while (!ack.err && ack.type != RDM_RESPONSE_TYPE_NONE);
          devices_remaining = (ack.err && ack.err != ESP_ERR_TIMEOUT);
        }
#endif

        // Iteratively search the next two RDM address spaces
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
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  struct rdm_disc_default_ctx context = {.size = size, .uids = uids};
  size_t found = rdm_discover_with_callback(dmx_num, &rdm_disc_cb, &context);

  return found;
}