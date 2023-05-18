#include "requests.h"

#include "dmx/driver.h"
#include "rdm/agent.h"
#include "rdm/mdb.h"

static const char *TAG = "rdm_requests";

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

size_t rdm_send_disc_unique_branch(dmx_port_t dmx_num, rdm_header_t *header,
                                   const rdm_disc_unique_branch_t *param,
                                   rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(param != NULL, 0, "param is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->dest_uid = RDM_UID_BROADCAST_ALL;
  header->sub_device = RDM_SUB_DEVICE_ROOT;
  header->cc = RDM_CC_DISC_COMMAND;
  header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
  header->src_uid = rdm_driver_get_uid(dmx_num);
  header->port_id = dmx_num + 1;

  const rdm_encode_t encode = {
      .function = rdm_encode_uids, .params = param, .num = 2};

  return rdm_send(dmx_num, header, &encode, NULL, ack);
}

size_t rdm_send_disc_mute(dmx_port_t dmx_num, rdm_header_t *header,
                          rdm_ack_t *ack, rdm_disc_mute_t *param) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_DISC_COMMAND;
  header->pid = RDM_PID_DISC_MUTE;
  header->src_uid = rdm_driver_get_uid(dmx_num);
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
  header->src_uid = rdm_driver_get_uid(dmx_num);
  header->port_id = dmx_num + 1;

  rdm_decode_t decode = {
      .function = rdm_decode_mute,
      .params = param,
      .num = 1,
  };

  return rdm_send(dmx_num, header, NULL, &decode, ack);
}

int rdm_discover_with_callback(dmx_port_t dmx_num, rdm_discovery_cb_t cb,
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
  stack[0].upper_bound = RDM_UID_MAX;

  rdm_header_t header;   // Send and receive header information.
  rdm_disc_mute_t mute;  // Mute parameters returned from devices.
  rdm_ack_t ack;         // Request response information.
  int num_found = 0;

  dmx_driver_t *restrict const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Un-mute all devices
  header.dest_uid = RDM_UID_BROADCAST_ALL;
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
      } while (ack.type == RDM_RESPONSE_TYPE_NONE && ++attempts < 3);
      if (ack.type != RDM_RESPONSE_TYPE_NONE) {
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
        if (ack.type == RDM_RESPONSE_TYPE_ACK) {
          const rdm_uid_t uid = header.src_uid;
          do {
            // Attempt to mute the device
            attempts = 0;
            do {
              header.src_uid = 0;
              header.sub_device = 0;
              header.dest_uid = uid;
              rdm_send_disc_mute(dmx_num, &header, &ack, &mute);
            } while (ack.type == RDM_RESPONSE_TYPE_NONE && ++attempts < 3);

            // Call the callback function and report a device has been found
            if (ack.type == RDM_RESPONSE_TYPE_ACK) {
              cb(dmx_num, uid, num_found, &mute, context);
              ++num_found;
            }

            // Check if there are more devices in this branch
            attempts = 0;
            do {
              header.dest_uid = RDM_UID_BROADCAST_ALL;
              rdm_send_disc_unique_branch(dmx_num, &header, branch, &ack);
            } while (ack.type == RDM_RESPONSE_TYPE_NONE && ++attempts < 3);
          } while (ack.type == RDM_RESPONSE_TYPE_ACK);
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

int rdm_discover_devices_simple(dmx_port_t dmx_num, rdm_uid_t *uids,
                                const size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  struct rdm_disc_default_ctx context = {.size = size, .uids = uids};
  int found = rdm_discover_with_callback(dmx_num, &rdm_disc_cb, &context);

  return found;
}

size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_header_t *header,
                           rdm_ack_t *ack, rdm_device_info_t *param) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(param != NULL, 0, "param is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_GET_COMMAND;
  header->pid = RDM_PID_DEVICE_INFO;
  header->src_uid = rdm_driver_get_uid(dmx_num);
  header->port_id = dmx_num + 1;

  rdm_decode_t decode = {
      .function = rdm_decode_device_info,
      .params = param,
      .num = 1,
  };

  return rdm_send(dmx_num, header, NULL, &decode, ack);
}

size_t rdm_get_software_version_label(dmx_port_t dmx_num, rdm_header_t *header,
                                      rdm_ack_t *ack, char *param,
                                      size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(param != NULL, 0, "param is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_GET_COMMAND;
  header->pid = RDM_PID_SOFTWARE_VERSION_LABEL;
  header->src_uid = rdm_driver_get_uid(dmx_num);
  header->port_id = dmx_num + 1;

  rdm_decode_t decode = {
      .function = rdm_decode_string,
      .params = param,
      .num = size
  };

  return rdm_send(dmx_num, header, NULL, &decode, ack);
}

size_t rdm_get_identify_device(dmx_port_t dmx_num, rdm_header_t *header,
                               rdm_ack_t *ack, bool *identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(identify != NULL, 0, "identify is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_GET_COMMAND;
  header->pid = RDM_PID_IDENTIFY_DEVICE;
  header->src_uid = rdm_driver_get_uid(dmx_num);
  header->port_id = dmx_num + 1;

  rdm_decode_t decode = {
      .function = rdm_decode_8bit,
      .params = identify,
      .num = 1,
  };

  return rdm_send(dmx_num, header, NULL, &decode, ack);
}

size_t rdm_set_identify_device(dmx_port_t dmx_num, rdm_header_t *header,
                               bool identify, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_SET_COMMAND;
  header->pid = RDM_PID_IDENTIFY_DEVICE;
  header->src_uid = rdm_driver_get_uid(dmx_num);
  header->port_id = dmx_num + 1;

  rdm_encode_t encode = {
      .function = rdm_encode_8bit,
      .params = &identify,
      .num = 1,
  };

  return rdm_send(dmx_num, header, &encode, NULL, ack);
}

size_t rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_header_t *header,
                                 rdm_ack_t *ack, int *start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(start_address != NULL, 0, "start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_GET_COMMAND;
  header->pid = RDM_PID_DMX_START_ADDRESS;
  header->src_uid = rdm_driver_get_uid(dmx_num);
  header->port_id = dmx_num + 1;

  rdm_decode_t decode = {
      .function = rdm_decode_16bit,
      .params = start_address,
      .num = 1,
  };

  return rdm_send(dmx_num, header, NULL, &decode, ack);
}

size_t rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_header_t *header,
                                 int start_address, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(start_address > 0 && start_address < 513, 0,
            "start_address is invalid");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_SET_COMMAND;
  header->pid = RDM_PID_DMX_START_ADDRESS;
  header->src_uid = rdm_driver_get_uid(dmx_num);
  header->port_id = dmx_num + 1;

  rdm_encode_t encode = {
      .function = rdm_encode_16bit,
      .params = &start_address,
      .num = 1,
  };

  return rdm_send(dmx_num, header, &encode, NULL, ack);
}