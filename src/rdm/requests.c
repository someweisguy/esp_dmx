#include "requests.h"

#include "dmx/driver.h"
#include "endian.h"
#include "rdm/agent.h"
#include "rdm/utils.h"

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
  rdm_driver_get_uid(dmx_num, &header->src_uid);
  header->port_id = dmx_num + 1;

  uint8_t pd_in[sizeof(*param)];
  pd_emplace(pd_in, "uu$", param, sizeof(*param), false);

  return rdm_request(dmx_num, header, sizeof(pd_in), pd_in, NULL, NULL, ack);
}

size_t rdm_send_disc_mute(dmx_port_t dmx_num, rdm_header_t *header,
                          rdm_ack_t *ack, rdm_disc_mute_t *param) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->sub_device = RDM_SUB_DEVICE_ROOT;
  header->cc = RDM_CC_DISC_COMMAND;
  header->pid = RDM_PID_DISC_MUTE;
  rdm_driver_get_uid(dmx_num, &header->src_uid);
  header->port_id = dmx_num + 1;

  uint8_t pdl_out = sizeof(*param);
  uint8_t pd_out[sizeof(*param)];
  size_t ret = rdm_request(dmx_num, header, 0, NULL, &pdl_out, pd_out, ack);
  pd_emplace(param, "wv&", pd_out, sizeof(*param), true);

  return ret;
}

size_t rdm_send_disc_un_mute(dmx_port_t dmx_num, rdm_header_t *header,
                             rdm_ack_t *ack, rdm_disc_mute_t *param) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->sub_device = RDM_SUB_DEVICE_ROOT;
  header->cc = RDM_CC_DISC_COMMAND;
  header->pid = RDM_PID_DISC_UN_MUTE;
  rdm_driver_get_uid(dmx_num, &header->src_uid);
  header->port_id = dmx_num + 1;

  uint8_t pdl_out = sizeof(*param);
  uint8_t pd_out[sizeof(*param)];
  size_t ret = rdm_request(dmx_num, header, 0, NULL, &pdl_out, pd_out, ack);
  pd_emplace(param, "wv&", pd_out, sizeof(*param), true);

  return ret;
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
  stack[0].lower_bound = RDM_UID_NULL;
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
    if (uid_is_eq(&branch->lower_bound, &branch->upper_bound)) {
      // Can't branch further so attempt to mute the device
      do {
        header.src_uid = RDM_UID_NULL;
        header.dest_uid = branch->lower_bound;
        rdm_send_disc_mute(dmx_num, &header, &ack, &mute);
      } while (ack.type != RDM_RESPONSE_TYPE_ACK && ++attempts < 3);

      // TODO: remove this workaround?
      // Attempt to fix possible error where responder is flipping its own UID
      if (ack.type != RDM_RESPONSE_TYPE_ACK) {
        uint64_t uid = bswap64(((uint64_t)header.dest_uid.man_id << 32) |
                               header.dest_uid.dev_id) >>
                       16;
        header.dest_uid.man_id = uid >> 32;
        header.dest_uid.dev_id = uid;
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
        header.src_uid = RDM_UID_NULL;
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
              header.src_uid = RDM_UID_NULL;
              header.sub_device = RDM_SUB_DEVICE_ROOT;
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
          const rdm_uid_t first_lbound = branch->lower_bound;
          uint64_t mid = ((((uint64_t)branch->lower_bound.man_id << 32) |
                           branch->lower_bound.dev_id) +
                          (((uint64_t)branch->upper_bound.man_id << 32) |
                           branch->upper_bound.dev_id)) /
                         2;

          // Add the upper branch so that it gets handled second
          stack[stack_size].lower_bound.man_id = (mid + 1) >> 32;
          stack[stack_size].lower_bound.dev_id = mid + 1;
          // Reuse the upper_bound that is currently on the stack
          ++stack_size;

          // Add the lower branch so it gets handled first
          stack[stack_size].lower_bound = first_lbound;
          stack[stack_size].upper_bound.man_id = mid >> 32;
          stack[stack_size].upper_bound.dev_id = mid;

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
  rdm_driver_get_uid(dmx_num, &header->src_uid);
  header->port_id = dmx_num + 1;

  uint8_t pdl_out = sizeof(*param);
  uint8_t pd_out[sizeof(*param)];
  size_t ret = rdm_request(dmx_num, header, 0, NULL, &pdl_out, pd_out, ack);
  pd_emplace(param, "#0100hwwdwbbwwb$", pd_out, sizeof(*param), true);

  return ret;
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
  rdm_driver_get_uid(dmx_num, &header->src_uid);
  header->port_id = dmx_num + 1;

  uint8_t pdl_out;
  uint8_t pd_out[33];
  size_t ret = rdm_request(dmx_num, header, 0, NULL, &pdl_out, pd_out, ack);
  if (pdl_out > size) {
    pdl_out = size;
  }
  pd_emplace(param, "a", pd_out, pdl_out, true);

  return ret;
}

size_t rdm_get_identify_device(dmx_port_t dmx_num, rdm_header_t *header,
                               rdm_ack_t *ack, bool *identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(identify != NULL, 0, "identify is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_GET_COMMAND;
  header->pid = RDM_PID_IDENTIFY_DEVICE;
  rdm_driver_get_uid(dmx_num, &header->src_uid);
  header->port_id = dmx_num + 1;

  uint8_t pdl_out = sizeof(*identify);
  uint8_t pd_out[sizeof(*identify)];
  size_t ret = rdm_request(dmx_num, header, 0, NULL, &pdl_out, pd_out, ack);
  pd_emplace(identify, "b$", pd_out, sizeof(*identify), true);

  return ret;
}

size_t rdm_set_identify_device(dmx_port_t dmx_num, rdm_header_t *header,
                               bool identify, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_SET_COMMAND;
  header->pid = RDM_PID_IDENTIFY_DEVICE;
  rdm_driver_get_uid(dmx_num, &header->src_uid);
  header->port_id = dmx_num + 1;

  uint8_t pd_in[sizeof(identify)];
  pd_emplace(pd_in, "b$", &identify, sizeof(identify), false);

  return rdm_request(dmx_num, header, sizeof(pd_in), pd_in, NULL, NULL, ack);
}

size_t rdm_get_dmx_start_address(dmx_port_t dmx_num, rdm_header_t *header,
                                 rdm_ack_t *ack, uint16_t *dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_start_address != NULL, 0, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint8_t pdl_out = sizeof(*dmx_start_address);
  uint8_t pd_out[sizeof(*dmx_start_address)];
  size_t ret = rdm_request(dmx_num, header, 0, NULL, &pdl_out, pd_out, ack);
  pd_emplace(dmx_start_address, "w$", pd_out, sizeof(*dmx_start_address), true);

  return ret;
}

size_t rdm_set_dmx_start_address(dmx_port_t dmx_num, rdm_header_t *header,
                                 uint16_t dmx_start_address, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_start_address < 513, 0, "dmx_start_address is invalid");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  header->cc = RDM_CC_SET_COMMAND;
  header->pid = RDM_PID_DMX_START_ADDRESS;
  rdm_driver_get_uid(dmx_num, &header->src_uid);
  header->port_id = dmx_num + 1;

  uint8_t pd_in[sizeof(dmx_start_address)];
  pd_emplace(pd_in, "w$", &dmx_start_address, sizeof(dmx_start_address), false);

  return rdm_request(dmx_num, header, sizeof(pd_in), pd_in, NULL, NULL, ack);
}
