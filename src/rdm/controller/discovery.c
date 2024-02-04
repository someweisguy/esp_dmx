#include "include/discovery.h"

#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/controller/include/utils.h"
#include "rdm/include/driver.h"
#include "rdm/include/uid.h"

bool rdm_send_disc_unique_branch(dmx_port_t dmx_num,
                                 const rdm_disc_unique_branch_t *branch,
                                 rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(branch != NULL, 0, "branch is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_request_t request = {.dest_uid = &RDM_UID_BROADCAST_ALL,
                                 .sub_device = RDM_SUB_DEVICE_ROOT,
                                 .cc = RDM_CC_DISC_COMMAND,
                                 .pid = RDM_PID_DISC_UNIQUE_BRANCH,
                                 .format = "uu$",
                                 .pd = branch,
                                 .pdl = sizeof(*branch)};

  return rdm_send_request(dmx_num, &request, NULL, NULL, 0, ack);
}

bool rdm_send_disc_mute(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                        rdm_disc_mute_t *mute, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dest_uid != NULL, 0, "dest_uid is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_request_t request = {.dest_uid = dest_uid,
                                 .sub_device = RDM_SUB_DEVICE_ROOT,
                                 .cc = RDM_CC_DISC_COMMAND,
                                 .pid = RDM_PID_DISC_MUTE};

  const char *format = "wv";
  return rdm_send_request(dmx_num, &request, format, &mute, sizeof(*mute), ack);
}

bool rdm_send_disc_un_mute(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                           rdm_disc_mute_t *mute, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dest_uid != NULL, 0, "dest_uid is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_request_t request = {.dest_uid = dest_uid,
                                 .sub_device = RDM_SUB_DEVICE_ROOT,
                                 .cc = RDM_CC_DISC_COMMAND,
                                 .pid = RDM_PID_DISC_UN_MUTE};

  const char *format = "wv";
  return rdm_send_request(dmx_num, &request, format, &mute, sizeof(*mute), ack);
}

int rdm_discover_with_callback(dmx_port_t dmx_num, rdm_disc_cb_t cb,
                               void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(cb != NULL, 0, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Allocate the instruction stack. The max binary tree depth is 49.
#ifdef CONFIG_RDM_STATIC_DISCOVERY_INSTRUCTIONS
  static rdm_disc_unique_branch_t stack[49]; 
#else
  rdm_disc_unique_branch_t *stack;
  stack = malloc(sizeof(rdm_disc_unique_branch_t) * 49);
  DMX_CHECK(stack != NULL, 0, "discovery malloc error");
#endif

  // Initialize the stack with the initial branch instruction
  int stack_size = 1;
  stack[0].lower_bound = (rdm_uid_t){0, 0};
  stack[0].upper_bound = RDM_UID_MAX;

  rdm_uid_t dest_uid;
  rdm_disc_mute_t mute;  // Mute parameters returned from devices.
  rdm_ack_t ack;         // Request response information.
  int num_found = 0;

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  xSemaphoreTakeRecursive(driver->mux, 0);

  // Un-mute all devices
  dest_uid = RDM_UID_BROADCAST_ALL;
  rdm_send_disc_un_mute(dmx_num, &dest_uid, NULL, NULL);

  while (stack_size > 0) {
    // Pop a DISC_UNIQUE_BRANCH instruction parameter from the stack
    const rdm_disc_unique_branch_t *branch = &stack[--stack_size];

    size_t attempts = 0;
    if (rdm_uid_is_eq(&branch->lower_bound, &branch->upper_bound)) {
      // Can't branch further so attempt to mute the device
      dest_uid = branch->lower_bound;
      do {
        rdm_send_disc_mute(dmx_num, &dest_uid, &mute, &ack);
      } while (ack.type != RDM_RESPONSE_TYPE_ACK && ++attempts < 3);

      // Call the callback function and report a device has been found
      if (ack.type == RDM_RESPONSE_TYPE_ACK) {
        xSemaphoreGiveRecursive(driver->mux);
        cb(dmx_num, ack.src_uid, num_found, &mute, context);
        xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
        ++num_found;
      }
    } else {
      // Search the current branch in the RDM address space
      do {
        rdm_send_disc_unique_branch(dmx_num, branch, &ack);
      } while (ack.type == RDM_RESPONSE_TYPE_NONE && ++attempts < 3);
      if (ack.type != RDM_RESPONSE_TYPE_NONE) {
        bool devices_remaining = true;

#if false  // FIXME: This should temporarily allow discovery to work.
// #ifndef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
        /*
        Stop the RDM controller from branching all the way down to the
        individual address if it is not necessary. When debugging, this code
        should not be called as it can hide bugs in the discovery algorithm.
        Users can use the sdkconfig to enable or disable discovery debugging if
        it is desired, but it isn't necessary unless the user makes changes to
        this function.
        */
        if (ack.type == RDM_RESPONSE_TYPE_ACK) {
          do {
            // Attempt to mute the device
            attempts = 0;
            dest_uid = ack.src_uid;
            do {
              rdm_send_disc_mute(dmx_num, &dest_uid, &mute, &ack);
            } while (ack.type == RDM_RESPONSE_TYPE_NONE && ++attempts < 3);

            // Call the callback function and report a device has been found
            if (ack.type == RDM_RESPONSE_TYPE_ACK) {
              cb(dmx_num, ack.src_uid, num_found, &mute, context);
              ++num_found;
            }

            // Check if there are more devices in this branch
            attempts = 0;
            do {
              rdm_send_disc_unique_branch(dmx_num, branch, &ack);
            } while (ack.type == RDM_RESPONSE_TYPE_NONE && ++attempts < 3);
          } while (ack.type == RDM_RESPONSE_TYPE_ACK);
          devices_remaining = (ack.err && ack.err != DMX_ERR_TIMEOUT);
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

#ifndef CONFIG_RDM_STATIC_DISCOVERY_INSTRUCTIONS
  free(stack);
#endif

  return num_found;
}

struct rdm_disc_default_ctx {
  unsigned int num;
  rdm_uid_t *uids;
};

static void rdm_disc_cb(dmx_port_t dmx_num, rdm_uid_t uid, int num_found,
                        const rdm_disc_mute_t *mute, void *context) {
  struct rdm_disc_default_ctx *c = (struct rdm_disc_default_ctx *)context;
  if (num_found < c->num && c->uids != NULL) {
    c->uids[num_found] = uid;
  }
}

int rdm_discover_devices_simple(dmx_port_t dmx_num, rdm_uid_t *uids,
                                unsigned int num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  struct rdm_disc_default_ctx context = {.num = num, .uids = uids};
  int found = rdm_discover_with_callback(dmx_num, &rdm_disc_cb, &context);

  return found;
}