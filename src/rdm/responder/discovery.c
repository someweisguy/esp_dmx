#include "include/discovery.h"

#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "rdm/utils/include/bus_ctl.h"
#include "rdm/utils/include/pd.h"
#include "rdm/utils/include/uid.h"

static size_t rdm_discovery_default_handler(
    dmx_port_t dmx_num, const rdm_pd_definition_t *definition,
    const rdm_header_t *header) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    return 0;  // Cannot respond to RDM_CC_DISC_COMMAND with NACK
  }

  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // Return early if this device is muted
    uint8_t is_muted = 1;  // Don't respond if an error occurs
    rdm_pd_get(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DISC_MUTE, &is_muted,
               sizeof(is_muted));
    if (is_muted) {
      return 0;
    }

    // Get the discovery branch parameters
    rdm_disc_unique_branch_t branch;
    const char *format = "uu$";
    rdm_read_pd(dmx_num, format, &branch, sizeof(branch));


    // Guard against !(branch.lower_bound <= this_uid <= branch.upper_bound)
    rdm_uid_t this_uid;
    rdm_uid_get(dmx_num, &this_uid);
    if (rdm_uid_is_lt(&this_uid, &branch.lower_bound) ||
        rdm_uid_is_gt(&this_uid, &branch.upper_bound)) {
      return 0;
    }

    return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
  } else {
    // Set or unset the mute parameter
    const uint8_t set_mute = (header->pid == RDM_PID_DISC_MUTE);
    rdm_pd_set(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DISC_MUTE, &set_mute,
               sizeof(set_mute));

    // Get the binding UID of this device
    int num_ports = 0;
    rdm_disc_mute_t mute;
    for (int i = 0; i < DMX_NUM_MAX; ++i) {
      if (dmx_driver_is_installed(i)) {
        ++num_ports;
      }
    }
    if (num_ports == 1) {
      mute.binding_uid = (rdm_uid_t){0, 0};  // Don't report a binding UID
    } else {
      rdm_uid_get_binding(&mute.binding_uid);
    }

    // Get the mute control field of this port
    mute.control_field = 0;  // TODO

    const char *format = "wv";
    return rdm_write_ack(dmx_num, header, format, &mute, sizeof(mute));
  }
}

bool rdm_register_disc_unique_branch(dmx_port_t dmx_num, rdm_callback_t cb,
                                     void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DISC_UNIQUE_BRANCH;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = 0,
      .pid_cc = RDM_CC_DISC,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_discovery_default_handler,
              .request.format = NULL,
              .response.format = NULL},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = sizeof(rdm_disc_unique_branch_t),
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_pd_set_definition(&definition);

  // RDM_PID_DISC_UNIQUE_BRANCH does not use parameter data

  return rdm_pd_set_callback(pid, cb, context);
}

bool rdm_register_disc_mute(dmx_port_t dmx_num, rdm_callback_t cb,
                            void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DISC_MUTE;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = sizeof(uint8_t),
      .pid_cc = RDM_CC_DISC,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_discovery_default_handler,
              .request.format = NULL,
              .response.format = NULL},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = 0,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_pd_set_definition(&definition);

  // Add the parameter as a new variable or as an alias to its counterpart
  const bool nvs = false;
  const rdm_pid_t alias_pid = RDM_PID_DISC_UN_MUTE;
  if (rdm_pd_get_ptr(dmx_num, RDM_SUB_DEVICE_ROOT, alias_pid) == NULL) {
    const uint8_t init_value = 0;
    if (rdm_pd_add_variable(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, &init_value,
                            sizeof(init_value)) == NULL) {
      return false;
    }
  } else {
    const size_t offset = 0;
    if (rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, alias_pid,
                         offset) == NULL) {
      return false;
    }
  }

  return rdm_pd_set_callback(pid, cb, context);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num, rdm_callback_t cb,
                               void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DISC_UN_MUTE;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = sizeof(uint8_t),
      .pid_cc = RDM_CC_DISC,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_discovery_default_handler,
              .request.format = NULL,
              .response.format = NULL},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = 0,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_pd_set_definition(&definition);

  // Add the parameter as a new variable or as an alias to its counterpart
  const bool nvs = false;
  const rdm_pid_t alias_pid = RDM_PID_DISC_MUTE;
  if (rdm_pd_get_ptr(dmx_num, RDM_SUB_DEVICE_ROOT, alias_pid) == NULL) {
    const uint8_t init_value = 0;
    if (rdm_pd_add_variable(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, &init_value,
                            sizeof(init_value)) == NULL) {
      return false;
    }
  } else {
    const size_t offset = 0;
    if (rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, alias_pid,
                         offset) == NULL) {
      return false;
    }
  }

  return rdm_pd_set_callback(pid, cb, context);
}