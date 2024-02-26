#include "include/discovery.h"

#include <string.h>

#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "include/utils.h"
#include "rdm/include/driver.h"
#include "rdm/include/uid.h"

static size_t rdm_rhd_discovery(dmx_port_t dmx_num,
                                const rdm_parameter_definition_t *definition,
                                const rdm_header_t *header) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    return 0;  // Cannot respond to RDM_CC_DISC_COMMAND with NACK
  }

  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // Return early if this device is muted
    uint8_t is_muted = 1;  // Don't respond if an error occurs
    dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DISC_MUTE,
                       &is_muted, sizeof(is_muted));
    if (is_muted) {
      return 0;
    }

    // Get the discovery branch parameters
    rdm_disc_unique_branch_t branch;
    if (!rdm_read_pd(dmx_num, definition->get.request.format, &branch,
                     sizeof(branch))) {
      return 0;  // Don't send NACK on error
    }

    // Guard against !(branch.lower_bound <= this_uid <= branch.upper_bound)
    const rdm_uid_t *this_uid = rdm_uid_get(dmx_num);
    if (rdm_uid_is_lt(this_uid, &branch.lower_bound) ||
        rdm_uid_is_gt(this_uid, &branch.upper_bound)) {
      return 0;  // Request not for this device
    }

    return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
  } else {
    // Set or unset the mute parameter
    const uint8_t set_mute = (header->pid == RDM_PID_DISC_MUTE);
    dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, header->pid, &set_mute,
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
      for (int i = 0; i < DMX_NUM_MAX; ++i) {
        if (dmx_driver_is_installed(i)) {
          memcpy(&mute.binding_uid, rdm_uid_get(i), sizeof(mute.binding_uid));
          break;
        }
      }
    }

    // Get the mute control field of this port
    mute.managed_proxy = 0;  // TODO: managed proxy flag
    mute.sub_device = dmx_sub_device_get_count(dmx_num) > 0 ? 1 : 0;
    mute.boot_loader = rdm_get_boot_loader(dmx_num);
    mute.proxied_device = 0;  // TODO: proxied device flag

    return rdm_write_ack(dmx_num, header, definition->get.response.format,
                         &mute, sizeof(mute));
  }
}

bool rdm_register_disc_unique_branch(dmx_port_t dmx_num, rdm_callback_t cb,
                                     void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DISC_UNIQUE_BRANCH;

  // Add the parameter as a NULL static variable
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_STATIC, NULL, 0)) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_DISC,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_discovery,
              .request.format = "uu$",
              .response.format = NULL},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = sizeof(rdm_disc_unique_branch_t),
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_register_disc_mute(dmx_port_t dmx_num, rdm_callback_t cb,
                            void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DISC_MUTE;

  // Add the parameter as a new variable or as an alias to its counterpart
  uint8_t *un_mute =
      dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DISC_UN_MUTE);
  if (un_mute == NULL) {
    uint8_t init_value = 0;
    if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                           DMX_PARAMETER_TYPE_DYNAMIC, &init_value,
                           sizeof(init_value))) {
      return false;
    }
  } else {
    if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                           DMX_PARAMETER_TYPE_STATIC, un_mute,
                           sizeof(*un_mute))) {
      return false;
    }
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_DISC,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_discovery,
              .request.format = NULL,
              .response.format = "wv"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = 0,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num, rdm_callback_t cb,
                               void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DISC_UN_MUTE;

  // Add the parameter as a new variable or as an alias to its counterpart
  uint8_t *mute =
      dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DISC_MUTE);
  if (mute == NULL) {
    uint8_t init_value = 0;
    if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                           DMX_PARAMETER_TYPE_DYNAMIC, &init_value,
                           sizeof(init_value))) {
      return false;
    }
  } else {
    if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                           DMX_PARAMETER_TYPE_STATIC, mute, sizeof(*mute))) {
      return false;
    }
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_DISC,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_discovery,
              .request.format = NULL,
              .response.format = "wv"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = 0,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}