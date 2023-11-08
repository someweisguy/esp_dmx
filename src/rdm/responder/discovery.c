#include "discovery.h"

#include "dmx/struct.h"
#include "dmx/driver.h"
#include "rdm/utils/uid.h"
#include "rdm/utils/bus_ctl.h"

static int rdm_rh_discovery_default(dmx_port_t dmx_num, rdm_header_t *header,
                                    void *pd, uint8_t *pdl_out,
                                    const char *format) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    // Cannot respond to RDM_CC_DISC_COMMAND with NACK
    return RDM_RESPONSE_TYPE_NONE;
  }

  int response_type;
  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // Ignore this message if discovery is muted
    const uint8_t *is_muted =
        rdm_pd_get(dmx_num, RDM_PID_DISC_MUTE, RDM_SUB_DEVICE_ROOT);
    if (is_muted == NULL) {
      rdm_set_boot_loader(dmx_num);
      return RDM_RESPONSE_TYPE_NONE;
    } else if (*is_muted) {
      return RDM_RESPONSE_TYPE_NONE;
    }

    // Get the discovery branch parameters
    rdm_disc_unique_branch_t branch;
    rdm_emplace(&branch, "uu$", pd, sizeof(branch), true);

    // Respond if lower_bound <= my_uid <= upper_bound
    rdm_uid_t my_uid;
    rdm_uid_get(dmx_num, &my_uid);
    if (rdm_uid_is_ge(&my_uid, &branch.lower_bound) &&
        rdm_uid_is_le(&my_uid, &branch.upper_bound)) {
      *pdl_out = rdm_emplace(pd, "u$", &my_uid, sizeof(my_uid), false);
      response_type = RDM_RESPONSE_TYPE_ACK;
    } else {
      response_type = RDM_RESPONSE_TYPE_NONE;
    }
  } else {
    // Mute or un-mute the discovery responses
    const uint8_t mute_command = (header->pid == RDM_PID_DISC_MUTE);
    rdm_pd_set(dmx_num, RDM_PID_DISC_MUTE, RDM_SUB_DEVICE_ROOT, &mute_command,
               sizeof(uint8_t));

    // Get the binding UID of this device
    int num_ports = 0;
    for (int i = 0; i < DMX_NUM_MAX; ++i) {
      if (dmx_driver_is_installed(i)) {
        ++num_ports;
      }
    }
    rdm_uid_t binding_uid;
    if (num_ports == 1) {
      // Only report binding UID if there are multiple ports
      binding_uid = (rdm_uid_t){0, 0};
    } else {
      rdm_uid_get_binding(&binding_uid);
    }

    // Respond with this device's mute parameters
    const rdm_disc_mute_t mute = {
        .control_field = 0,  // TODO: get the control_field of the device
        .binding_uid = binding_uid,
    };

    *pdl_out = rdm_emplace(pd, "wv$", &mute, sizeof(mute), false);
    response_type = RDM_RESPONSE_TYPE_ACK;
  }

  return response_type;
}

bool rdm_register_disc_unique_branch(dmx_port_t dmx_num, rdm_callback_t cb,
                                     void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DISC_UNIQUE_BRANCH;
  const rdm_pd_schema_t schema = {
      .data_type = RDM_DS_NOT_DEFINED,
      .cc = RDM_CC_DISC,
      .size = 0,
      .format = "",
      .nvs = false,
      .response_handler = rdm_rh_discovery_default,
  };

  // Register the parameter
  rdm_pd_add_deterministic(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &schema);
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_register_disc_mute(dmx_port_t dmx_num, rdm_callback_t cb,
                            void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DISC_MUTE;
  const rdm_pd_schema_t schema = {
      .data_type = RDM_DS_NOT_DEFINED,
      .cc = RDM_CC_DISC,
      .size = sizeof(uint8_t),
      .format = "",
      .nvs = false,
      .response_handler = rdm_rh_discovery_default,
  };

  // Register the parameter as an alias if RDM_PID_DISC_UN_MUTE exists
  if (rdm_pd_get(dmx_num, RDM_PID_DISC_UN_MUTE, RDM_SUB_DEVICE_ROOT)) {
    const size_t offset = 0;  // Mute and un-mute are shared parameters
    rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &schema,
                     RDM_PID_DISC_UN_MUTE, offset);
  } else {
    const uint8_t init_value = 0;  // Initial value is un-muted
    rdm_pd_add_new(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &schema, &init_value);
  }
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num, rdm_callback_t cb,
                               void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DISC_UN_MUTE;
  const rdm_pd_schema_t schema = {
      .data_type = RDM_DS_NOT_DEFINED,
      .cc = RDM_CC_DISC,
      .size = sizeof(uint8_t),
      .format = "",
      .nvs = false,
      .response_handler = rdm_rh_discovery_default,
  };

  // Register the parameter as an alias if RDM_PID_DISC_MUTE exists
  if (rdm_pd_get(dmx_num, RDM_PID_DISC_MUTE, RDM_SUB_DEVICE_ROOT)) {
    const size_t offset = 0;  // Mute and un-mute are shared parameters
    rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &schema,
                     RDM_PID_DISC_MUTE, offset);
  } else {
    const uint8_t init_value = 0;  // Initial value is un-muted
    rdm_pd_add_new(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &schema, &init_value);
  }
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}