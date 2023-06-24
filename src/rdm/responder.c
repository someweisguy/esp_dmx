#include "responder.h"

#include "dmx/driver.h"
#include "endian.h"
#include "rdm/utils.h"

static const char *TAG = "rdm_responder";  // The log tagline for the file.

static rdm_response_type_t rdm_default_discovery_cb(
    dmx_port_t dmx_num, const rdm_header_t *header, void *pd, uint8_t *pdl_out,
    void *param, size_t param_len, void *context) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    // Cannot respond to RDM_CC_DISC_COMMAND with NACK
    return RDM_RESPONSE_TYPE_NONE;
  }

  rdm_response_type_t response_type;
  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // Ignore this message if discovery is muted
    if (rdm_disc_mute_get(dmx_num)) {
      return RDM_RESPONSE_TYPE_NONE;
    }

    // Get the discovery branch parameters
    rdm_disc_unique_branch_t branch;
    pd_emplace(&branch, "uu$", pd, sizeof(branch), true);

    // Respond if lower_bound <= my_uid <= upper_bound
    rdm_uid_t my_uid;
    uid_get(dmx_num, &my_uid);
    if (uid_is_ge(&my_uid, &branch.lower_bound) &&
        uid_is_le(&my_uid, &branch.upper_bound)) {
      *pdl_out = pd_emplace(pd, "u$", &my_uid, sizeof(my_uid), false);
      response_type = RDM_RESPONSE_TYPE_ACK;
    } else {
      response_type = RDM_RESPONSE_TYPE_NONE;
    }
  } else {
    // Mute or un-mute the discovery responses
    rdm_disc_mute_set(dmx_num, (header->pid == RDM_PID_DISC_MUTE));

    // Get the binding UID of this device
    int num_ports = 0;
    rdm_uid_t binding_uid;
    for (int i = 0; i < DMX_NUM_MAX; ++i) {
      if (dmx_driver_is_installed(i)) {
        if (num_ports == 0) {
          uid_get(i, &binding_uid);
        }
        ++num_ports;
      }
    }
    if (num_ports == 1) {
      // Only report binding UID if there are multiple ports
      binding_uid = (rdm_uid_t){0, 0};
    }

    // Respond with this device's mute parameters
    const rdm_disc_mute_t mute = {
        .control_field = 0,  // TODO: get the control_field of the device
        .binding_uid = binding_uid,
    };
    *pdl_out = pd_emplace(pd, "wv$", &mute, sizeof(mute), false);
    response_type = RDM_RESPONSE_TYPE_ACK;
  }

  return response_type;
}

bool rdm_register_disc_unique_branch(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UNIQUE_BRANCH,
                                      .pdl_size = sizeof(rdm_uid_t),
                                      .data_type = RDM_DS_BIT_FIELD,
                                      .cc = RDM_CC_DISC,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Discovery Unique Branch"};

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_default_discovery_cb, NULL, NULL);
}

bool rdm_register_disc_mute(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_DISC_MUTE,
                                      .pdl_size = sizeof(rdm_disc_mute_t),
                                      .data_type = RDM_DS_BIT_FIELD,
                                      .cc = RDM_CC_DISC,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Discovery Mute"};

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_default_discovery_cb, NULL, NULL);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UN_MUTE,
                                      .pdl_size = sizeof(rdm_disc_mute_t),
                                      .data_type = RDM_DS_BIT_FIELD,
                                      .cc = RDM_CC_DISC,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Discovery Un-Mute"};

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_default_discovery_cb, NULL, NULL);
}

static rdm_response_type_t rdm_simple_response_cb(dmx_port_t dmx_num,
                                                  const rdm_header_t *header,
                                                  void *pd, uint8_t *pdl_out,
                                                  void *param, size_t param_len,
                                                  void *context) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if (header->cc == RDM_CC_GET_COMMAND) {
    *pdl_out = pd_emplace(pd, context, param, param_len, false);
  } else {
    pd_emplace(param, context, pd, header->pdl, true);
  }

  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(device_info != NULL, false, "device_info is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_DEVICE_INFO,
                                      .pdl_size = sizeof(rdm_device_info_t),
                                      .data_type = RDM_DS_BIT_FIELD,
                                      .cc = RDM_CC_GET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Device Info"};
  const char *param_str = "#0100hwwdwbbwwb$";

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_simple_response_cb, device_info,
                               (void *)param_str);
}

bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         const char *software_version_label) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(software_version_label != NULL, false,
            "software_version_label is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Get the string length and clamp it to 32 chars
  unsigned int num = strlen(software_version_label);
  if (num > 32) {
    ESP_LOGW(TAG, "Software version label will be truncated to 32 characters.");
    num = 32;
  }

  const rdm_pid_description_t desc = {.pid = RDM_PID_SOFTWARE_VERSION_LABEL,
                                      .pdl_size = num,
                                      .data_type = RDM_DS_ASCII,
                                      .cc = RDM_CC_GET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Software Version Label"};
  const char *param_str = "a";

  return rdm_register_response(
      dmx_num, RDM_SUB_DEVICE_ROOT, &desc, rdm_simple_response_cb,
      (void *)software_version_label, (void *)param_str);
}

static rdm_response_type_t rdm_identify_response_cb(
    dmx_port_t dmx_num, const rdm_header_t *header, void *pd, uint8_t *pdl_out,
    void *param, size_t param_len, void *context) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  const char *param_str = "b$";
  if (header->cc == RDM_CC_GET_COMMAND) {
    const uint8_t identify = rdm_identify_get();
    *pdl_out = pd_emplace(pd, param_str, &identify, sizeof(identify), false);
  } else {
    // Call the user-specified callback when the state changes
    const uint8_t old_value = rdm_identify_get();
    const uint8_t new_value = *(uint8_t *)pd;
    if (old_value != new_value) {
      rdm_identify_set(new_value);
      rdm_identify_cb_t identify_cb = param;
      identify_cb(dmx_num, new_value, context);
    }
  }

  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_identify_cb_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(cb != NULL, false, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_IDENTIFY_DEVICE,
                                      .pdl_size = sizeof(uint8_t),
                                      .data_type = RDM_DS_UNSIGNED_BYTE,
                                      .cc = RDM_CC_GET_SET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 1,
                                      .default_value = 0,
                                      .description = "Identify Device"};

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_identify_response_cb, cb, context);
}

bool rdm_register_dmx_start_address(dmx_port_t dmx_num,
                                    uint16_t *dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_start_address != NULL, false, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_DMX_START_ADDRESS,
                                      .pdl_size = sizeof(uint16_t),
                                      .data_type = RDM_DS_UNSIGNED_WORD,
                                      .cc = RDM_CC_GET_SET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 1,
                                      .max_value = 512,
                                      .default_value = 1,
                                      .description = "DMX Start Address"};
  const char *param_str = "w$";

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_simple_response_cb, dmx_start_address,
                               (void *)param_str);
}