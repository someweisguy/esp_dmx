#include "responder.h"

#include "dmx/config.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"
#include "endian.h"
#include "esp_dmx.h"
#include "rdm_utils.h"

static int rdm_default_discovery_cb(dmx_port_t dmx_num, rdm_header_t *header,
                                    void *pd, uint8_t *pdl_out,
                                    const char *param_str) {
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
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      dmx_driver[dmx_num]->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      return RDM_RESPONSE_TYPE_NONE;
    } else if (*is_muted) {
      return RDM_RESPONSE_TYPE_NONE;
    }

    // Get the discovery branch parameters
    rdm_disc_unique_branch_t branch;
    rdm_pd_emplace(&branch, "uu$", pd, sizeof(branch), true);

    // Respond if lower_bound <= my_uid <= upper_bound
    rdm_uid_t my_uid;
    rdm_uid_get(dmx_num, &my_uid);
    if (rdm_uid_is_ge(&my_uid, &branch.lower_bound) &&
        rdm_uid_is_le(&my_uid, &branch.upper_bound)) {
      *pdl_out = rdm_pd_emplace(pd, "u$", &my_uid, sizeof(my_uid), false);
      response_type = RDM_RESPONSE_TYPE_ACK;
    } else {
      response_type = RDM_RESPONSE_TYPE_NONE;
    }
  } else {
    // Mute or un-mute the discovery responses
    uint8_t *is_muted = rdm_pd_get(dmx_num, RDM_PID_DISC_MUTE, 0);
    *is_muted = (header->pid == RDM_PID_DISC_MUTE);

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

    *pdl_out = rdm_pd_emplace(pd, "wv$", &mute, sizeof(mute), false);
    response_type = RDM_RESPONSE_TYPE_ACK;
  }

  return response_type;
}

bool rdm_register_disc_unique_branch(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                     void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UNIQUE_BRANCH,
                                      .pdl_size = 0,
                                      .data_type = RDM_DS_NOT_DEFINED,
                                      .cc = RDM_CC_DISC,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Discovery Unique Branch"};

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_default_discovery_cb, NULL, cb, context,
                                false);
}

bool rdm_register_disc_mute(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                            void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  uint8_t *param =
      rdm_pd_get(dmx_num, RDM_PID_DISC_MUTE, RDM_SUB_DEVICE_ROOT);
  if (param == NULL) {
    param =
        rdm_pd_get(dmx_num, RDM_PID_DISC_UN_MUTE, RDM_SUB_DEVICE_ROOT);
    if (param == NULL) {
      param = rdm_pd_alloc(dmx_num, sizeof(*param));
      if (param == NULL) {
        return false;
      }
      *param = 0;
    }
  }

  const rdm_pid_description_t desc = {.pid = RDM_PID_DISC_MUTE,
                                      .pdl_size = sizeof(*param),
                                      .data_type = RDM_DS_UNSIGNED_BYTE,
                                      .cc = RDM_CC_DISC,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Discovery Mute"};

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_default_discovery_cb, param, cb, context,
                                false);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                               void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  uint8_t *param =
      rdm_pd_get(dmx_num, RDM_PID_DISC_UN_MUTE, RDM_SUB_DEVICE_ROOT);
  if (param == NULL) {
    param = rdm_pd_get(dmx_num, RDM_PID_DISC_MUTE, RDM_SUB_DEVICE_ROOT);
    if (param == NULL) {
      param = rdm_pd_alloc(dmx_num, sizeof(*param));
      if (param == NULL) {
        return false;
      }
      *param = 0;
    }
  }

  const rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UN_MUTE,
                                      .pdl_size = sizeof(*param),
                                      .data_type = RDM_DS_UNSIGNED_BYTE,
                                      .cc = RDM_CC_DISC,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Discovery Un-Mute"};

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_default_discovery_cb, param, cb, context,
                                false);
}

static int rdm_simple_response_cb(dmx_port_t dmx_num, rdm_header_t *header,
                                  void *pd, uint8_t *pdl_out,
                                  const char *param_str) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  void *param = rdm_pd_get(dmx_num, header->pid, header->sub_device);
  if (header->cc == RDM_CC_GET_COMMAND) {
    *pdl_out = rdm_pd_emplace(pd, param_str, param, 231, false);
  } else {
    rdm_pd_emplace(param, param_str, pd, header->pdl, true);
  }

  return RDM_RESPONSE_TYPE_ACK;
}

static int rdm_personality_description_response_cb(dmx_port_t dmx_num,
                                                   rdm_header_t *header,
                                                   void *pd, uint8_t *pdl_out,
                                                   const char *param_str) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  rdm_device_info_t *di =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, header->sub_device);
  if(di == NULL)
  {
    //none of the error codes really fit, thus we just go with unknown pid
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_UNKNOWN_PID);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if(header->cc == RDM_CC_SET_COMMAND)
  {      
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_WRITE_PROTECT);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if(header->pdl != 1)
  {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_FORMAT_ERROR);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  const uint8_t requestedPersonality = *((uint8_t*)pd);
  const uint16_t footprint = (uint16_t)dmx_get_footprint(dmx_num, requestedPersonality);
  const char* personalityDesc = dmx_get_personality_description(dmx_num, requestedPersonality);
  
  if(personalityDesc == NULL)
  {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  memcpy(pd, &requestedPersonality, 1);
  pd++;
  rdm_pd_emplace_word(pd, footprint);
  pd += 2;
  const size_t emplacedBytes = rdm_pd_emplace(pd, "a$", personalityDesc, 32, false);
  *pdl_out = 3 + emplacedBytes;

  return RDM_RESPONSE_TYPE_ACK;
}

static int rdm_personality_response_cb(dmx_port_t dmx_num, rdm_header_t *header,
                                       void *pd, uint8_t *pdl_out,
                                       const char *param_str) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  rdm_device_info_t *di =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  if(di == NULL)
  {
    //none of the error codes really fit, thus we just go with unknown pid
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_UNKNOWN_PID); 
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if(header->cc == RDM_CC_GET_COMMAND)
  {
    const uint8_t data[] = {di->current_personality, di->personality_count};
    memcpy(pd, data, 2);
    *pdl_out = 2;
    return RDM_RESPONSE_TYPE_ACK;
  }
  else if(header->cc == RDM_CC_SET_COMMAND)
  {
    if(header->pdl != 1)
    {
      *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_FORMAT_ERROR);
      return RDM_RESPONSE_TYPE_NACK_REASON;
    }

    const uint8_t requestedPersonality = *((uint8_t*)pd);
    if(requestedPersonality >= di->personality_count)
    {
      *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
      return RDM_RESPONSE_TYPE_NACK_REASON;
    }

    dmx_set_current_personality(dmx_num, requestedPersonality);
    //note: we do not need to set it in nvs because that is done in hal.c:dmx_receive()
    return RDM_RESPONSE_TYPE_ACK;
  }
  else
  {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }
}

static int rdm_parameter_description_response_cb(dmx_port_t dmx_num,
                                                 rdm_header_t *header, void *pd,
                                                 uint8_t *pdl_out,
                                                 const char *param_str) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT)
  {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  uint16_t requestedPid;
  memcpy(&requestedPid, pd, sizeof(uint16_t)); // need to memcpy to avoid undefined behavior (strict aliasing rule)
  requestedPid = bswap16(requestedPid);

  // 0x8000 to 0xFFDF is the allowed range for manufacturer specific pids
  if (requestedPid < RDM_PID_MANUFACTURER_SPECIFIC_BEGIN || requestedPid > RDM_PID_MANUFACTURER_SPECIFIC_END)
  {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Iterate the callback list to see if a callback with this PID exists
  for (int i = 0; i < driver->num_rdm_cbs; ++i)
  {
    if (driver->rdm_cbs[i].desc.pid == requestedPid)
    {
      //The pdl can be in range x014-0x34 depending on how long the parameter description string is.
      //There is no harm in always sending the full string, so we just do that.
      *pdl_out = rdm_pd_emplace(pd, param_str, &driver->rdm_cbs[i].desc, 0x34, false);
      return RDM_RESPONSE_TYPE_ACK;
    }
  }

  // no pid found
  *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
  return RDM_RESPONSE_TYPE_NACK_REASON;
}

bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info,
                              rdm_responder_cb_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_device_info_t *param =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  if (param == NULL) {
    DMX_CHECK(device_info != NULL, false, "device_info is null");
    DMX_CHECK((device_info->dmx_start_address < DMX_PACKET_SIZE_MAX ||
               device_info->dmx_start_address == DMX_START_ADDRESS_NONE),
              false, "dmx_start_address error");
    DMX_CHECK((device_info->footprint == 0 &&
               device_info->dmx_start_address == DMX_START_ADDRESS_NONE) ||
                  (device_info->footprint > 0 &&
                   device_info->footprint < DMX_PACKET_SIZE_MAX),
              false, "footprint error");
    DMX_CHECK((device_info->personality_count == 0 &&
               device_info->dmx_start_address == DMX_START_ADDRESS_NONE) ||
                  (device_info->personality_count > 0 &&
                   device_info->personality_count < DMX_PERSONALITY_COUNT_MAX),
              false, "personality_count error");
    DMX_CHECK(
        device_info->current_personality <= device_info->personality_count,
        false, "current_personality error");

    // Load the DMX start address from NVS
    if (device_info->dmx_start_address == 0) {
      size_t size = sizeof(device_info->dmx_start_address);
      if (!dmx_nvs_get(dmx_num, RDM_PID_DMX_START_ADDRESS, RDM_DS_UNSIGNED_WORD,
                       &device_info->dmx_start_address, &size)) {
        device_info->dmx_start_address = 1;
      }
    }

    // Load the current DMX personality from NVS
    if (device_info->current_personality == 0 &&
        device_info->dmx_start_address != DMX_START_ADDRESS_NONE) {
      rdm_dmx_personality_t personality;
      size_t size = sizeof(personality);
      if (dmx_nvs_get(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_DS_BIT_FIELD,
                      &personality, &size) ||
          personality.personality_count != device_info->personality_count) {
        device_info->current_personality = 1;
      } else {
        device_info->current_personality = personality.current_personality;
      }
      device_info->footprint =
          dmx_get_footprint(dmx_num, device_info->current_personality);
    }

    param = rdm_pd_alloc(dmx_num, sizeof(*param));
    if (param == NULL) {
      return false;
    }
    memcpy(param, device_info, sizeof(*param));
  }

  const rdm_pid_description_t desc = {.pid = RDM_PID_DEVICE_INFO,
                                      .pdl_size = sizeof(*param),
                                      .data_type = RDM_DS_BIT_FIELD,
                                      .cc = RDM_CC_GET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Device Info"};
  const char *param_str = "#0100hwwdwbbwwb$";

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context,
                                false);
}

bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         const char *software_version_label,
                                         rdm_responder_cb_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_SOFTWARE_VERSION_LABEL,
                                      .pdl_size = 32,
                                      .data_type = RDM_DS_ASCII,
                                      .cc = RDM_CC_GET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Software Version Label"};
  const char *param_str = "a$";

  char *param = rdm_pd_get(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL,
                                  RDM_SUB_DEVICE_ROOT);
  if (param == NULL) {
    DMX_CHECK(software_version_label != NULL, false,
              "software_version_label is null");
    DMX_CHECK(strnlen(software_version_label, 33) < 33, false,
              "software_version_label error");
    param = rdm_pd_alloc(dmx_num, 32);
    if (param == NULL) {
      return false;
    }
    strncpy(param, software_version_label, 32);
  }

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context,
                                false);
}

bool rdm_register_device_label(dmx_port_t dmx_num,
                               const char *device_label,
                               rdm_responder_cb_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_DEVICE_LABEL,
                                      .pdl_size = 32,
                                      .data_type = RDM_DS_ASCII,
                                      .cc = RDM_CC_GET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Device Label"};
  const char *param_str = "a$";

  char *param =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_LABEL, RDM_SUB_DEVICE_ROOT);
  if (param == NULL) {
    DMX_CHECK(device_label != NULL, false,
              "device_label is null");
    DMX_CHECK(strnlen(device_label, 33) < 33, false,
              "device_label error");
    param = rdm_pd_alloc(dmx_num, 32);
    if (param == NULL) {
      return false;
    }
    strncpy(param, device_label, 32);
  }

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context,
                                true);
}

bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(cb != NULL, false, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  uint8_t *param =
      rdm_pd_get(dmx_num, RDM_PID_IDENTIFY_DEVICE, RDM_SUB_DEVICE_ROOT);
  if (param == NULL) {
    param = rdm_pd_alloc(dmx_num, sizeof(*param));
    if (param == NULL) {
      return false;
    }
    *param = 0;
  }

  const rdm_pid_description_t desc = {.pid = RDM_PID_IDENTIFY_DEVICE,
                                      .pdl_size = sizeof(*param),
                                      .data_type = RDM_DS_UNSIGNED_BYTE,
                                      .cc = RDM_CC_GET_SET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 1,
                                      .default_value = 0,
                                      .description = "Identify Device"};
  const char *param_str = "b$";

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context,
                                false);
}

static int rdm_supported_params_response_cb(dmx_port_t dmx_num,
                                            rdm_header_t *header, void *pd,
                                            uint8_t *pdl_out,
                                            const char *param_str) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if (header->cc != RDM_CC_GET_COMMAND) {
    // The supported params list is read-only
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_UNSUPPORTED_COMMAND_CLASS);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Copy all PIDs into a temporary buffer
  uint16_t pids[RDM_RESPONDER_PIDS_MAX];
  const uint32_t num_pids =
      rdm_pd_list(dmx_num, header->sub_device, pids, RDM_RESPONDER_PIDS_MAX);

  // Emplace the PIDs into the parameter data
  for (int i = 0; i < num_pids && *pdl_out <= 231; ++i) {
    switch (pids[i]) {
      // Minimum required PIDs are not included
      case RDM_PID_DISC_UNIQUE_BRANCH:
      case RDM_PID_DISC_MUTE:
      case RDM_PID_DISC_UN_MUTE:
      case RDM_PID_SUPPORTED_PARAMETERS:
      case RDM_PID_PARAMETER_DESCRIPTION:
      case RDM_PID_DEVICE_INFO:
      case RDM_PID_SOFTWARE_VERSION_LABEL:
      case RDM_PID_DMX_START_ADDRESS:
      case RDM_PID_IDENTIFY_DEVICE:
        continue;
      default:
        *pdl_out += rdm_pd_emplace_word(pd, pids[i]);
        pd += sizeof(uint16_t);
    }
  }

  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                       void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_SUPPORTED_PARAMETERS,
                                      .pdl_size = 0xe6,
                                      .data_type = RDM_DS_UNSIGNED_WORD,
                                      .cc = RDM_CC_GET,
                                      .description = "Supported Parameters"};
  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                         rdm_supported_params_response_cb, NULL, NULL, NULL,
                         false);  // FIXME: pass cb and context
}


bool rdm_register_dmx_personality(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context){
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Current personality is stored within device info
  rdm_device_info_t *di =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  DMX_CHECK(di != NULL, false, "RDM_PID_DEVICE_INFO must be registered first");

  // Note: The personality is a strange parameter that needs a custom callback
  //       because in the get case it behaves like two parameters.
  //       The pd of get is a 2 byte array consisting of the current personality
  //       and maximum number of personalities.
  //       The pd of set is byte personality.
  //       Thus we cannot use the rdm_simple_response_cb.
  //       

  uint8_t* param = &di->current_personality;

  const rdm_pid_description_t desc = {.pid = RDM_PID_DMX_PERSONALITY,
                                    .pdl_size = 1,
                                    .data_type = RDM_DS_UNSIGNED_BYTE,
                                    .cc = RDM_CC_GET_SET,
                                    .unit = RDM_UNITS_NONE,
                                    .prefix = RDM_PREFIX_NONE,
                                    .min_value = 1, 
                                    .max_value = 255,
                                    .default_value = 1, 
                                    .description = "DMX Personality"};

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, "b$",
                                rdm_personality_response_cb, param, cb, context,
                                false);
}


bool rdm_register_dmx_personality_description(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context)
{
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Personality is stored within device info
  rdm_device_info_t *di =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  DMX_CHECK(di != NULL, false, "RDM_PID_DEVICE_INFO must be registered first");

  const rdm_pid_description_t desc = {.pid = RDM_PID_DMX_PERSONALITY_DESCRIPTION,
                                    .pdl_size = 35, // this is the max size, not necessarily the one we send 
                                    .data_type = RDM_DS_BIT_FIELD,
                                    .cc = RDM_CC_GET,
                                    .unit = RDM_UNITS_NONE,
                                    .prefix = RDM_PREFIX_NONE,
                                    .min_value = 0, // has no meaning for RDM_DS_BIT_FIELD
                                    .max_value = 0, // has no meaning for RDM_DS_BIT_FIELD
                                    .default_value = 0, // has no meaning for RDM_DS_BIT_FIELD
                                    .description = "DMX Personality Description"};

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_personality_description_response_cb, NULL,
                                cb, context, false);
}


bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                    void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // DMX start address is stored within device info
  rdm_device_info_t *di =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  DMX_CHECK(di != NULL, false, "RDM_PID_DEVICE_INFO must be registered first");
  uint16_t *param = (void *)di + offsetof(rdm_device_info_t, dmx_start_address);

  const rdm_pid_description_t desc = {.pid = RDM_PID_DMX_START_ADDRESS,
                                      .pdl_size = sizeof(*param),
                                      .data_type = RDM_DS_UNSIGNED_WORD,
                                      .cc = RDM_CC_GET_SET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 1,
                                      .max_value = 512,
                                      .default_value = 1,
                                      .description = "DMX Start Address"};
  const char *param_str = "w$";

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context,
                                true);
}

bool rdm_register_parameter_description(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                        void *context)
{
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_PARAMETER_DESCRIPTION,
                                      .pdl_size = 0x34,                  // this is the max size, not necessarily the one we send
                                      .data_type = RDM_DS_UNSIGNED_BYTE, // not really true but there is no data type for complex struct
                                      .cc = RDM_CC_GET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Parameter Description"};

  const char *param_str = "wbbbbbbddda$";
  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_parameter_description_response_cb, NULL,
                                NULL, NULL, false);
}

bool rdm_register_manufacturer_specific_simple(dmx_port_t dmx_num, rdm_pid_description_t desc,
                                               void* param, const char *param_str, rdm_responder_cb_t cb,
                                               void *context, bool nvs)
{
  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context, nvs);  
}

static int rdm_status_messages_response_cb(dmx_port_t dmx_num,
                                           rdm_header_t *header, void *pd,
                                           uint8_t *pdl_out,
                                           const char *param_str) {
  // TODO: error checking

  // TODO: generate status messages for each sub-device
  const rdm_status_message_t status_message = {
      .sub_device = RDM_SUB_DEVICE_ROOT,
      .type = RDM_STATUS_ADVISORY,
      .id = 0,
      .data = {}};
  *pdl_out = rdm_pd_emplace(pd, "wbwww", &status_message, 1, false);
  return RDM_RESPONSE_TYPE_ACK;
}

static int rdm_queued_message_response_cb(dmx_port_t dmx_num,
                                          rdm_header_t *header, void *pd,
                                          uint8_t *pdl_out,
                                          const char *param_str) {
  // Verify data is valid
  const uint8_t status_type_requested = *(uint8_t *)pd;
  if (status_type_requested != RDM_STATUS_GET_LAST_MESSAGE &&
      status_type_requested != RDM_STATUS_ADVISORY &&
      status_type_requested != RDM_STATUS_WARNING &&
      status_type_requested != RDM_STATUS_ERROR) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }  // TODO: ensure error-checking is correct

  int ack;

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  uint8_t message_count = dmx_driver[dmx_num]->rdm_queue_size;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  if (message_count > 0) {
    --message_count;
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    header->pid = dmx_driver[dmx_num]->rdm_queue[message_count];
    dmx_driver[dmx_num]->rdm_queue_last_sent = header->pid;
    dmx_driver[dmx_num]->rdm_queue_size = message_count;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

    // TODO: get the PD and emplace it into pd
    
    ack = RDM_RESPONSE_TYPE_ACK;
  } else {
    // When there aren't any queued messages respond with a status message
    header->pid = RDM_PID_STATUS_MESSAGE;
    ack = rdm_status_messages_response_cb(dmx_num, header, pd, pdl_out,
                                          param_str);
  }

  return ack;
}

bool rdm_register_queued_message(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                 void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_description_t desc = {.pid = RDM_PID_QUEUED_MESSAGE,
                                      .pdl_size = 1,
                                      .data_type = RDM_DS_NOT_DEFINED,
                                      .cc = RDM_CC_GET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Queued Message"};

  return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_queued_message_response_cb, NULL, cb,
                                context, false);
}