#include "responder.h"

#include "dmx/config.h"
#include "dmx/hal.h"
#include "dmx/nvs.h"
#include "dmx/struct.h"
#include "esp_dmx.h"
#include "rdm_utils.h"

static int rdm_default_discovery_cb(dmx_port_t dmx_num,
                                    const rdm_header_t *header, void *pd,
                                    uint8_t *pdl_out, void *param,
                                    const rdm_pid_description_t *desc,
                                    const char *param_str) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    // Cannot respond to RDM_CC_DISC_COMMAND with NACK
    return RDM_RESPONSE_TYPE_NONE;
  }

  int response_type;
  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // Ignore this message if discovery is muted
    const uint8_t *is_muted = rdm_pd_find(dmx_num, RDM_PID_DISC_MUTE);
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
    uint8_t *is_muted = param;
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_default_discovery_cb, NULL, cb, context);
}

bool rdm_register_disc_mute(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                            void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  uint8_t *param = rdm_pd_find(dmx_num, RDM_PID_DISC_MUTE);
  if (param == NULL) {
    param = rdm_pd_find(dmx_num, RDM_PID_DISC_UN_MUTE);
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_default_discovery_cb, param, cb, context);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                               void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  uint8_t *param = rdm_pd_find(dmx_num, RDM_PID_DISC_UN_MUTE);
  if (param == NULL) {
    param = rdm_pd_find(dmx_num, RDM_PID_DISC_MUTE);
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_default_discovery_cb, param, cb, context);
}

static int rdm_simple_response_cb(dmx_port_t dmx_num,
                                  const rdm_header_t *header, void *pd,
                                  uint8_t *pdl_out, void *param,
                                  const rdm_pid_description_t *desc,
                                  const char *param_str) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if (header->cc == RDM_CC_GET_COMMAND) {
    *pdl_out = rdm_pd_emplace(pd, param_str, param, desc->pdl_size, false);
  } else {
    rdm_pd_emplace(param, param_str, pd, header->pdl, true);
  }

  return RDM_RESPONSE_TYPE_ACK;
}

static int rdm_supported_params_response_cb(dmx_port_t dmx_num,
                                            const rdm_header_t *header, void *pd,
                                            uint8_t *pdl_out, void *param,
                                            const rdm_pid_description_t *desc,
                                            const char *param_str) 
{
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if (header->cc != RDM_CC_GET_COMMAND) {
    //the supported params list is read-only
      *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_WRITE_PROTECT);
      return RDM_RESPONSE_TYPE_NACK_REASON;
    }

  uint16_t *params = (uint16_t*)param;

  int i = 0;
  for(; i < RDM_RESPONDER_NUM_PIDS_OPTIONAL; i++) {
    if(params[i] == 0) {
      break;
    }
    rdm_pd_emplace_word(pd, params[i]);
    pd += sizeof(uint16_t);
  }
  *pdl_out = i * sizeof(uint16_t);

  return RDM_RESPONSE_TYPE_ACK;
}

static int rdm_personality_description_response_cb(dmx_port_t dmx_num,
                                            const rdm_header_t *header, void *pd,
                                            uint8_t *pdl_out, void *param,
                                            const rdm_pid_description_t *desc,
                                            const char *param_str) 
{
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  rdm_device_info_t *di = rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
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
static int rdm_personality_response_cb(dmx_port_t dmx_num,
                                            const rdm_header_t *header, void *pd,
                                            uint8_t *pdl_out, void *param,
                                            const rdm_pid_description_t *desc,
                                            const char *param_str) 
{
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  rdm_device_info_t *di = rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
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


bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info,
                              rdm_responder_cb_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_device_info_t *param = rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context);
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

  char *param = rdm_pd_find(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL);
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                  rdm_simple_response_cb, param, cb, context);
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

  char *param = rdm_pd_find(dmx_num, RDM_PID_DEVICE_LABEL);
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context);
}

bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(cb != NULL, false, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  uint8_t *param = rdm_pd_find(dmx_num, RDM_PID_IDENTIFY_DEVICE);
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context);
}


bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context)
{
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const uint16_t size = RDM_RESPONDER_NUM_PIDS_OPTIONAL * sizeof(uint16_t);
  uint16_t *param = rdm_pd_find(dmx_num, RDM_PID_SUPPORTED_PARAMETERS);
  if (param == NULL) {
    param = rdm_pd_alloc(dmx_num, size);
    if (param == NULL) {
      return false;
    }
    memset(param, 0, size);
  }

  const rdm_pid_description_t desc = {.pid = RDM_PID_SUPPORTED_PARAMETERS,
                                      .pdl_size = size,
                                      .data_type = RDM_DS_UNSIGNED_WORD,
                                      .cc = RDM_CC_GET,
                                      .unit = RDM_UNITS_NONE,
                                      .prefix = RDM_PREFIX_NONE,
                                      .min_value = 0,
                                      .max_value = 0,
                                      .default_value = 0,
                                      .description = "Supported Parameters"};
  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_supported_params_response_cb, param, NULL, NULL);  
}


bool rdm_register_dmx_personality(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context){
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Current personality is stored within device info
  rdm_device_info_t *di = rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, "b$",
                                rdm_personality_response_cb, param, cb, context);  
}


bool rdm_register_dmx_personality_description(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                  void *context)
{
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Personality is stored within device info
  rdm_device_info_t *di = rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
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


  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, NULL,
                                rdm_personality_description_response_cb, NULL, cb, context);  


}


bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_responder_cb_t cb,
                                    void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // DMX start address is stored within device info
  rdm_device_info_t *di = rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
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

  return rdm_register_parameter(dmx_num, RDM_SUB_DEVICE_ROOT, &desc, param_str,
                                rdm_simple_response_cb, param, cb, context);
}
