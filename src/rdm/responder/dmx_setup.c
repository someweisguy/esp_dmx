#include "rdm/responder/dmx_setup.h"

#include "dmx/driver.h"
#include "dmx/struct.h"
#include "dmx/device.h"
#include "rdm/utils/bus_ctl.h"

static int rdm_rhd_dmx_personality(dmx_port_t dmx_num, rdm_header_t *header,
                                   void *pd, uint8_t *pdl_out,
                                   const rdm_pd_schema_t *schema) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if (header->cc == RDM_CC_GET_COMMAND) {
    const rdm_dmx_personality_t *data =
        rdm_pd_get(dmx_num, header->pid, header->sub_device);
    *pdl_out = rdm_emplace(pd, schema->format, data, 231, false);
  } else {
    // Get the requested personality number from the parameter data
    uint8_t personality_num;
    rdm_emplace(&personality_num, "b$", pd, 231, true);

    // Ensure the requested personality number is within bounds
    if (personality_num < schema->min_value ||
        personality_num > schema->max_value) {
      *pdl_out = rdm_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
      return RDM_RESPONSE_TYPE_NACK_REASON;
    }

    // Just set the new personality - do not write a response
    rdm_pd_set(dmx_num, header->pid, header->sub_device, &personality_num,
               sizeof(uint8_t));
  }

  return RDM_RESPONSE_TYPE_ACK;
}

static int rdm_rhd_dmx_personality_description(dmx_port_t dmx_num,
                                               rdm_header_t *header, void *pd,
                                               uint8_t *pdl_out,
                                               const rdm_pd_schema_t *schema) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Get the requested personality number from the parameter data
  uint8_t personality_num;
  rdm_emplace(&personality_num, "b$", pd, 231, true);

  // Ensure the requested personality number is within bounds
  if (personality_num < schema->min_value ||
      personality_num > schema->max_value) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Attempt to get the personality description
  dmx_personality_description_t pers_desc;
  if (!dmx_get_personality_description(dmx_num, personality_num, &pers_desc)) {
    // This code should not run
    *pdl_out = rdm_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Emplace the response
  *pdl_out = rdm_emplace(pd, schema->format, &pers_desc, 231, false);
  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_dmx_personality(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context){
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(rdm_pd_exists(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT),
            false, "RDM_PID_DEVICE_INFO must be registered first");

  // Note: The personality is a strange parameter that needs a custom callback
  //       because in the get case it behaves like two parameters.
  //       The pd of get is a 2 byte array consisting of the current personality
  //       and maximum number of personalities.
  //       The pd of set is byte personality.
  //       Thus we cannot use the rdm_simple_response_cb.
  // TODO: ensure this works correctly

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY;
  const rdm_pd_definition_t def = {
      .schema = {.data_type = RDM_DS_UNSIGNED_BYTE,
                 .cc = RDM_CC_GET_SET,
                 .pdl_size = 1,
                 .format = "bb$",
                 .min_value = 1,
                 .max_value = dmx_get_personality_count(dmx_num)},
      .pd_size = sizeof(rdm_dmx_personality_t),
      .nvs = false,
      .response_handler = rdm_rhd_dmx_personality,
  };

  rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &def, RDM_PID_DEVICE_INFO,
                   offsetof(rdm_device_info_t, current_personality));
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_register_dmx_personality_description(dmx_port_t dmx_num,
                                              rdm_callback_t cb,
                                              void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY_DESCRIPTION;
  const rdm_pd_definition_t def = {
      .schema = {.data_type = RDM_DS_ASCII,
                 .cc = RDM_CC_GET,
                 .pdl_size = 1,
                 .format = "bwa$",
                 .min_value = 1,
                 .max_value = dmx_get_personality_count(dmx_num)},
      .pd_size = 0,  // Parameter is deterministic
      .nvs = false,
      .response_handler = rdm_rhd_dmx_personality_description,
  };

  rdm_pd_add_deterministic(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &def);
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_callback_t cb,
                                    void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(rdm_pd_exists(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT),
            false, "RDM_PID_DEVICE_INFO must be registered first");

    // Define the parameter
  const rdm_pid_t pid = RDM_PID_DMX_START_ADDRESS;
  const rdm_pd_definition_t def = {
      .schema = {.data_type = RDM_DS_UNSIGNED_WORD,
                 .cc = RDM_CC_GET_SET,
                 .pdl_size = sizeof(uint16_t),
                 .format = "w$",
                 .min_value = 1,
                 .max_value = 512},
      .pd_size = sizeof(uint16_t),
      .nvs = true,
      .response_handler = rdm_response_handler_simple,
  };

  rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &def, RDM_PID_DEVICE_INFO,
                   offsetof(rdm_device_info_t, dmx_start_address));
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}