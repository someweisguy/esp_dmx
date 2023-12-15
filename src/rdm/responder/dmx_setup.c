#include "rdm/responder/include/dmx_setup.h"

#include "dmx/include/device.h"
#include "dmx/include/driver.h"
#include "dmx/include/io.h"
#include "dmx/include/struct.h"
#include "rdm/responder/include/utils.h"

static size_t rdm_rhd_set_dmx_personality(dmx_port_t dmx_num,
                                          const rdm_pd_definition_t *definition,
                                          const rdm_header_t *header) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    return rdm_write_nack_reason(dmx_num, header,
                                 RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  }

  // Get the personality number from the packet
  uint8_t personality_num;
  if (!rdm_read_pd(dmx_num, definition->set.request.format, &personality_num,
                   sizeof(personality_num))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }

  // Ensure the requested personality number is within range
  if (personality_num == 0 ||
      personality_num > dmx_get_personality_count(dmx_num)) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Write the new parameter value
  rdm_dmx_personality_t personality;
  if (!rdm_parameter_copy(dmx_num, header->sub_device, header->pid, &personality,
                  sizeof(personality))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }
  personality.current_personality = personality_num;
  if (!rdm_parameter_set(dmx_num, header->sub_device, header->pid, &personality,
                  sizeof(personality))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }

  return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
}

// static int rdm_rhd_dmx_personality_description(dmx_port_t dmx_num,
//                                                rdm_header_t *header, void *pd,
//                                                uint8_t *pdl_out,
//                                                const rdm_pd_schema_t *schema) {
//   if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
//     *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
//     return RDM_RESPONSE_TYPE_NACK_REASON;
//   }

//   // Get the requested personality number from the parameter data
//   uint8_t personality_num;
//   rdm_pd_deserialize(&personality_num, sizeof(personality_num), "b$", pd);

//   // Ensure the requested personality number is within bounds
//   if (personality_num < schema->min_value ||
//       personality_num > schema->max_value) {
//     *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
//     return RDM_RESPONSE_TYPE_NACK_REASON;
//   }

//   // Attempt to get the personality description
//   dmx_personality_description_t pers_desc;
//   if (!dmx_get_personality_description(dmx_num, personality_num, &pers_desc)) {
//     // This code should not run
//     *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
//     return RDM_RESPONSE_TYPE_NACK_REASON;
//   }

//   // Emplace the response
//   *pdl_out = rdm_pd_serialize(pd, 231, schema->format, &pers_desc);
//   return RDM_RESPONSE_TYPE_ACK;
// }

bool rdm_register_dmx_personality(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = sizeof(rdm_dmx_personality_t),
      .pid_cc = RDM_CC_GET_SET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "bb$"},
      .set = {.handler = rdm_rhd_set_dmx_personality,
              .request.format = "b$",
              .response.format = NULL},
      .pdl_size = sizeof(rdm_dmx_personality_t),
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_parameter_define(&definition);

  // Allocate parameter data
  const bool nvs = true;
  const rdm_dmx_personality_t init_value = {
      dmx_get_current_personality(dmx_num), dmx_get_personality_count(dmx_num)};
  if (!rdm_parameter_add_dynamic(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs,
                                 &init_value, sizeof(init_value))) {
    return false;
  }

  return rdm_pd_set_callback(pid, cb, context);
}

size_t rdm_get_dmx_personality(dmx_port_t dmx_num,
                               rdm_dmx_personality_t *personality) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(personality != NULL, 0, "personality is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DMX_PERSONALITY,
                    personality, sizeof(*personality));

}

bool rdm_set_dmx_personality(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num error");

  const rdm_sub_device_t sub_device = RDM_SUB_DEVICE_ROOT;
  const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY;

  rdm_dmx_personality_t personality;
  if (!rdm_parameter_copy(dmx_num, sub_device, pid, &personality,
                  sizeof(personality))) {
    return false;
  }
  personality.current_personality = personality_num;
  if (!rdm_parameter_set_and_queue(dmx_num, sub_device, pid, &personality,
                            sizeof(personality))) {
    return false;
  }

  return true;
}

bool rdm_register_dmx_personality_description(dmx_port_t dmx_num,
                                              rdm_callback_t cb,
                                              void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // // Define the parameter
  // const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY_DESCRIPTION;
  // const rdm_pd_definition_t def = {
  //     .schema = {.data_type = RDM_DS_ASCII,
  //                .cc = RDM_CC_GET,
  //                .pdl_size = 1,
  //                .min_value = 1,
  //                .max_value = dmx_get_personality_count(dmx_num),
  //                .alloc_size = 0,  // Parameter is deterministic
  //                .format = "bwa$"},
  //     .nvs = false,
  //     .response_handler = rdm_rhd_dmx_personality_description,
  // };

  // rdm_pd_add_deterministic(dmx_num, pid, RDM_SUB_DEVICE_ROOT, &def,
  //                          rdm_personality_description_wrapper);
  // return rdm_pd_update_callback(dmx_num, pid, RDM_SUB_DEVICE_ROOT, cb, context);
  return false;
}

bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_callback_t cb,
                                    void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DMX_START_ADDRESS;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = sizeof(uint16_t),
      .pid_cc = RDM_CC_GET_SET,
      .ds = RDM_DS_UNSIGNED_WORD,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "w$"},
      .set = {.handler = rdm_simple_response_handler,
              .request.format = "w$",
              .response.format = NULL},
      .pdl_size = sizeof(uint16_t),
      .max_value = 512,
      .min_value = 1,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_parameter_define(&definition);

  // Allocate parameter data
  const bool nvs = true;
  const uint16_t init_value = 1;
  rdm_parameter_add_dynamic(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, &init_value,
                            sizeof(init_value));

  return rdm_pd_set_callback(pid, cb, context);
}

size_t rdm_get_dmx_start_address(dmx_port_t dmx_num,
                                 uint16_t *dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address != NULL, 0, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DMX_START_ADDRESS,
                    dmx_start_address, sizeof(*dmx_start_address));
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address > 0 && dmx_start_address < DMX_PACKET_SIZE_MAX, 0,
            "dmx_start_address error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_parameter_set_and_queue(dmx_num, RDM_PID_DMX_START_ADDRESS,
                              RDM_SUB_DEVICE_ROOT, &dmx_start_address,
                              sizeof(dmx_start_address));
}