#include "rdm/responder/include/dmx_setup.h"

#include "dmx/include/device.h"
#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "rdm/utils/bus_ctl.h"

// static int rdm_rhd_dmx_personality(dmx_port_t dmx_num, rdm_header_t *header,
//                                    void *pd, uint8_t *pdl_out,
//                                    const rdm_pd_schema_t *schema) {
//   // Return early if the sub-device is out of range
//   if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
//     *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
//     return RDM_RESPONSE_TYPE_NACK_REASON;
//   }

//   if (header->cc == RDM_CC_GET_COMMAND) {
//     const rdm_dmx_personality_t *data =
//         rdm_pd_get(dmx_num, header->pid, header->sub_device);
//     *pdl_out = rdm_pd_serialize(pd, 231, schema->format, data);
//   } else {
//     // Get the requested personality number from the parameter data
//     uint8_t personality_num;
//     rdm_pd_deserialize(&personality_num, sizeof(personality_num), "b$", pd);

//     // Ensure the requested personality number is within bounds
//     if (personality_num < schema->min_value ||
//         personality_num > schema->max_value) {
//       *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
//       return RDM_RESPONSE_TYPE_NACK_REASON;
//     }

//     // Just set the new personality - do not write a response
//     rdm_pd_set(dmx_num, header->pid, header->sub_device, &personality_num,
//                sizeof(uint8_t));
//   }

//   return RDM_RESPONSE_TYPE_ACK;
// }

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
  // DMX_CHECK(rdm_pd_exists(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT),
  //           false, "RDM_PID_DEVICE_INFO must be registered first");

  // Note: The personality is a strange parameter that needs a custom callback
  //       because in the get case it behaves like two parameters.
  //       The pd of get is a 2 byte array consisting of the current personality
  //       and maximum number of personalities.
  //       The pd of set is byte personality.
  //       Thus we cannot use the rdm_simple_response_cb.
  // TODO: ensure this works correctly

  // // Define the parameter
  // const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY;
  // const rdm_pd_definition_t def = {
  //     .schema = {.data_type = RDM_DS_UNSIGNED_BYTE,
  //                .cc = RDM_CC_GET_SET,
  //                .pdl_size = 1,
  //                .min_value = 1,
  //                .max_value = dmx_get_personality_count(dmx_num),
  //                .alloc_size = sizeof(rdm_dmx_personality_t),
  //                .format = "bb$"},
  //     .nvs = false,
  //     .response_handler = rdm_rhd_dmx_personality,
  // };

  // rdm_pd_add_alias(dmx_num, pid, RDM_SUB_DEVICE_ROOT, &def, RDM_PID_DEVICE_INFO,
  //                  offsetof(rdm_device_info_t, current_personality));
  // return rdm_pd_update_callback(dmx_num, pid, RDM_SUB_DEVICE_ROOT, cb, context);
  return false;
}

size_t rdm_get_dmx_personality(dmx_port_t dmx_num,
                               rdm_dmx_personality_t *personality) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(personality != NULL, 0, "personality is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  // TODO
  if (rdm_is_enabled(dmx_num)) {
    // if (rdm_pd_exists(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT)) {
    //   const rdm_dmx_personality_t *pd = rdm_pd_get_pointer(
    //       dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT);
    //   taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    //   memcpy(personality, pd, sizeof(rdm_dmx_personality_t));
    //   taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    // } else if (rdm_pd_exists(dmx_num, RDM_PID_DEVICE_INFO,
    //                          RDM_SUB_DEVICE_ROOT)) {
    //   const rdm_device_info_t *pd =
    //       rdm_pd_get_pointer(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    //   taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    //   personality->current_personality = pd->current_personality;
    //   personality->personality_count = pd->personality_count;
    //   taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    // } else {
    //   // An unusual error occurred
    //   DMX_ERR("unable to get current DMX personality");
    //   return false;
    // }
  } else {
    personality->current_personality = dmx_get_current_personality(dmx_num);
    personality->personality_count = dmx_get_personality_count(dmx_num);
  }

  return 0;
}

bool rdm_set_dmx_personality(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num error");

  bool ret = false;
  // TODO
  if (rdm_is_enabled(dmx_num)) {
    // if (rdm_pd_exists(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT)) {
    //   ret = rdm_pd_set_and_queue(dmx_num, RDM_PID_DMX_PERSONALITY,
    //                              RDM_SUB_DEVICE_ROOT, &personality_num,
    //                              sizeof(uint8_t));
    // } else if (rdm_pd_exists(dmx_num, RDM_PID_DEVICE_INFO,
    //                          RDM_SUB_DEVICE_ROOT)) {
    //   rdm_device_info_t device_info;
    //   const void *pd =
    //       rdm_pd_get_pointer(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    //   taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    //   memcpy(&device_info, pd, sizeof(rdm_device_info_t));
    //   taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    //   device_info.current_personality = personality_num;
    //   ret = rdm_pd_set(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT,
    //                    &device_info, sizeof(rdm_device_info_t));
    // } else {
    //   // An unusual error occurred
    //   DMX_ERR("unable to set current DMX personality");
    //   return false;
    // }
  } else {
    ret = dmx_set_current_personality(dmx_num, personality_num);
  }

  return ret;
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
  DMX_CHECK(
      rdm_pd_get_ptr(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DEVICE_INFO) != NULL,
      false, "RDM_PID_DEVICE_INFO must be registered first");

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
  rdm_pd_set_definition(&definition);

  // Allocate parameter data
  const bool nvs = true;
  const size_t offset = offsetof(rdm_device_info_t, dmx_start_address);
  if (rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs,
                       RDM_PID_DEVICE_INFO, offset) == NULL) {
    return false;
  }

  return rdm_pd_set_callback(pid, cb, context);
}

size_t rdm_get_dmx_start_address(dmx_port_t dmx_num,
                                 uint16_t *dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address != NULL, 0, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // FIXME: handle condition where RDM is not supported on this device

  return rdm_pd_get(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DMX_START_ADDRESS,
                    dmx_start_address, sizeof(*dmx_start_address));
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address > 0 && dmx_start_address < DMX_PACKET_SIZE_MAX, 0,
            "dmx_start_address error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // FIXME: handle condition where RDM is not supported on this device

  return rdm_pd_set_and_queue(dmx_num, RDM_PID_DMX_START_ADDRESS,
                              RDM_SUB_DEVICE_ROOT, &dmx_start_address,
                              sizeof(dmx_start_address));
}