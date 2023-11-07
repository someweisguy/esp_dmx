#include "dmx_setup.h"

#include "dmx/struct.h"


static int rdm_personality_response_cb(dmx_port_t dmx_num, rdm_header_t *header,
                                       void *pd, uint8_t *pdl_out,
                                       const char *format) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // rdm_device_info_t *di =
  //     rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  // if(di == NULL)
  // {
  //   //none of the error codes really fit, thus we just go with unknown pid
  //   *pdl_out = rdm_emplace_word(pd, RDM_NR_UNKNOWN_PID); 
  //   return RDM_RESPONSE_TYPE_NACK_REASON;
  // }

  // if(header->cc == RDM_CC_GET_COMMAND)
  // {
  //   const uint8_t data[] = {di->current_personality, di->personality_count};
  //   memcpy(pd, data, 2);
  //   *pdl_out = 2;
  //   return RDM_RESPONSE_TYPE_ACK;
  // }
  // else if(header->cc == RDM_CC_SET_COMMAND)
  // {
  //   if(header->pdl != 1)
  //   {
  //     *pdl_out = rdm_emplace_word(pd, RDM_NR_FORMAT_ERROR);
  //     return RDM_RESPONSE_TYPE_NACK_REASON;
  //   }

  //   const uint8_t requestedPersonality = *((uint8_t*)pd);
  //   if(requestedPersonality >= di->personality_count)
  //   {
  //     *pdl_out = rdm_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
  //     return RDM_RESPONSE_TYPE_NACK_REASON;
  //   }

  //   dmx_set_current_personality(dmx_num, requestedPersonality);
  //   //note: we do not need to set it in nvs because that is done in hal.c:dmx_receive()
  //   return RDM_RESPONSE_TYPE_ACK;
  // }
  // else
  // {
  //   *pdl_out = rdm_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
  //   return RDM_RESPONSE_TYPE_NACK_REASON;
  // }

  // FIXME
  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_dmx_personality(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context){
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT) != NULL,
      false, "RDM_PID_DEVICE_INFO must be registered first");

  // Note: The personality is a strange parameter that needs a custom callback
  //       because in the get case it behaves like two parameters.
  //       The pd of get is a 2 byte array consisting of the current personality
  //       and maximum number of personalities.
  //       The pd of set is byte personality.
  //       Thus we cannot use the rdm_simple_response_cb.

  const rdm_pid_description_t pd_def = {.pid = RDM_PID_DMX_PERSONALITY,
                                    .pdl_size = 1,
                                    .data_type = RDM_DS_UNSIGNED_BYTE,
                                    .cc = RDM_CC_GET_SET,
                                    .unit = RDM_UNITS_NONE,
                                    .prefix = RDM_PREFIX_NONE,
                                    .min_value = 1, 
                                    .max_value = 255,
                                    .default_value = 1, 
                                    .description = "DMX Personality"};
  const char *format = "bb$";
  const bool nvs = false;

  rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, &pd_def, format, nvs,
                   rdm_personality_response_cb, RDM_PID_DEVICE_INFO,
                   offsetof(rdm_device_info_t, current_personality));
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT,
                                RDM_PID_DMX_PERSONALITY, cb, context);
}

static int rdm_personality_description_response_cb(dmx_port_t dmx_num,
                                                   rdm_header_t *header,
                                                   void *pd, uint8_t *pdl_out,
                                                   const char *format) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // rdm_device_info_t *di =
  //     rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, header->sub_device);
  // if(di == NULL)
  // {
  //   //none of the error codes really fit, thus we just go with unknown pid
  //   *pdl_out = rdm_emplace_word(pd, RDM_NR_UNKNOWN_PID);
  //   return RDM_RESPONSE_TYPE_NACK_REASON;
  // }

  // if(header->cc == RDM_CC_SET_COMMAND)
  // {      
  //   *pdl_out = rdm_emplace_word(pd, RDM_NR_WRITE_PROTECT);
  //   return RDM_RESPONSE_TYPE_NACK_REASON;
  // }

  // if(header->pdl != 1)
  // {
  //   *pdl_out = rdm_emplace_word(pd, RDM_NR_FORMAT_ERROR);
  //   return RDM_RESPONSE_TYPE_NACK_REASON;
  // }

  // const uint8_t requestedPersonality = *((uint8_t*)pd);
  // const uint16_t footprint = (uint16_t)dmx_get_footprint(dmx_num, requestedPersonality);
  // const char* personalityDesc = dmx_get_personality_description(dmx_num, requestedPersonality);
  
  // if(personalityDesc == NULL)
  // {
  //   *pdl_out = rdm_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
  //   return RDM_RESPONSE_TYPE_NACK_REASON;
  // }

  // memcpy(pd, &requestedPersonality, 1);
  // pd++;
  // rdm_emplace_word(pd, footprint);
  // pd += 2;
  // const size_t emplacedBytes = rdm_emplace(pd, "a$", personalityDesc, 32, false);
  // *pdl_out = 3 + emplacedBytes;

  // FIXME
  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_dmx_personality_description(dmx_port_t dmx_num,
                                              rdm_callback_t cb,
                                              void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT) != NULL,
      false, "RDM_PID_DEVICE_INFO must be registered first");

  const rdm_pid_description_t pd_def = {
      .pid = RDM_PID_DMX_PERSONALITY_DESCRIPTION,
      .pdl_size = 35,
      .data_type = RDM_DS_BIT_FIELD,
      .cc = RDM_CC_GET,
      .description = "DMX Personality Description"};
  const char *format = "a$";
  const bool nvs = false;

  rdm_pd_add_deterministic(dmx_num, RDM_SUB_DEVICE_ROOT, &pd_def, format,
                           rdm_personality_description_response_cb);

  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT,
                                RDM_PID_DMX_PERSONALITY_DESCRIPTION, cb,
                                context);
}

bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_callback_t cb,
                                    void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT) != NULL,
      false, "RDM_PID_DEVICE_INFO must be registered first");

  const rdm_pid_description_t pd_def = {.pid = RDM_PID_DMX_START_ADDRESS,
                                        .pdl_size = sizeof(uint16_t),
                                        .data_type = RDM_DS_UNSIGNED_WORD,
                                        .cc = RDM_CC_GET_SET,
                                        .unit = RDM_UNITS_NONE,
                                        .prefix = RDM_PREFIX_NONE,
                                        .min_value = 1,
                                        .max_value = 512,
                                        .default_value = 1,
                                        .description = "DMX Start Address"};
  const char *format = "w$";
  const bool nvs = true;

  rdm_pd_add_alias(dmx_num, RDM_SUB_DEVICE_ROOT, &pd_def, format, nvs,
                   rdm_simple_response_cb, RDM_PID_DEVICE_INFO,
                   offsetof(rdm_device_info_t, dmx_start_address));

  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT,
                                RDM_PID_DEVICE_INFO, cb, context);
}