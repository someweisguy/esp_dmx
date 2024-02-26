#include "rdm/responder/include/dmx_setup.h"

#include <string.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/device.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/include/driver.h"
#include "rdm/responder/include/utils.h"

static size_t rdm_rhd_set_dmx_personality(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  // Get the personality number from the packet
  uint8_t personality_num;
  if (!rdm_read_pd(dmx_num, definition->set.request.format, &personality_num,
                   sizeof(personality_num))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }

  // Ensure the requested personality number is within range
  if (personality_num == 0 ||
      personality_num > dmx_get_personality_count(dmx_num)) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Write the new parameter value
  rdm_dmx_personality_t personality;
  if (!dmx_parameter_copy(dmx_num, header->sub_device, header->pid,
                          &personality, sizeof(personality))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }
  personality.current = personality_num;
  if (!dmx_parameter_set(dmx_num, header->sub_device, header->pid, &personality,
                         sizeof(personality))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }

  return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
}

static size_t rdm_rhd_get_dmx_personality_description(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Get the personality number from the packet
  uint8_t personality_num;
  if (!rdm_read_pd(dmx_num, definition->get.request.format, &personality_num,
                   sizeof(personality_num))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }

  // Ensure the requested personality number is within range
  if (personality_num == 0 ||
      personality_num > dmx_get_personality_count(dmx_num)) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  rdm_dmx_personality_description_t pd;
  pd.personality_num = personality_num;
  pd.footprint = dmx_get_footprint(dmx_num, personality_num);
  const char *desc = dmx_get_personality_description(dmx_num, personality_num);
  memcpy(pd.description, desc, strnlen(desc, RDM_ASCII_SIZE_MAX));

  const size_t pdl = sizeof(pd);
  return rdm_write_ack(dmx_num, header, definition->get.response.format, &pd,
                       pdl);
}

size_t rdm_get_dmx_personality_description(
    dmx_port_t dmx_num, uint8_t personality_num,
    rdm_dmx_personality_description_t *personality_description) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY_DESCRIPTION;

  // Guard against an out-of-bounds error
  const uint32_t personality_num_max =
      (dmx_parameter_size(dmx_num, RDM_SUB_DEVICE_ROOT, pid) /
       sizeof(*personality_description));
  if (personality_num == 0 || personality_num > personality_num_max) {
    return 0;
  }

  // Get a pointer to the stored personalities
  const rdm_dmx_personality_description_t *personalities =
      dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT, pid);
  if (personalities == NULL) {
    return 0;
  }

  // Copy the parameter
  --personality_num;  // Personalities begin at 1
  memcpy(personality_description, &personalities[personality_num],
         sizeof(*personality_description));

  return sizeof(*personality_description);
}

bool rdm_register_dmx_personality(dmx_port_t dmx_num, uint8_t personality_count,
                                  rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY;

  // Attempt to load the value from NVS
  rdm_dmx_personality_t personality;
  if (!dmx_nvs_get(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &personality,
                   sizeof(personality)) ||
      personality.count != personality_count) {
    personality.current = 1;
    personality.count = personality_count;
  }

  // Allocate parameter data
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, &personality,
                         sizeof(personality))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
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
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_dmx_personality(dmx_port_t dmx_num,
                               rdm_dmx_personality_t *personality) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(personality != NULL, 0, "personality is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT,
                            RDM_PID_DMX_PERSONALITY, personality,
                            sizeof(*personality));
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
  if (!dmx_parameter_copy(dmx_num, sub_device, pid, &personality,
                          sizeof(personality))) {
    return false;
  }
  personality.current = personality_num;
  if (!dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &personality,
                         sizeof(personality))) {
    return false;
  }
  rdm_queue_push(dmx_num, pid);

  return true;
}

bool rdm_register_dmx_personality_description(
    dmx_port_t dmx_num, rdm_dmx_personality_description_t *personalities,
    uint32_t count, rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DMX_PERSONALITY_DESCRIPTION;

  // Validate personalities is valid
  if (!dmx_parameter_exists(dmx_num, RDM_SUB_DEVICE_ROOT, pid)) {
    for (int i = 0; i < count; ++i) {
      DMX_CHECK(personalities[i].personality_num == i + 1, false,
                "personality_num error");
      DMX_CHECK((personalities[i].footprint > 0 &&
                 personalities[i].footprint < DMX_PACKET_SIZE_MAX),
                false, "footprint error");
      // The personality description field is not validated
    }
  }

  // Allocate parameter data
  const size_t size = count * sizeof(*personalities);
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_DYNAMIC, personalities, size)) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_get_dmx_personality_description,
              .request.format = "b$",
              .response.format = "bwa"},
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

bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_callback_t cb,
                                    void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DMX_START_ADDRESS;

  // Attempt to load the value from NVS
  uint16_t dmx_start_address;
  if (!dmx_nvs_get(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &dmx_start_address,
                   sizeof(dmx_start_address))) {
    dmx_start_address = 1;
  }

  // Reset the DMX start address if the current footprint is too large
  const int current_personality = dmx_get_current_personality(dmx_num);
  if (current_personality > 0) {
    int fp = dmx_get_footprint(dmx_num, dmx_get_current_personality(dmx_num));
    if (dmx_start_address + fp > DMX_PACKET_SIZE_MAX) {
      dmx_start_address = 1;
    }
  }

  // Allocate parameter data
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, &dmx_start_address,
                         sizeof(dmx_start_address))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
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
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_dmx_start_address(dmx_port_t dmx_num,
                                 uint16_t *dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address != NULL, 0, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT,
                            RDM_PID_DMX_START_ADDRESS, dmx_start_address,
                            sizeof(*dmx_start_address));
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address > 0 && dmx_start_address < DMX_PACKET_SIZE_MAX, 0,
            "dmx_start_address error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DMX_START_ADDRESS;
  if (!dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &dmx_start_address,
                         sizeof(dmx_start_address))) {
    return false;
  }
  rdm_queue_push(dmx_num, pid);

  return true;
}