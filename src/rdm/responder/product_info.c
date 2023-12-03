#include "product_info.h"

#include "dmx/device.h"
#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"

bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info, rdm_callback_t cb,
                              void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  if (rdm_pd_get_ptr(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DEVICE_INFO) ==
      NULL) {
    // Ensure the user's default value is valid
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
      if (!dmx_nvs_get(dmx_num, RDM_PID_DMX_START_ADDRESS, RDM_SUB_DEVICE_ROOT,
                       RDM_DS_UNSIGNED_WORD, &device_info->dmx_start_address,
                       sizeof(device_info->dmx_start_address))) {
        device_info->dmx_start_address = 1;
      }
    }
    // Load the current DMX personality from NVS
    if (device_info->current_personality == 0 &&
        device_info->dmx_start_address != DMX_START_ADDRESS_NONE) {
      rdm_dmx_personality_t personality;
      if (!dmx_nvs_get(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT,
                       RDM_DS_BIT_FIELD, &personality, sizeof(personality)) ||
          personality.personality_count != device_info->personality_count) {
        device_info->current_personality = 1;
      } else {
        device_info->current_personality = personality.current_personality;
      }
      device_info->footprint =
          dmx_get_footprint(dmx_num, device_info->current_personality);
    }
  }

  // Allocate parameter data
  const bool nvs = false;
  const rdm_pid_t pid = RDM_PID_DEVICE_INFO;
  if (rdm_pd_add_variable(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, device_info,
                          sizeof(*device_info)) == NULL) {
    return false;
  }

  // Define the parameter
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = sizeof(*device_info),
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "x01x00wwdwbbwwb$"},
      .set = {.handler = NULL,
              .request.format = NULL,
              .response.format = NULL},
      .pdl_size = 0,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_pd_set_definition(dmx_num, pid, &definition);

  return rdm_pd_set_callback(dmx_num, pid, cb, context);
}

bool rdm_register_device_label(dmx_port_t dmx_num, const char *device_label,
                               rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  // if (rdm_pd_get(dmx_num, RDM_PID_DEVICE_LABEL, RDM_SUB_DEVICE_ROOT) == NULL) {
  //   DMX_CHECK(device_label != NULL, false, "device_label is null");
  //   DMX_CHECK(strnlen(device_label, 33) < 33, false, "device_label error");
  // }

  // // Define the parameter
  // const rdm_pid_t pid = RDM_PID_DEVICE_LABEL;
  // const rdm_pd_definition_t def = {
  //     .schema = {.data_type = RDM_DS_ASCII,
  //                .cc = RDM_CC_GET_SET,
  //                .pdl_size = 33,
  //                .alloc_size = 33,
  //                .format = "a$"},
  //     .nvs = true,
  //     .response_handler = rdm_response_handler_simple,
  // };

  // rdm_pd_add_new(dmx_num, pid, RDM_SUB_DEVICE_ROOT, &def, device_label);
  // return rdm_pd_update_callback(dmx_num, pid, RDM_SUB_DEVICE_ROOT, cb, context);
  return false;
}

bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         const char *software_version_label,
                                         rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  if (rdm_pd_get_ptr(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL,
                     RDM_SUB_DEVICE_ROOT) == NULL) {
    DMX_CHECK(software_version_label != NULL, false,
              "software_version_label is null");
    DMX_CHECK(strnlen(software_version_label, 33) < 33, false,
              "software_version_label error");
  }

  // Allocate parameter data
  const rdm_pid_t pid = RDM_PID_SOFTWARE_VERSION_LABEL;
  if (rdm_pd_add_const(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                       software_version_label) == NULL) {
    return false;
  }

  // Define the parameter
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = sizeof(*software_version_label),
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_ASCII,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "a$"},
      .set = {.handler = NULL,
              .request.format = NULL,
              .response.format = NULL},
      .pdl_size = 0,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_pd_set_definition(dmx_num, pid, &definition);

  return rdm_pd_set_callback(dmx_num, pid, cb, context);
}