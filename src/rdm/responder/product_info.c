#include "include/product_info.h"

#include <string.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/device.h"
#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "rdm/responder/include/utils.h"

static size_t rdm_device_info_rh(dmx_port_t dmx_num,
                                 const rdm_pd_definition_t *def,
                                 const rdm_header_t *header) {
  rdm_device_info_t device_info;
  size_t pdl = rdm_get_device_info(dmx_num, &device_info);
  if (pdl != sizeof(device_info)) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }

  const char *format = def->get.response.format;
  return rdm_write_ack(dmx_num, header, format, &device_info, pdl);
}

bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info, rdm_callback_t cb,
                              void *context) {

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_DEVICE_INFO;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = 0,
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_device_info_rh,
              .request.format = NULL,
              .response.format = "x01x00wwdwbbwwb$"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = 0,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_pd_set_definition(&definition);

  // Add the parameter as a NULL static variable
  const bool nvs = false;
  rdm_parameter_add_static(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, NULL);

  return rdm_pd_set_callback(pid, cb, context);
}

size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_info != NULL, 0, "device_info is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  device_info->model_id = driver->rdm.root_device.model_id;
  device_info->product_category = driver->rdm.root_device.product_category;
  device_info->software_version_id =
      driver->rdm.root_device.software_version_id;
  const int current_personality = dmx_get_current_personality(dmx_num);
  device_info->current_personality = current_personality;
  const int footprint = current_personality > 0
                            ? dmx_get_footprint(dmx_num, current_personality)
                            : 0;
  device_info->footprint = footprint;
  device_info->personality_count = dmx_get_personality_count(dmx_num);
  device_info->dmx_start_address = dmx_get_start_address(dmx_num);
  device_info->sub_device_count = 0;  // TODO
  device_info->sensor_count = 0;      // TODO

  return sizeof(*device_info);
}

bool rdm_register_device_label(dmx_port_t dmx_num, const char *device_label,
                               rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  // TODO
  // if (rdm_pd_get(dmx_num, RDM_PID_DEVICE_LABEL, RDM_SUB_DEVICE_ROOT) == NULL)
  // {
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
  // return rdm_pd_update_callback(dmx_num, pid, RDM_SUB_DEVICE_ROOT, cb,
  // context);
  return false;
}

size_t rdm_get_device_label(dmx_port_t dmx_num, char *device_label,
                            size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_label != NULL, false, "device_label is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_pd_get(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DEVICE_LABEL,
                    device_label, size);
}

bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         char *software_version_label,
                                         rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  if (rdm_pd_get_ptr(dmx_num, RDM_SUB_DEVICE_ROOT,
                     RDM_PID_SOFTWARE_VERSION_LABEL) == NULL) {
    DMX_CHECK(software_version_label != NULL, false,
              "software_version_label is null");
    DMX_CHECK(strnlen(software_version_label, 33) < 33, false,
              "software_version_label error");
  }

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_SOFTWARE_VERSION_LABEL;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
      .alloc_size = 32,
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_ASCII,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "a$"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = 0,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_pd_set_definition(&definition);

  // Add the parameter as a static variable
  const bool nvs = false;
  rdm_parameter_add_static(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs,
                           software_version_label);

  return rdm_pd_set_callback(pid, cb, context);
}

size_t rdm_get_software_version_label(dmx_port_t dmx_num,
                                      char *software_version_label,
                                      size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(software_version_label != NULL, 0,
            "software_version_label is null");
  DMX_CHECK(size > 0, 0, "size error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_pd_get(dmx_num, RDM_SUB_DEVICE_ROOT,
                    RDM_PID_SOFTWARE_VERSION_LABEL, software_version_label,
                    size);
}