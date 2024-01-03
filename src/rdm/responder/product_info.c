#include "include/product_info.h"

#include <string.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/device.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/responder/include/utils.h"

static size_t rdm_device_info_rh(dmx_port_t dmx_num,
                                 const rdm_parameter_definition_t *def,
                                 const rdm_header_t *header) {
  rdm_device_info_t device_info;
  size_t pdl = rdm_get_device_info(dmx_num, &device_info);
  if (pdl != sizeof(device_info)) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }

  const char *format = def->get.response.format;
  return rdm_write_ack(dmx_num, header, format, &device_info, pdl);
}

bool rdm_register_device_info(dmx_port_t dmx_num, rdm_callback_t cb,
                              void *context) {
  // TODO: arg check

  const rdm_pid_t pid = RDM_PID_DEVICE_INFO;

  // Add the parameter as a NULL static variable
  const bool nvs = false;
  dmx_parameter_add_static(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, NULL, 0);

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
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
  rdm_parameter_define(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_parameter_set_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb,
                                    context);
}

size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_info != NULL, 0, "device_info is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  device_info->model_id = driver->device.root.model_id;
  device_info->product_category = driver->device.root.product_category;
  device_info->software_version_id = driver->device.root.software_version_id;
  if (!rdm_get_dmx_personality(dmx_num, &device_info->personality)) {
    device_info->personality.count = 0;
    device_info->personality.current = 0;
  }
  if (device_info->personality.current > 0) {
    device_info->footprint =
        dmx_get_footprint(dmx_num, device_info->personality.current);
  } else {
    device_info->footprint = 0;
  }
  device_info->dmx_start_address = dmx_get_start_address(dmx_num);
  device_info->sub_device_count = 0;  // TODO
  device_info->sensor_count =
      rdm_sensor_get_count(dmx_num, RDM_SUB_DEVICE_ROOT);

  return sizeof(*device_info);
}

bool rdm_register_device_label(dmx_port_t dmx_num, const char *device_label,
                               rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DEVICE_LABEL;

  if (!dmx_parameter_exists(dmx_num, RDM_SUB_DEVICE_ROOT, pid)) {
    DMX_CHECK(device_label != NULL, false, "device_label is null");
    DMX_CHECK(strnlen(device_label, RDM_ASCII_SIZE_MAX) < RDM_ASCII_SIZE_MAX,
              false, "device_label error");
  }

  // Allocate parameter data
  const bool nvs = true;
  dmx_parameter_add_dynamic(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs,
                            device_label, RDM_ASCII_SIZE_MAX);

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET_SET,
      .ds = RDM_DS_ASCII,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "a"},
      .set = {.handler = rdm_simple_response_handler,
              .request.format = "a",
              .response.format = NULL},
      .pdl_size = 32,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_parameter_define(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_parameter_set_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb,
                                    context);
}

size_t rdm_get_device_label(dmx_port_t dmx_num, char *device_label,
                            size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_label != NULL, false, "device_label is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DEVICE_LABEL,
                            device_label, size);
}

bool rdm_set_device_label(dmx_port_t dmx_num, const char *device_label,
                          size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DEVICE_LABEL;
  if (!dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, device_label,
                         size)) {
    return false;
  }
  rdm_queue_push(dmx_num, pid);

  return true;
}

bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         char *software_version_label,
                                         rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  if (dmx_parameter_get(dmx_num, RDM_SUB_DEVICE_ROOT,
                        RDM_PID_SOFTWARE_VERSION_LABEL) == NULL) {
    DMX_CHECK(software_version_label != NULL, false,
              "software_version_label is null");
    DMX_CHECK(strnlen(software_version_label, RDM_ASCII_SIZE_MAX) <
                  RDM_ASCII_SIZE_MAX,
              false, "software_version_label error");
  }

  const rdm_pid_t pid = RDM_PID_SOFTWARE_VERSION_LABEL;

  // Add the parameter as a static variable
  const bool nvs = false;
  const size_t size = strnlen(software_version_label, RDM_ASCII_SIZE_MAX);
  dmx_parameter_add_static(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs,
                           software_version_label, size);

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_ASCII,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "a$"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = RDM_ASCII_SIZE_MAX,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_parameter_define(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_parameter_set_callback(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb,
                                    context);
}

size_t rdm_get_software_version_label(dmx_port_t dmx_num,
                                      char *software_version_label,
                                      size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(software_version_label != NULL, 0,
            "software_version_label is null");
  DMX_CHECK(size > 0, 0, "size error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT,
                            RDM_PID_SOFTWARE_VERSION_LABEL,
                            software_version_label, size);
}