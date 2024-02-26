#include "include/product_info.h"

#include <string.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/device.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/responder/include/utils.h"

/** @brief Product information used in the RDM_PID_DEVICE_INFO parameter. All
 * other fields in RDM_PID_DEVICE_INFO can be computed at call-time.
 */
struct rdm_product_info_t {
  uint16_t model_id;  // The model ID of the device. Unique per manufacturer.
  uint16_t product_category;     // Enumerated in rdm_product_category_t.
  uint32_t software_version_id;  // The unique software verion id of the device.
};

static size_t rdm_rhd_get_device_info(dmx_port_t dmx_num,
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

static size_t rdm_rhd_get_set_language(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  if (header->cc == RDM_CC_GET_COMMAND) {
    // Get the parameter and write it to the RDM bus
    size_t pdl = dmx_parameter_size(dmx_num, header->sub_device, header->pid);
    const void *pd =
        dmx_parameter_get_data(dmx_num, header->sub_device, header->pid);
    return rdm_write_ack(dmx_num, header, definition->get.response.format, pd,
                         pdl);
  } else {
    // Verify the language code is two characters long
    if (header->pdl != 2) {
      return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
    }
    // Get the parameter from the request and write it to the RDM driver
    char language[3];
    if (!rdm_read_pd(dmx_num, definition->set.request.format, language,
                     sizeof(language))) {
      return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
    }
    dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, header->pid, language,
                      sizeof(language));
    return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
  }
}

bool rdm_register_device_info(dmx_port_t dmx_num, uint16_t model_id,
                              uint16_t product_category,
                              uint32_t software_version_id, rdm_callback_t cb,
                              void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_DEVICE_INFO;

  // Add the parameter dynamically - only the product info is stored
  struct rdm_product_info_t product_info = {
      .model_id = model_id,
      .product_category = product_category,
      .software_version_id = software_version_id};
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_DYNAMIC, &product_info,
                         sizeof(product_info))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_get_device_info,
              .request.format = NULL,
              .response.format = "x01x00wwdwbbwwb$"},
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

size_t rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_info != NULL, 0, "device_info is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Get the product info for the device
  const struct rdm_product_info_t *product_info =
      dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DEVICE_INFO);

  if (product_info != NULL) {
    device_info->model_id = product_info->model_id;
    device_info->product_category = product_info->product_category;
    device_info->software_version_id = product_info->software_version_id;
  } else {
    device_info->model_id = -1;
    device_info->product_category = -1;
    device_info->software_version_id = -1;
  }
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
  device_info->sub_device_count = dmx_sub_device_get_count(dmx_num);
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
  char init_value[RDM_ASCII_SIZE_MAX];
  strncpy(init_value, device_label, 32);
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, init_value,
                         sizeof(init_value))) {
    return false;
  }

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
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
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
                                         const char *software_version_label,
                                         rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  if (dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT,
                        RDM_PID_SOFTWARE_VERSION_LABEL) == NULL) {
    DMX_CHECK(software_version_label != NULL, false,
              "software_version_label is null");
    DMX_CHECK(strnlen(software_version_label, RDM_ASCII_SIZE_MAX) <
                  RDM_ASCII_SIZE_MAX,
              false, "software_version_label error");
  }

  const rdm_pid_t pid = RDM_PID_SOFTWARE_VERSION_LABEL;

  // Add the parameter as a non-const, dynamic variable
  char pd[RDM_ASCII_SIZE_MAX];
  strncpy(pd, software_version_label, RDM_ASCII_SIZE_MAX);
  const size_t size = strnlen(software_version_label, RDM_ASCII_SIZE_MAX);
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_DYNAMIC, pd, size)) {
    return false;
  }

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
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
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

bool rdm_register_manufacturer_label(dmx_port_t dmx_num,
                                     char *manufacturer_label,
                                     rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  if (dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT,
                        RDM_PID_SOFTWARE_VERSION_LABEL) == NULL) {
    DMX_CHECK(manufacturer_label != NULL, false, "manufacturer_label is null");
    DMX_CHECK(
        strnlen(manufacturer_label, RDM_ASCII_SIZE_MAX) < RDM_ASCII_SIZE_MAX,
        false, "manufacturer_label error");
  }

  const rdm_pid_t pid = RDM_PID_MANUFACTURER_LABEL;

  // Add the parameter as a static variable
  const size_t size = strnlen(manufacturer_label, RDM_ASCII_SIZE_MAX);
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_STATIC, manufacturer_label, size)) {
    return false;
  }

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
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_manufacturer_label(dmx_port_t dmx_num, char *manufacturer_label,
                                  size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(manufacturer_label != NULL, 0, "manufacturer_label is null");
  DMX_CHECK(size > 0, 0, "size error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT,
                            RDM_PID_MANUFACTURER_LABEL, manufacturer_label,
                            size);
}

bool rdm_register_device_model_description(dmx_port_t dmx_num,
                                           const char *device_model_description,
                                           rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  if (dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT,
                        RDM_PID_DEVICE_MODEL_DESCRIPTION) == NULL) {
    DMX_CHECK(device_model_description != NULL, false,
              "device_model_description is null");
    DMX_CHECK(strnlen(device_model_description, RDM_ASCII_SIZE_MAX) <
                  RDM_ASCII_SIZE_MAX,
              false, "device_model_description error");
  }

  const rdm_pid_t pid = RDM_PID_DEVICE_MODEL_DESCRIPTION;

  // Add the parameter as a non-const, dynamic variable
  char pd[RDM_ASCII_SIZE_MAX];
  strncpy(pd, device_model_description, RDM_ASCII_SIZE_MAX);
  const size_t size = strnlen(device_model_description, RDM_ASCII_SIZE_MAX);
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_DYNAMIC, pd, size)) {
    return false;
  }

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
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_device_model_description(dmx_port_t dmx_num,
                                        char *device_model_description,
                                        size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_model_description != NULL, 0,
            "device_model_description is null");
  DMX_CHECK(size > 0, 0, "size error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT,
                            RDM_PID_DEVICE_MODEL_DESCRIPTION,
                            device_model_description, size);
}

bool rdm_register_language(dmx_port_t dmx_num, const char *language,
                           rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  if (dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_LANGUAGE) ==
      NULL) {
    DMX_CHECK(language != NULL, false, "language is null");
    DMX_CHECK(strnlen(language, 3) != 2, false, "language error");
  }

  const rdm_pid_t pid = RDM_PID_LANGUAGE;

  // Add the parameter as a non-const, dynamic variable
  char pd[3];
  memcpy(pd, language, 2);
  pd[2] = '\0';
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_DYNAMIC, pd, sizeof(pd))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET_SET,
      .ds = RDM_DS_ASCII,
      .get = {.handler = rdm_rhd_get_set_language,
              .request.format = NULL,
              .response.format = "a$"},
      .set = {.handler = rdm_rhd_get_set_language,
              .request.format = "a$",
              .response.format = NULL},
      .pdl_size = 2,
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_language(dmx_port_t dmx_num, char *language) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(language != NULL, 0, "language is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_LANGUAGE,
                            language, 3);
}

bool rdm_set_language(dmx_port_t dmx_num, const char *language) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(strnlen(language, 3) != 2, false, "language error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Null-terminate the language string in case it isn't
  char pd[3];
  strncpy(pd, language, 2);
  pd[2] = '\0';

  const rdm_pid_t pid = RDM_PID_DEVICE_LABEL;
  if (!dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, pd, sizeof(pd))) {
    return false;
  }
  rdm_queue_push(dmx_num, pid);

  return true;
}