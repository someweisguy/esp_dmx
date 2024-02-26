#include "include/rdm_info.h"

#include <string.h>

#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/include/driver.h"
#include "rdm/responder/include/utils.h"

static size_t rdm_rhd_get_supported_parameters(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  int pid_count = 0;
  uint16_t pids[115];

  // Handle situation where parameters overflowed last request
  int i = 0;  // TODO: handle ACK overflow

  for (; i < 115; ++i) {
    uint16_t pid = dmx_parameter_at(dmx_num, header->sub_device, i);
    if (pid == 0) {
      break;
    }
    switch (pid) {
      case RDM_PID_DISC_UNIQUE_BRANCH:
      case RDM_PID_DISC_MUTE:
      case RDM_PID_DISC_UN_MUTE:
      case RDM_PID_SUPPORTED_PARAMETERS:
      case RDM_PID_PARAMETER_DESCRIPTION:
      case RDM_PID_DEVICE_INFO:
      case RDM_PID_SOFTWARE_VERSION_LABEL:
      case RDM_PID_DMX_START_ADDRESS:
      case RDM_PID_IDENTIFY_DEVICE:
        continue;  // Minimum required PIDs are not reported
    }
    pids[pid_count] = pid;
    ++pid_count;
  }

  const size_t pdl = pid_count * sizeof(uint16_t);
  return rdm_write_ack(dmx_num, header, definition->get.response.format, pids,
                       pdl);
}

static size_t rdm_rhd_get_parameter_description(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  // This request may only be sent to the root device
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    return rdm_write_nack_reason(dmx_num, header,
                                 RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  }

  // Read the request data
  uint16_t pid;
  if (!rdm_read_pd(dmx_num, definition->get.request.format, &pid,
                   sizeof(pid))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }

  // Ensure the request PID is within bounds
  if (pid < definition->min_value || pid > definition->max_value) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Ensure the request PID is known
  const rdm_parameter_definition_t *requested_definition =
      rdm_definition_get(dmx_num, header->sub_device, pid);
  if (requested_definition == NULL) {
    // Don't return RDM_NR_UNKNOWN_PID because it has a different meaning
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  rdm_parameter_description_t pd;
  pd.pid = pid;
  pd.pdl_size = requested_definition->pdl_size;
  pd.data_type = requested_definition->ds;
  pd.cc = requested_definition->pid_cc;
  pd.unit = requested_definition->units;
  pd.prefix = requested_definition->prefix;
  pd.min_value = requested_definition->min_value;
  pd.max_value = requested_definition->max_value;
  pd.default_value = requested_definition->default_value;
  strncpy(pd.description, requested_definition->description,
          RDM_ASCII_SIZE_MAX);

  return rdm_write_ack(dmx_num, header, definition->get.response.format, &pd,
                       sizeof(pd));
}

bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_callback_t cb,
                                       void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_SUPPORTED_PARAMETERS;

  // Add the parameter as NULL static
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_STATIC, NULL, 0)) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_UNSIGNED_WORD,
      .get = {.handler = rdm_rhd_get_supported_parameters,
              .request.format = NULL,
              .response.format = "w"},
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

size_t rdm_get_supported_parameters(dmx_port_t dmx_num, uint16_t *pids,
                                    size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  int i = 0;
  uint32_t pid_count = 0;

  // Write PIDs to the destination buffer
  if (pids == NULL) {
    size = 0;  // Guard against null pointer writes
  } else if (size % sizeof(uint16_t) != 0) {
    size -= 1;
  }
  for (; size > 0; size -= sizeof(uint16_t)) {
    uint16_t pid = dmx_parameter_at(dmx_num, RDM_SUB_DEVICE_ROOT, i);
    if (pid == 0) {
      break;
    }
    switch (pid) {
      case RDM_PID_DISC_UNIQUE_BRANCH:
      case RDM_PID_DISC_MUTE:
      case RDM_PID_DISC_UN_MUTE:
      case RDM_PID_SUPPORTED_PARAMETERS:
      case RDM_PID_PARAMETER_DESCRIPTION:
      case RDM_PID_DEVICE_INFO:
      case RDM_PID_SOFTWARE_VERSION_LABEL:
      case RDM_PID_DMX_START_ADDRESS:
      case RDM_PID_IDENTIFY_DEVICE:
        continue;  // Minimum required PIDs are not reported
    }
    pids[pid_count] = pid;
    ++pid_count;
    ++i;
  }
  while (dmx_parameter_at(dmx_num, RDM_SUB_DEVICE_ROOT, i) != 0) {
    ++pid_count;
    ++i;
  }

  return pid_count * sizeof(uint16_t);
}

bool rdm_register_parameter_description(dmx_port_t dmx_num, rdm_callback_t cb,
                                        void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_PARAMETER_DESCRIPTION;

  // Add the parameter as NULL static
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_STATIC, NULL, 0)) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_ASCII,
      .get = {.handler = rdm_rhd_get_parameter_description,
              .request.format = "w$",
              .response.format = "wbbbx00bbddda"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = sizeof(uint16_t),
      .max_value = RDM_PID_MANUFACTURER_SPECIFIC_END,
      .min_value = RDM_PID_MANUFACTURER_SPECIFIC_BEGIN,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_parameter_description(dmx_port_t dmx_num, rdm_pid_t pid,
                                     rdm_parameter_description_t *parameter) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(!(pid < RDM_PID_MANUFACTURER_SPECIFIC_BEGIN ||
              pid > RDM_PID_MANUFACTURER_SPECIFIC_END),
            false, "pid error");
  DMX_CHECK(parameter != NULL, false, "parameter is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Ensure the request PID is known
  const rdm_parameter_definition_t *requested_definition =
      rdm_definition_get(dmx_num, RDM_SUB_DEVICE_ROOT, pid);
  if (requested_definition == NULL) {
    return 0;
  }

  // Copy the parameter
  parameter->pid = pid;
  parameter->pdl_size = requested_definition->pdl_size;
  parameter->data_type = requested_definition->ds;
  parameter->cc = requested_definition->pid_cc;
  parameter->unit = requested_definition->units;
  parameter->prefix = requested_definition->prefix;
  parameter->min_value = requested_definition->min_value;
  parameter->max_value = requested_definition->max_value;
  parameter->default_value = requested_definition->default_value;
  strncpy(parameter->description, requested_definition->description,
          RDM_ASCII_SIZE_MAX);

  return sizeof(rdm_parameter_description_t);
}