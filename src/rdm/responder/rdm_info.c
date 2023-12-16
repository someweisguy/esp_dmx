#include "include/rdm_info.h"

#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "endian.h"
#include "rdm/responder/include/utils.h"
#include "dmx/include/io.h"

static size_t rdm_rhd_get_supported_parameters(
    dmx_port_t dmx_num, const rdm_pd_definition_t *definition,
    const rdm_header_t *header) {
  int pid_count = 0;
  uint16_t pids[115];

  // Handle situation where parameters overflowed last request
  int i = 0; // TODO

  for (; i < 115; ++i) {
    uint16_t pid = rdm_parameter_at(dmx_num, header->sub_device, i);
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
    dmx_port_t dmx_num, const rdm_pd_definition_t *definition,
    const rdm_header_t *header) {
  // This request may only be sent to the root device
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    return rdm_write_nack_reason(dmx_num, header,
                                 RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  }

  // Read the request data
  uint16_t pid;
  if (!rdm_read_pd(dmx_num, definition->get. request.format, &pid,
                   sizeof(pid))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_HARDWARE_FAULT);
  }

  // Ensure the request PID is within bounds
  if (pid < definition->min_value || pid > definition->max_value) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Ensure the request PID is known
  const rdm_pd_definition_t *requested_definition = rdm_parameter_lookup(pid);
  if (requested_definition == NULL) {
    // Don't return RDM_NR_UNKNOWN_PID because it has a different meaning
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  rdm_pid_description_t pd;
  pd.pid = pid;
  pd.pdl_size = requested_definition->pdl_size;
  pd.data_type = requested_definition->ds;
  pd.cc = requested_definition->pid_cc;
  pd.unit = requested_definition->units;
  pd.prefix = requested_definition->prefix;
  pd.min_value = requested_definition->min_value;
  pd.max_value = requested_definition->max_value;
  pd.default_value = requested_definition->default_value;
  strncpy(pd.description, requested_definition->description, 32);

  return rdm_write_ack(dmx_num, header, definition->get.response.format,
                       &pd, sizeof(pd));
}

bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_callback_t cb,
                                       void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_SUPPORTED_PARAMETERS;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
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
  rdm_parameter_define(&definition);

  // Add the parameter as a NULL static variable
  const bool nvs = false;
  rdm_parameter_add_static(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, NULL, 0);

  return true;
}

bool rdm_register_parameter_description(dmx_port_t dmx_num, rdm_callback_t cb,
                                        void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_PARAMETER_DESCRIPTION;
  static const rdm_pd_definition_t definition = {
      .pid = pid,
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
  rdm_parameter_define(&definition);

  // Add the parameter as a NULL static variable
  const bool nvs = false;
  rdm_parameter_add_static(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, NULL, 0);

  return true;
}