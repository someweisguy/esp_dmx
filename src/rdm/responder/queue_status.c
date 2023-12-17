#include "include/queue_status.h"

#include "dmx/include/driver.h"
#include "dmx/include/io.h"
#include "dmx/include/struct.h"
#include "rdm/responder/include/utils.h"

static size_t rdm_rhd_get_queued_message(dmx_port_t dmx_num,
                                         const rdm_parameter_definition_t *definition,
                                         const rdm_header_t *header) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    // Requests may only be made to the root device
    return rdm_write_nack_reason(dmx_num, header,
                                 RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  }

  // Verify status type is valid
  uint8_t status_type;
  if (!rdm_read_pd(dmx_num, definition->get.request.format, &status_type,
                   sizeof(status_type))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }
  if (status_type < definition->min_value ||
      status_type > definition->max_value) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Determine what the response will be depending on the RDM queue size
  rdm_pid_t pid;
  rdm_header_t response_header = *header;
  const rdm_parameter_definition_t *response_definition;
  if (rdm_queue_size(dmx_num) > 0) {
    pid = rdm_queue_pop(dmx_num);
    response_definition = rdm_parameter_lookup(pid);
    assert(response_definition != NULL);
  } else {
    pid = RDM_PID_STATUS_MESSAGE;
    response_definition = rdm_parameter_lookup(pid);
    if (response_definition == NULL) {
      response_header.pid = RDM_PID_STATUS_MESSAGE;
      return rdm_write_ack(dmx_num, &response_header, NULL, NULL, 0);
    }
  }

  response_header.pid = pid;
  return response_definition->get.handler(dmx_num, response_definition,
                                          &response_header);
}

bool rdm_register_queued_message(dmx_port_t dmx_num, rdm_callback_t cb,
                                 void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_QUEUED_MESSAGE;
  static const rdm_parameter_definition_t definition = {
      .pid = pid,
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_get_queued_message,
              .request.format = NULL,
              .response.format = "b$"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = sizeof(uint8_t),
      .max_value = RDM_STATUS_ERROR,
      .min_value = RDM_STATUS_GET_LAST_MESSAGE,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_parameter_define(&definition);

  // Add the parameter as a NULL static variable
  const bool nvs = false;
  dmx_parameter_add_static(dmx_num, RDM_SUB_DEVICE_ROOT, pid, nvs, NULL, 0);

  return true;
}