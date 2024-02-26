#include "rdm/responder/include/device_control.h"

#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/responder/include/utils.h"

static size_t rdm_rhd_set_reset_device(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  // Verify the reset state is valid
  uint8_t reset;
  if (!rdm_read_pd(dmx_num, definition->set.request.format, &reset,
                   sizeof(reset))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }
  if (reset != RDM_RESET_TYPE_WARM && reset != RDM_RESET_TYPE_COLD) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Set the mute type
  dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, header->pid, &reset,
                    sizeof(reset));

  // This parameter clears the mute flag
  const uint8_t mute = 0;
  dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DISC_MUTE, &mute,
                    sizeof(mute));

  return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
}

bool rdm_register_identify_device(dmx_port_t dmx_num, rdm_callback_t cb,
                                  void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(cb != NULL, false, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;

  // Allocate parameter data
  uint8_t init_value = 0;
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, &init_value,
                         sizeof(init_value))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET_SET,
      .ds = RDM_DS_UNSIGNED_BYTE,
      .get = {.handler = rdm_simple_response_handler,
              .request.format = NULL,
              .response.format = "b$"},
      .set = {.handler = rdm_simple_response_handler,
              .request.format = "b$",
              .response.format = NULL},
      .pdl_size = sizeof(uint8_t),
      .max_value = 1,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_identify_device(dmx_port_t dmx_num, bool *identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify != NULL, 0, "identify is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT,
                            RDM_PID_IDENTIFY_DEVICE, identify, sizeof(uint8_t));
}

bool rdm_set_identify_device(dmx_port_t dmx_num, const bool identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify == 0 || identify == 1, 0, "identify error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;
  if (!dmx_parameter_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &identify,
                         sizeof(uint8_t))) {
    return false;
  }
  rdm_queue_push(dmx_num, pid);

  return true;
}

bool rdm_register_reset_device(dmx_port_t dmx_num, rdm_callback_t cb,
                               void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(cb != NULL, false, "cb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_RESET_DEVICE;

  // Allocate parameter data
  uint8_t init_value = RDM_RESET_TYPE_NONE;
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, &init_value,
                         sizeof(init_value))) {
    return false;
  }

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_SET,
      .ds = RDM_DS_UNSIGNED_BYTE,
      .get = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .set = {.handler = rdm_rhd_set_reset_device,
              .request.format = "b$",
              .response.format = NULL},
      .pdl_size = sizeof(uint8_t),
      .max_value = 0,
      .min_value = 0,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

size_t rdm_get_reset_device(dmx_port_t dmx_num, uint8_t *reset) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(reset != NULL, 0, "reset is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_parameter_copy(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_RESET_DEVICE,
                            reset, sizeof(*reset));
}
