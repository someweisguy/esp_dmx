#include "include/rdm_info.h"

#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "endian.h"
#include "rdm/responder/include/utils.h"

static size_t rdm_rhd_get_supported_parameters(
    dmx_port_t dmx_num, const rdm_pd_definition_t *definition,
    const rdm_header_t *header) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    return rdm_write_nack_reason(dmx_num, header,
                                 RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  }

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

  return rdm_write_ack(dmx_num, header, definition->get.response.format, pids,
                       pid_count * sizeof(uint16_t));
}

// static int rdm_rhd_parameter_description(dmx_port_t dmx_num,
//                                          rdm_header_t *header, void *pd,
//                                          uint8_t *pdl_out,
//                                          const rdm_pd_schema_t *schema) {
//   if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
//     *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
//     return RDM_RESPONSE_TYPE_NACK_REASON;
//   }

//   // Get the requested PID and avoid undefined behavior (strict aliasing rule)
//   uint16_t requested_pid;
//   rdm_pd_deserialize(&requested_pid, sizeof(requested_pid), "w$", pd);

//   // Get the PID description - fails if no PID or description found
//   rdm_pid_description_t description;
//   if (!rdm_pd_get_description(dmx_num, header->sub_device, requested_pid,
//                               &description)) {
//     *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
//     return RDM_RESPONSE_TYPE_NACK_REASON;
//   }

//   *pdl_out = rdm_pd_serialize(pd, 231, schema->format, &description);
//   return RDM_RESPONSE_TYPE_ACK;
// }

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

  // // Define the parameter
  // const rdm_pid_t pid = RDM_PID_PARAMETER_DESCRIPTION;
  // const rdm_pd_definition_t def = {
  //     .schema = {.data_type = RDM_DS_BIT_FIELD,
  //                .cc = RDM_CC_GET,
  //                .pdl_size = sizeof(uint16_t),
  //                .alloc_size = 0,  // Parameter is deterministic
  //                .format = "wbbb#00hbbddda$"},
  //     .nvs = false,
  //     .response_handler = rdm_rhd_parameter_description,
  // };

  // rdm_pd_add_deterministic(dmx_num, pid, RDM_SUB_DEVICE_ROOT, &def,
  //                          rdm_pd_get_description);
  // return rdm_pd_update_callback(dmx_num, pid, RDM_SUB_DEVICE_ROOT, cb,
  // context);
  return false;
}