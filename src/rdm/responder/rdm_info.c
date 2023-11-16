#include "rdm_info.h"

#include "dmx/driver.h"
#include "dmx/struct.h"
#include "endian.h"
#include "rdm/utils/bus_ctl.h"

static int rdm_rhd_supported_parameters(dmx_port_t dmx_num,
                                        rdm_header_t *header, void *pd,
                                        uint8_t *pdl_out,
                                        const rdm_pd_schema_t *schema) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Copy all PIDs into a temporary buffer
  uint16_t pids[RDM_RESPONDER_NUM_PIDS_MAX];
  const uint32_t num_pids = rdm_pd_list(dmx_num, header->sub_device, pids,
                                        RDM_RESPONDER_NUM_PIDS_MAX);
  if (num_pids == 0) {
    *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_HARDWARE_FAULT);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Emplace the PIDs into the parameter data
  for (int i = 0; i < num_pids && *pdl_out < 230; ++i) {
    switch (pids[i]) {
      case RDM_PID_DISC_UNIQUE_BRANCH:
      case RDM_PID_DISC_MUTE:
      case RDM_PID_DISC_UN_MUTE:
      case RDM_PID_SUPPORTED_PARAMETERS:
      case RDM_PID_PARAMETER_DESCRIPTION:
      case RDM_PID_DEVICE_INFO:
      case RDM_PID_SOFTWARE_VERSION_LABEL:
      case RDM_PID_DMX_START_ADDRESS:
      case RDM_PID_IDENTIFY_DEVICE:
        // Minimum required PIDs are not reported
        continue;
      default:
        *pdl_out += rdm_pd_serialize_word(pd, pids[i]);
        pd += sizeof(uint16_t);
    }
  }

  return RDM_RESPONSE_TYPE_ACK;
}

static int rdm_rhd_parameter_description(dmx_port_t dmx_num,
                                         rdm_header_t *header, void *pd,
                                         uint8_t *pdl_out,
                                         const rdm_pd_schema_t *schema) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Get the requested PID and avoid undefined behavior (strict aliasing rule)
  uint16_t requested_pid;
  rdm_pd_deserialize(&requested_pid, sizeof(requested_pid), "w$", pd);

  // Get the PID description - fails if no PID or description found
  rdm_pid_description_t description;
  if (!rdm_pd_get_description(dmx_num, requested_pid, header->sub_device,
                              &description)) {
    *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }
  
  *pdl_out = rdm_pd_serialize(pd, 231, schema->format, &description);
  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_callback_t cb,
                                       void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_SUPPORTED_PARAMETERS;
  const rdm_pd_definition_t def = {
      .schema = {.data_type = RDM_DS_UNSIGNED_WORD,
                 .cc = RDM_CC_GET,
                 .pdl_size = 0,
                 .alloc_size = 0,  // Parameter is deterministic
                 .format = "w"},
      .nvs = false,
      .response_handler = rdm_rhd_supported_parameters,
  };

  rdm_pd_add_deterministic(dmx_num, pid, RDM_SUB_DEVICE_ROOT, &def);
  return rdm_pd_update_callback(dmx_num, pid, RDM_SUB_DEVICE_ROOT, cb, context);
}

bool rdm_register_parameter_description(dmx_port_t dmx_num, rdm_callback_t cb,
                                        void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // Define the parameter
  const rdm_pid_t pid = RDM_PID_PARAMETER_DESCRIPTION;
  const rdm_pd_definition_t def = {
      .schema = {.data_type = RDM_DS_BIT_FIELD,
                 .cc = RDM_CC_GET,
                 .pdl_size = sizeof(uint16_t),
                 .alloc_size = 0,  // Parameter is deterministic
                 .format = "wbbb#00hbbddda$"},
      .nvs = false,
      .response_handler = rdm_rhd_parameter_description,
  };

  rdm_pd_add_deterministic(dmx_num, pid, RDM_SUB_DEVICE_ROOT, &def);
  return rdm_pd_update_callback(dmx_num, pid, RDM_SUB_DEVICE_ROOT, cb, context);
}