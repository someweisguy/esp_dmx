#include "rdm_info.h"

#include "dmx/driver.h"
#include "dmx/struct.h"
#include "rdm/utils/bus_ctl.h"
#include "endian.h"

static int rdm_supported_params_response_cb(dmx_port_t dmx_num,
                                            rdm_header_t *header, void *pd,
                                            uint8_t *pdl_out,
                                            const char *format) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  if (header->cc != RDM_CC_GET_COMMAND) {
    // The supported params list is read-only
    *pdl_out = rdm_emplace_word(pd, RDM_NR_UNSUPPORTED_COMMAND_CLASS);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Copy all PIDs into a temporary buffer
  uint16_t pids[RDM_RESPONDER_NUM_PIDS_MAX];
  const uint32_t num_pids = rdm_pd_list(dmx_num, header->sub_device, pids,
                                        RDM_RESPONDER_NUM_PIDS_MAX);

  // Emplace the PIDs into the parameter data
  for (int i = 0; i < num_pids && *pdl_out <= 231; ++i) {
    switch (pids[i]) {
      // Minimum required PIDs are not included
      case RDM_PID_DISC_UNIQUE_BRANCH:
      case RDM_PID_DISC_MUTE:
      case RDM_PID_DISC_UN_MUTE:
      case RDM_PID_SUPPORTED_PARAMETERS:
      case RDM_PID_PARAMETER_DESCRIPTION:
      case RDM_PID_DEVICE_INFO:
      case RDM_PID_SOFTWARE_VERSION_LABEL:
      case RDM_PID_DMX_START_ADDRESS:
      case RDM_PID_IDENTIFY_DEVICE:
        continue;
      default:
        *pdl_out += rdm_emplace_word(pd, pids[i]);
        pd += sizeof(uint16_t);
    }
  }

  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_supported_parameters(dmx_port_t dmx_num, rdm_callback_t cb,
                                       void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // const rdm_pid_description_t description = {.pid = RDM_PID_SUPPORTED_PARAMETERS,
  //                                     .pdl_size = 0xe6,
  //                                     .data_type = RDM_DS_UNSIGNED_WORD,
  //                                     .cc = RDM_CC_GET,
  //                                     .description = "Supported Parameters"};
  // return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &description, NULL,
  //                        rdm_supported_params_response_cb, NULL, NULL, NULL,
  //                        false);

  // FIXME
  return false;
}

static int rdm_parameter_description_response_cb(dmx_port_t dmx_num,
                                                 rdm_header_t *header, void *pd,
                                                 uint8_t *pdl_out,
                                                 const char *format) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT)
  {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  uint16_t requestedPid;
  memcpy(&requestedPid, pd, sizeof(uint16_t)); // need to memcpy to avoid undefined behavior (strict aliasing rule)
  requestedPid = bswap16(requestedPid);

  // 0x8000 to 0xFFDF is the allowed range for manufacturer specific pids
  if (requestedPid < RDM_PID_MANUFACTURER_SPECIFIC_BEGIN || requestedPid > RDM_PID_MANUFACTURER_SPECIFIC_END)
  {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Iterate the callback list to see if a callback with this PID exists
  for (int i = 0; i < driver->num_parameters; ++i)
  {
    if (driver->params[i].definition.pid == requestedPid)
    {
      //The pdl can be in range x014-0x34 depending on how long the parameter description string is.
      //There is no harm in always sending the full string, so we just do that.
      *pdl_out = rdm_emplace(pd, format, &driver->params[i].definition, 0x34, false);
      return RDM_RESPONSE_TYPE_ACK;
    }
  }

  // no pid found
  *pdl_out = rdm_emplace_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
  return RDM_RESPONSE_TYPE_NACK_REASON;
}

bool rdm_register_parameter_description(dmx_port_t dmx_num, rdm_callback_t cb,
                                        void *context)
{
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT) != NULL,
      false, "RDM_PID_DEVICE_INFO must be registered first");

  const rdm_pid_description_t pd_def = {
      .pid = RDM_PID_PARAMETER_DESCRIPTION,
      .pdl_size =
          0x34,  // this is the max size, not necessarily the one we send
      .data_type = RDM_DS_UNSIGNED_BYTE,  // not really true but there is no
                                          // data type for complex struct
      .cc = RDM_CC_GET,
      .description = "Parameter Description"};
  const char *format = "wbbbbbbddda$";

  rdm_pd_add_deterministic(dmx_num, RDM_SUB_DEVICE_ROOT, &pd_def, format,
                           rdm_parameter_description_response_cb);
  return rdm_pd_update_callback(dmx_num, RDM_SUB_DEVICE_ROOT,
                                RDM_PID_PARAMETER_DESCRIPTION, cb, context);
}