#include "pids.h"

#include "rdm/utils.h"

bool rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  // TODO: arg checks

  return rdm_get_parameter(dmx_num, RDM_PID_DEVICE_INFO, device_info,
                           sizeof(*device_info));
}

bool rdm_get_software_version_label(dmx_port_t dmx_num,
                                    char *software_version_label, size_t size) {
  // TODO: arg check

  return rdm_get_parameter(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL,
                           software_version_label, size);
}

bool rdm_get_identify_device(dmx_port_t dmx_num, uint8_t *identify) {
  // TODO: arg check

  return rdm_get_parameter(dmx_num, RDM_PID_IDENTIFY_DEVICE, identify,
                           sizeof(*identify));
}

bool rdm_set_identify_device(dmx_port_t dmx_num, const uint8_t identify) {
  // TODO: arg check

  return rdm_set_parameter(dmx_num, RDM_PID_IDENTIFY_DEVICE, &identify,
                           sizeof(identify), false);
}

bool rdm_get_dmx_start_address(dmx_port_t dmx_num,
                               uint16_t *dmx_start_address) {
  // TODO: arg check

  return rdm_get_parameter(dmx_num, RDM_PID_DMX_START_ADDRESS,
                           dmx_start_address, sizeof(*dmx_start_address));
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address) {
  // TODO: arg check

  return rdm_set_parameter(dmx_num, RDM_PID_DMX_START_ADDRESS,
                           &dmx_start_address, sizeof(dmx_start_address), true);
}