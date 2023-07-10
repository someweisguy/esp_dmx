#include "pids.h"

#include "rdm/utils.h"

bool rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  // TODO: arg checks

  rdm_device_info_t *param = rdm_get_pid(dmx_num, RDM_PID_DEVICE_INFO);
  if (param == NULL) {
    return false;
  }
  *device_info = *param;

  return true;
}

bool rdm_get_software_version_label(dmx_port_t dmx_num,
                                    const char **software_version_label) {
  // TODO: arg check

  const char *param = rdm_get_pid(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL);
  if (param == NULL) {
    return false;
  }
  *software_version_label = param;

  return true;
}

bool rdm_get_identify_device(dmx_port_t dmx_num, uint8_t *identify) {
  // TODO: arg check

  uint8_t *param = rdm_get_pid(dmx_num, RDM_PID_IDENTIFY_DEVICE);
  if (param == NULL) {
    return false;
  }
  *identify = *param;

  return true;
}

bool rdm_set_identify_device(dmx_port_t dmx_num, const uint8_t identify) {
  // TODO: arg check

  uint8_t *param = rdm_get_pid(dmx_num, RDM_PID_IDENTIFY_DEVICE);
  if (param == NULL) {
    return false;
  }
  *param = identify;

  return true;
}

bool rdm_get_dmx_start_address(dmx_port_t dmx_num, uint16_t *dmx_start_address) {
  // TODO: arg check

  uint16_t *param = rdm_get_pid(dmx_num, RDM_PID_DMX_START_ADDRESS);
  if (param == NULL) {
    return false;
  }
  *dmx_start_address = *param;

  return true;
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address) {
  // TODO: arg check

  uint16_t *param = rdm_get_pid(dmx_num, RDM_PID_DMX_START_ADDRESS);
  if (param == NULL) {
    return false;
  }
  *param = dmx_start_address;

  esp_err_t err = rdm_set_nvs(dmx_num, RDM_PID_DMX_START_ADDRESS,
                              RDM_DS_UNSIGNED_WORD, param, 2);
  if (err) {
    // TODO: set boot-loader flag
  }

  // TODO: add queued message

  return true;
}