#include "pids.h"

#include "rdm/utils.h"

static bool rdm_get_generic(dmx_port_t dmx_num, rdm_pid_t pid, void *param,
                            size_t size) {
  const rdm_pid_description_t *desc;
  void *pd = rdm_get_pid(dmx_num, pid, &desc);

  bool ret;
  if (pd != NULL) {
    // TODO: check param bounds
    size = size < desc->pdl_size ? size : desc->pdl_size;
    if (desc->data_type == RDM_DS_ASCII) {
      strncpy(param, pd, size);
    } else {
      memcpy(param, pd, size);
    }
    ret = true;
  } else {
    ret = false;
  }

  // TODO: add queued message

  return ret;
}

static bool rdm_set_generic(dmx_port_t dmx_num, rdm_pid_t pid,
                            const void *param, size_t size, bool nvs) {
  const rdm_pid_description_t *desc;
  void *pd = rdm_get_pid(dmx_num, pid, &desc);

  bool ret;
  if (pd != NULL) {
    size = size < desc->pdl_size ? size : desc->pdl_size;
    if (desc->data_type == RDM_DS_ASCII) {
      strncpy(pd, param, size);
    } else {
      memcpy(pd, param, size);
    }
    if (nvs) {
      esp_err_t err =
          rdm_set_pid_to_nvs(dmx_num, pid, desc->data_type, param, size);
      if (err) {
        // TODO: set boot-loader flag
      }
    }
    ret = true;
  } else {
    ret = false;
  }

  return ret;
}

bool rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  // TODO: arg checks

  return rdm_get_generic(dmx_num, RDM_PID_DEVICE_INFO, device_info,
                         sizeof(*device_info));
}

bool rdm_get_software_version_label(dmx_port_t dmx_num,
                                    char *software_version_label, size_t size) {
  // TODO: arg check

  return rdm_get_generic(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL,
                         software_version_label, size);
}

bool rdm_get_identify_device(dmx_port_t dmx_num, uint8_t *identify) {
  // TODO: arg check

  return rdm_get_generic(dmx_num, RDM_PID_IDENTIFY_DEVICE, identify,
                         sizeof(*identify));
}

bool rdm_set_identify_device(dmx_port_t dmx_num, const uint8_t identify) {
  // TODO: arg check

  return rdm_set_generic(dmx_num, RDM_PID_IDENTIFY_DEVICE, &identify,
                         sizeof(identify), false);
}

bool rdm_get_dmx_start_address(dmx_port_t dmx_num,
                               uint16_t *dmx_start_address) {
  // TODO: arg check

  return rdm_get_generic(dmx_num, RDM_PID_DMX_START_ADDRESS, dmx_start_address,
                         sizeof(*dmx_start_address));
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address) {
  // TODO: arg check

  return rdm_set_generic(dmx_num, RDM_PID_DMX_START_ADDRESS, &dmx_start_address,
                         sizeof(dmx_start_address), true);
}