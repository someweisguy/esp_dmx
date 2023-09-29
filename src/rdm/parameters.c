#include "parameters.h"

#include "dmx/config.h"
#include "dmx/hal.h"
#include "dmx/struct.h"
#include "esp_dmx.h"
#include "rdm_utils.h"

bool rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_info != NULL, 0, "device_info is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t size = sizeof(*device_info);
  return rdm_get_parameter(dmx_num, RDM_PID_DEVICE_INFO, device_info, &size);
}

bool rdm_get_software_version_label(dmx_port_t dmx_num,
                                    char *software_version_label,
                                    size_t *size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(software_version_label != NULL, 0,
            "software_version_label is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_get_parameter(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL,
                           software_version_label, size);
}

bool rdm_get_identify_device(dmx_port_t dmx_num, uint8_t *identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify != NULL, 0, "identify is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t size = sizeof(*identify);
  return rdm_get_parameter(dmx_num, RDM_PID_IDENTIFY_DEVICE, identify, &size);
}

bool rdm_set_identify_device(dmx_port_t dmx_num, const uint8_t identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify == 0 || identify == 1, 0, "identify error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_set_parameter(dmx_num, RDM_PID_IDENTIFY_DEVICE, &identify,
                           sizeof(identify), RDM_PARAMETER_FLAG_QUEUE);
}

bool rdm_get_dmx_start_address(dmx_port_t dmx_num,
                               uint16_t *dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address != NULL, 0, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // DMX getter can be called and should always return true
  *dmx_start_address = dmx_get_start_address(dmx_num);
  return (dmx_start_address != 0);
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address > 0 && dmx_start_address < DMX_PACKET_SIZE_MAX, 0,
            "dmx_start_address error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // DMX setter can be called
  return dmx_set_start_address(dmx_num, dmx_start_address);
}

size_t rdm_get_device_label(dmx_port_t dmx_num, char * label, size_t label_len)
{
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const char *rdm_label = rdm_pd_find(dmx_num, RDM_PID_DEVICE_LABEL); 
  DMX_CHECK(rdm_label != NULL, 0, "RDM_PID_DEVICE_LABEL not found");

  const size_t rdm_label_len = strnlen(rdm_label, 32); //length without '\0'
  const size_t size = label_len < rdm_label_len? label_len : rdm_label_len;
  strncpy(label, rdm_label, size);
  
  return size;
}