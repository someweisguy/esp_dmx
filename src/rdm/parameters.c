#include "parameters.h"

#include "dmx/device.h"
#include "dmx/driver.h"
#include "dmx/struct.h"

bool rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_info != NULL, 0, "device_info is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Get the parameter and copy it to the user's pointer
  const rdm_device_info_t *di = rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, 0);
  DMX_CHECK(di != NULL, false, "device_info not registered");
  memcpy(device_info, di, sizeof(rdm_device_info_t));
  
  return true;
}

bool rdm_get_software_version_label(dmx_port_t dmx_num,
                                    char *software_version_label,
                                    size_t *size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(software_version_label != NULL, 0,
            "software_version_label is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Get the parameter and copy it to the user's pointer
  const char *svl = rdm_pd_get(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL, 0);
  DMX_CHECK(svl != NULL, false, "software_version_label not registered");
  *size = strnlen(svl, *size);  // Prevent buffer overflows
  strncpy(software_version_label, svl, *size);

  return true;
}

bool rdm_get_identify_device(dmx_port_t dmx_num, uint8_t *identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify != NULL, 0, "identify is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Get the parameter and copy it to the user's pointer
  const uint8_t *id = rdm_pd_get(dmx_num, RDM_PID_IDENTIFY_DEVICE, 0);
  DMX_CHECK(id != NULL, false, "identify_device not registered");
  *identify = *id;

  return true;
}

bool rdm_set_identify_device(dmx_port_t dmx_num, const uint8_t identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify == 0 || identify == 1, 0, "identify error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_pd_set(dmx_num, RDM_PID_IDENTIFY_DEVICE, 0, &identify,
                    sizeof(uint8_t));
}

bool rdm_get_dmx_start_address(dmx_port_t dmx_num,
                               uint16_t *dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address != NULL, 0, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // DMX getter can be called
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

size_t rdm_get_device_label(dmx_port_t dmx_num, char *label, size_t label_len) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const char *rdm_label = rdm_pd_get(dmx_num, RDM_PID_DEVICE_LABEL, 0);
  DMX_CHECK(rdm_label != NULL, 0, "RDM_PID_DEVICE_LABEL not found");

  const size_t rdm_label_len = strnlen(rdm_label, 32); //length without '\0'
  const size_t size = label_len < rdm_label_len? label_len : rdm_label_len;
  strncpy(label, rdm_label, size);
  
  return size;
}