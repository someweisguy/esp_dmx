#include "include/product_info.h"

#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/controller/include/utils.h"
#include "rdm/include/driver.h"
#include "rdm/include/uid.h"

size_t rdm_send_get_device_info(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                                rdm_sub_device_t sub_device,
                                rdm_device_info_t *device_info,
                                rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dest_uid != NULL, 0, "dest_uid is null");
  DMX_CHECK(!rdm_uid_is_broadcast(dest_uid), 0, "dest_uid error");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX, 0, "sub_device error");
  DMX_CHECK(device_info != NULL, 0, "device_info is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_cc_t cc = RDM_CC_GET_COMMAND;
  const rdm_pid_t pid = RDM_PID_DEVICE_INFO;
  size_t pdl = rdm_send_request(dmx_num, dest_uid, sub_device, pid, cc, NULL,
                                NULL, 0, ack);
  if (pdl == sizeof(*device_info)) {
    const char *format = "x01x00wwdwbbwwb$";
    rdm_read_pd(dmx_num, format, device_info, sizeof(*device_info));
  }
  return pdl;
}

size_t rdm_send_get_software_version_label(dmx_port_t dmx_num,
                                           const rdm_uid_t *dest_uid,
                                           rdm_sub_device_t sub_device,
                                           char *software_version_label,
                                           size_t size, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dest_uid != NULL, 0, "dest_uid is null");
  DMX_CHECK(sub_device < RDM_SUB_DEVICE_MAX || sub_device == RDM_SUB_DEVICE_ALL,
            0, "sub_device error");
  DMX_CHECK(software_version_label != NULL, 0,
            "software_version_label is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_cc_t cc = RDM_CC_GET_COMMAND;
  const rdm_pid_t pid = RDM_PID_SOFTWARE_VERSION_LABEL;
  size_t pdl = rdm_send_request(dmx_num, dest_uid, sub_device, pid, cc, NULL,
                                NULL, 0, ack);
  if (pdl > 0) {
    const char *format = "a$";
    rdm_read_pd(dmx_num, format, software_version_label, size);
  }
  return pdl;
}
