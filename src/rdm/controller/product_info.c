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

  const rdm_request_t request = {
    .dest_uid = dest_uid,
    .sub_device = sub_device,
    .cc = RDM_CC_GET_COMMAND,
    .pid = RDM_PID_DEVICE_INFO
  };

  const char *format = "x01x00wwdwbbwwb$";
  return rdm_send_request(dmx_num, &request, format, device_info,
                          sizeof(*device_info), ack);
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

  const rdm_request_t request = {.dest_uid = dest_uid,
                                 .sub_device = sub_device,
                                 .cc = RDM_CC_GET_COMMAND,
                                 .pid = RDM_PID_SOFTWARE_VERSION_LABEL};

  const char *format = "a$";
  return rdm_send_request(dmx_num, &request, format, software_version_label,
                          size, ack);
}
