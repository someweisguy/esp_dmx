#include "include/device_control.h"

#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "rdm/utils/include/io.h"
#include "rdm/utils/include/uid.h"

size_t rdm_send_get_identify_device(dmx_port_t dmx_num,
                                    const rdm_uid_t *dest_uid,
                                    rdm_sub_device_t sub_device, bool *identify,
                                    rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dest_uid != NULL, 0, "dest_uid is null");
  DMX_CHECK(identify != NULL, 0, "identify is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_cc_t cc = RDM_CC_GET_COMMAND;
  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;
  size_t pdl = rdm_send_generic(dmx_num, dest_uid, sub_device, pid, cc, NULL,
                                NULL, 0, ack);
  if (pdl == sizeof(uint8_t)) {
    const char *format = "b$";
    rdm_read_pd(dmx_num, format, identify, sizeof(uint8_t));
  }
  return pdl;
}

bool rdm_send_set_identify_device(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                                  rdm_sub_device_t sub_device,
                                  const uint8_t identify, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dest_uid != NULL, 0, "dest_uid is null");
  DMX_CHECK(identify == 0 || identify == 1, 0, "identify is invalid");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const char *format = "b$";
  const rdm_cc_t cc = RDM_CC_SET_COMMAND;
  const rdm_pid_t pid = RDM_PID_IDENTIFY_DEVICE;
  rdm_send_generic(dmx_num, dest_uid, sub_device, pid, cc, format, &identify,
                   sizeof(identify), ack);
  if (ack != NULL) {
    return ack->type == RDM_RESPONSE_TYPE_ACK;
  } else {
    rdm_header_t header;
    return rdm_read_header(dmx_num, &header) &&
           header.response_type == RDM_RESPONSE_TYPE_ACK;
  }
}