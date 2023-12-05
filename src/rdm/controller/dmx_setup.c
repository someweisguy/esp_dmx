#include "dmx_setup.h"

#include "dmx/driver.h"
#include "dmx/struct.h"
#include "rdm/utils/uid.h"
#include "rdm/utils/bus_ctl.h"

size_t rdm_send_get_dmx_start_address(dmx_port_t dmx_num,
                                      const rdm_uid_t *dest_uid,
                                      rdm_sub_device_t sub_device,
                                      uint16_t *dmx_start_address,
                                      rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dest_uid != NULL, 0, "dest_uid is null");
  DMX_CHECK(dmx_start_address != NULL, 0, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_cc_t cc = RDM_CC_GET_COMMAND;
  const rdm_pid_t pid = RDM_PID_DMX_START_ADDRESS;
  size_t pdl = rdm_send_generic(dmx_num, dest_uid, sub_device, pid, cc, NULL,
                                NULL, 0, ack);
  if (pdl == sizeof(*dmx_start_address)) {
    const char *format = "w$";
    rdm_read_pd(dmx_num, format, dmx_start_address, sizeof(*dmx_start_address));
  }
  return pdl;
}

bool rdm_send_set_dmx_start_address(dmx_port_t dmx_num,
                                    const rdm_uid_t *dest_uid,
                                    rdm_sub_device_t sub_device,
                                    const uint16_t dmx_start_address,
                                    rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dest_uid != NULL, 0, "dest_uid is null");
  DMX_CHECK(dmx_start_address < 513, 0, "dmx_start_address is invalid");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const char *format = "w$";
  const rdm_cc_t cc = RDM_CC_SET_COMMAND;
  const rdm_pid_t pid = RDM_PID_DMX_START_ADDRESS;
  rdm_send_generic(dmx_num, dest_uid, sub_device, pid, cc, format,
                   &dmx_start_address, sizeof(dmx_start_address), ack);
  if (ack != NULL) {
    return ack->type == RDM_RESPONSE_TYPE_ACK;
  } else {
    rdm_header_t header;
    return rdm_read_header(dmx_num, &header) &&
           header.response_type == RDM_RESPONSE_TYPE_ACK;
  }
}