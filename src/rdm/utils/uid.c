#include "rdm/utils/include/uid.h"

#include "dmx/include/struct.h"
#include "endian.h"

void DMX_ISR_ATTR rdm_uid_get(dmx_port_t dmx_num, rdm_uid_t *uid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(uid != NULL);

  // Copy the binding UID and increment the final octet by dmx_num
  uid->man_id = rdm_device_uid.man_id;
  uid->dev_id = rdm_device_uid.dev_id;
  if (!rdm_uid_is_null(uid)) {
    uint8_t last_octet = (uint8_t)uid->dev_id;
    last_octet += dmx_num;
    uid->dev_id &= 0xffffff00;
    uid->dev_id |= last_octet;
  }
}

void rdm_uid_get_binding(rdm_uid_t *uid) { rdm_uid_get(rdm_binding_port, uid); }
