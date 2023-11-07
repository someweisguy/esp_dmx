/*
#include "rdm/responder.h"

#include "dmx/device.h"
#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"
#include "endian.h"
#include "esp_dmx.h"
#include "rdm/utils/bus_ctl.h"
#include "rdm/utils/uid.h"

static int rdm_simple_response_cb(dmx_port_t dmx_num, rdm_header_t *header,
                                  void *pd, uint8_t *pdl_out,
                                  const char *format) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  void *data = rdm_pd_get(dmx_num, header->pid, header->sub_device);
  if (header->cc == RDM_CC_GET_COMMAND) {
    *pdl_out = rdm_emplace(pd, format, data, 231, false);
  } else {
    rdm_emplace(data, format, pd, header->pdl, true);
  }

  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_manufacturer_specific_simple(dmx_port_t dmx_num, rdm_pid_description_t description,
                                               void* data, const char *format, rdm_callback_t cb,
                                               void *context, bool nvs)
{
  // return rdm_pd_register(dmx_num, RDM_SUB_DEVICE_ROOT, &description, format,
  //                               rdm_simple_response_cb, data, cb, context, nvs);  

  // FIXME
  return false;
}



*/