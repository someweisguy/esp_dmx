#pragma once

#include <stdint.h>

#include "esp_dmx.h"
#include "impl/rdm_encode_types.h"
#include "rdm_constants.h"
#include "rdm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t rdm_encode_disc_unique_branch_response(void *destination, size_t size,
                                              rdm_uid_t uid) {
  uint8_t *data = destination;
  
  // Encode the RDM preamble and delimiter
  for (int i = 0; i < 7; ++i) {
    data[i] = RDM_PREAMBLE;
  }
  data[7] = RDM_DELIMITER;
  
  // Encode the UID and calculate the checksum
  uint16_t checksum = 0;
  for (int i = 8, j = 5; i < 20; i += 2, --j) {
    data[i] = ((uint8_t *)&uid)[j] | 0xaa;
    data[i + 1] = ((uint8_t *)&uid)[j] | 0x55;
    checksum += ((uint8_t *)&uid)[j] + (0xaa + 0x55);
  }

  // Encode the checksum
  data[20] = (checksum >> 8) | 0xaa;
  data[21] = (checksum >> 8) | 0x55;
  data[22] = checksum | 0xaa;
  data[23] = checksum | 0x55;

  return 24;
}

size_t rdm_encode_disc_mute(rdm_data_t *destination, size_t size, rdm_cc_t cc,
                            const void *params, size_t num_params,
                            size_t message_num) {
  size_t bytes_encoded = 0;
  rdm_disc_mute_data_t *buf = (rdm_disc_mute_data_t *)destination->pd;
  const rdm_disc_mute_t *pd = (const rdm_disc_mute_t *)params;

  // No parameters to encode on RDM_CC_DISC_COMMAND

  if (cc == RDM_CC_DISC_COMMAND_RESPONSE) {
    buf->managed_proxy = pd->managed_proxy;
    buf->sub_device = pd->sub_device;
    buf->boot_loader = pd->boot_loader;
    buf->proxied_device = pd->proxied_device;
    bytes_encoded = sizeof(buf->control_field);
    if (pd->binding_uid != 0) {
      // Binding UID is an optional parameter
      uid_to_buf(buf->binding_uid, pd->binding_uid);
      bytes_encoded += sizeof(buf->binding_uid);
    }
  }
  
  return bytes_encoded;
}

size_t rdm_decode_disc_mute(const rdm_data_t *source, size_t size, rdm_cc_t cc,
                            void *params, size_t num_params,
                            size_t message_num) {
  size_t params_available = 0;
  const rdm_disc_mute_data_t *buf = (const rdm_disc_mute_data_t *)source->pd;
  rdm_disc_mute_t *pd = (rdm_disc_mute_t *)params;

  // No parameters to decode on RDM_CC_DISC_COMMAND

  if (cc == RDM_CC_DISC_COMMAND_RESPONSE) {
    pd->managed_proxy = buf->managed_proxy;
    pd->sub_device = buf->sub_device;
    pd->boot_loader = buf->boot_loader;
    pd->proxied_device = buf->proxied_device;
    if (source->pdl >= 8) {
      // Binding UID is an optional parameter
      pd->binding_uid = buf_to_uid(buf->binding_uid);
    } else {
      pd->binding_uid = 0;
    }
    params_available = 1;
  }

  return params_available;
}

#ifdef __cplusplus
}
#endif
