#include "include/utils.h"

#include <string.h>

#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/include/uid.h"

size_t rdm_write_ack(dmx_port_t dmx_num, const rdm_header_t *header,
                     const char *format, const void *pd, size_t pdl) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(header != NULL);
  assert(rdm_cc_is_request(header->cc));
  // assert(format == NULL || rdm_pd_format_is_valid(format)); // TODO
  assert(format != NULL || pd == NULL);
  assert(pd != NULL || pdl == 0);
  assert(pdl < 231);
  assert(dmx_driver_is_installed(dmx_num));

  // Build the response header
  rdm_header_t response_header = {
      .message_len = 24 + pdl,
      .dest_uid = header->src_uid,
      .src_uid = *rdm_uid_get(dmx_num),
      .tn = header->tn,
      .response_type = RDM_RESPONSE_TYPE_ACK,
      .message_count = rdm_queue_size(dmx_num),
      .sub_device = header->sub_device,
      .cc = (header->cc | 0x1),  // Set to RDM_CC_x_COMMAND_RESPONSE
      .pid = header->pid,
      .pdl = pdl};

  return rdm_write(dmx_num, &response_header, format, pd);
}

size_t rdm_write_nack_reason(dmx_port_t dmx_num, const rdm_header_t *header,
                             rdm_nr_t nack_reason) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(header != NULL);
  assert(rdm_cc_is_request(header->cc));
  assert(dmx_driver_is_installed(dmx_num));

  // PDL is a single word
  const size_t pdl = sizeof(uint16_t);

  // Build the response header
  rdm_header_t response_header = {
      .message_len = 24 + pdl,
      .dest_uid = header->src_uid,
      .src_uid = *rdm_uid_get(dmx_num),
      .tn = header->tn,
      .response_type = RDM_RESPONSE_TYPE_NACK_REASON,
      .message_count = rdm_queue_size(dmx_num),
      .sub_device = header->sub_device,
      .cc = (header->cc | 0x1),  // Set to RDM_CC_x_COMMAND_RESPONSE
      .pid = header->pid,
      .pdl = pdl};

  return rdm_write(dmx_num, &response_header, "w", &nack_reason);
}

size_t rdm_write_ack_timer(dmx_port_t dmx_num, const rdm_header_t *header,
                           TickType_t ready_ticks) {
  return 0;  // TODO
}

size_t rdm_write_ack_overflow(dmx_port_t dmx_num, const rdm_header_t *header,
                              const char *format, const void *pd, size_t pdl,
                              int page) {
  return 0;  // TODO
}

void rdm_set_boot_loader(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
}

size_t rdm_simple_response_handler(dmx_port_t dmx_num,
                                   const rdm_parameter_definition_t *definition,
                                   const rdm_header_t *header) {
  // TODO: support header->sub_device == RDM_SUB_DEVICE_ALL
  if (!dmx_parameter_exists(dmx_num, header->sub_device, header->pid)) {
    return rdm_write_nack_reason(dmx_num, header,
                                 RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  }

  const char *format;
  if (header->cc == RDM_CC_GET_COMMAND) {
    // Get the parameter and write it to the RDM bus
    size_t pdl = dmx_parameter_size(dmx_num, header->sub_device, header->pid);
    const void *pd =
        dmx_parameter_get(dmx_num, header->sub_device, header->pid);
    format = definition->get.response.format;
    return rdm_write_ack(dmx_num, header, format, pd, pdl);
  } else {
    // Get the parameter from the request and write it to the RDM driver
    uint8_t pd[231];
    format = definition->set.request.format;
    size_t size = rdm_read_pd(dmx_num, format, pd, header->pdl);
    dmx_parameter_set(dmx_num, header->sub_device, header->pid, pd, size);
    return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
  }
}