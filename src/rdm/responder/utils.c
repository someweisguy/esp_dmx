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
  assert(rdm_format_is_valid(format));
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
  return 0;  // TODO: implement write_ack_timer()
}

size_t rdm_write_ack_overflow(dmx_port_t dmx_num, const rdm_header_t *header,
                              const char *format, const void *pd, size_t pdl,
                              int page) {
  return 0;  // TODO: implement write_ack_overflow()
}

bool rdm_get_boot_loader(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  bool boot_loader;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  boot_loader = dmx_driver[dmx_num]->rdm.boot_loader;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return boot_loader;
}

void rdm_set_boot_loader(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->rdm.boot_loader = true;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
}

size_t rdm_simple_response_handler(dmx_port_t dmx_num,
                                   const rdm_parameter_definition_t *definition,
                                   const rdm_header_t *header) {
  const char *format;
  if (header->sub_device != RDM_SUB_DEVICE_ALL) {
    // Ensure the sub-device exists
    if (!dmx_sub_device_exists(dmx_num, header->sub_device)) {
      return rdm_write_nack_reason(dmx_num, header,
                                   RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    }

    // Ensure the sub-device has the requested parameter
    if (!dmx_parameter_exists(dmx_num, header->sub_device, header->pid)) {
      return rdm_write_nack_reason(dmx_num, header,
                                   RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    }

    // Handle the GET or SET command
    if (header->cc == RDM_CC_GET_COMMAND) {
      // Get the parameter and write it to the RDM bus
      size_t pdl = dmx_parameter_size(dmx_num, header->sub_device, header->pid);
      const void *pd =
          dmx_parameter_get_data(dmx_num, header->sub_device, header->pid);
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

  } else {
    // Return early if a GET command is received with RDM_SUB_DEVICE_ALL
    if (header->cc == RDM_CC_GET_COMMAND) {
      // Must return NACK with RDM_NR_SUB_DEVICE_OUT_OF_RANGE
      return rdm_write_nack_reason(dmx_num, header,
                                   RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    }

    // Iterate through all sub-devices and update the parameter
    uint8_t pd[231];
    format = definition->set.request.format;
    size_t size = rdm_read_pd(dmx_num, format, pd, header->pdl);
    for (int i = RDM_SUB_DEVICE_ROOT; i < RDM_SUB_DEVICE_MAX; ++i) {
      dmx_parameter_set(dmx_num, i, header->pid, pd, size);
    }
    return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
  }
}

bool rdm_definition_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                        rdm_pid_t pid,
                        const rdm_parameter_definition_t *definition) {
  assert(definition != NULL);
  assert(definition->pid_cc >= RDM_CC_DISC &&
         definition->pid_cc <= RDM_CC_GET_SET);
  assert((definition->ds >= RDM_DS_NOT_DEFINED &&
          definition->ds <= RDM_DS_SIGNED_DWORD) ||
         (definition->ds >= 0x80 && definition->ds <= 0xdf));
  assert(rdm_format_is_valid(definition->get.request.format) &&
         rdm_format_is_valid(definition->get.response.format));
  assert(rdm_format_is_valid(definition->set.request.format) &&
         rdm_format_is_valid(definition->set.response.format));
  assert(
      (definition->get.handler != NULL && (definition->pid_cc == RDM_CC_DISC ||
                                           definition->pid_cc == RDM_CC_GET)) ||
      (definition->set.handler != NULL && definition->pid_cc == RDM_CC_SET) ||
      (definition->get.handler != NULL && definition->set.handler != NULL &&
       definition->pid_cc == RDM_CC_GET_SET));
  assert(definition->pdl_size < RDM_PD_SIZE_MAX);
  assert(definition->units <= RDM_UNITS_BYTE ||
         (definition->units >= 0x80 && definition->units <= 0xff));
  assert(definition->prefix <= RDM_PREFIX_YOCTO ||
         (definition->prefix >= RDM_PREFIX_DECA &&
          definition->prefix <= RDM_PREFIX_YOTTA));

  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return false;
  }

  entry->definition = definition;

  return true;
}

const rdm_parameter_definition_t *rdm_definition_get(
    dmx_port_t dmx_num, rdm_sub_device_t sub_device, rdm_pid_t pid) {
  assert(pid > 0);

  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return NULL;
  }

  return entry->definition;
}

bool rdm_callback_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                      rdm_pid_t pid, rdm_callback_t callback, void *context) {
  assert(pid > 0);

  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return false;
  }

  entry->callback = callback;
  entry->context = context;

  return true;
}
