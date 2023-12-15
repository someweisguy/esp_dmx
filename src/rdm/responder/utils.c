#include "include/utils.h"

#include <ctype.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/driver.h"
#include "dmx/include/io.h"
#include "dmx/include/struct.h"
#include "rdm/uid.h"

enum rdm_pd_flags_e {
  RDM_PD_STORAGE_TYPE_VOLATILE = 0,
  RDM_PD_STORAGE_TYPE_NON_VOLATILE,
  RDM_PD_STORAGE_TYPE_NON_VOLATILE_STAGED
};

// TODO: docs
static uint32_t rdm_definition_count = 0;
static struct rdm_pd_dictionary_s {
  const rdm_pd_definition_t *definition;
  rdm_callback_t callback;
  void *context;
} rdm_dictionary[RDM_RESPONDER_NUM_PIDS_MAX];

static struct rdm_parameter_s *rdm_pd_get_entry(dmx_port_t dmx_num,
                                                rdm_sub_device_t sub_device,
                                                rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO: Implement sub-devices
  assert(sub_device == 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get the RDM sub-device
  rdm_device_t *device = &driver->rdm.root_device;
  // TODO: loop through sub-device until the correct sub-device is found

  // Iterate through parameters until the correct parameter is found
  const uint32_t parameter_count = sub_device == RDM_SUB_DEVICE_ROOT
                                       ? driver->rdm.root_device_parameter_max
                                       : driver->rdm.sub_device_parameter_max;
  for (int i = 0; i < parameter_count; ++i) {
    if (device->parameters[i].pid == pid) {
      return &device->parameters[i];
    } else if (device->parameters[i].pid == 0) {
      break;
    }
  }

  return NULL;
}

static struct rdm_parameter_s *rdm_pd_add_entry(dmx_port_t dmx_num,
                                                rdm_sub_device_t sub_device,
                                                rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO: Implement sub-devices
  assert(sub_device == 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get the RDM sub-device
  rdm_device_t *device = &driver->rdm.root_device;
  // TODO: loop through sub-device until the correct sub-device is found

  // Iterate through parameters until the correct parameter is found
  struct rdm_parameter_s *entry = NULL;
  const uint32_t parameter_count = sub_device == RDM_SUB_DEVICE_ROOT
                                       ? driver->rdm.root_device_parameter_max
                                       : driver->rdm.sub_device_parameter_max;
  for (int i = 0; i < parameter_count; ++i) {
    if (device->parameters[i].pid == 0) {
      entry = &device->parameters[i];
      break;
    }
  }
    
  // Set default parameter values
  if (entry != NULL) {
    entry->pid = pid;
    entry->is_queued = false;
  }

  return entry;
}

const rdm_pd_definition_t *rdm_parameter_lookup(rdm_pid_t pid) {
  assert(pid > 0);

  // Search for and return a pointer to the definition
  for (int i = 0; i < rdm_definition_count; ++i) {
    if (rdm_dictionary[i].definition->pid == pid) {
      return rdm_dictionary[i].definition;
    }
  }

  return NULL;
}

int rdm_parameter_define(const rdm_pd_definition_t *definition) {
  assert(definition != NULL);
  assert(definition->pid > 0);
  assert((definition->ds >= RDM_DS_NOT_DEFINED &&
          definition->ds <= RDM_DS_SIGNED_DWORD) ||
         (definition->ds >= 0x80 && definition->ds <= 0xdf));
  assert(definition->pid_cc >= RDM_CC_DISC &&
         definition->pid_cc <= RDM_CC_GET_SET);
  assert(definition->get.handler != NULL || definition->set.handler != NULL);
  assert(!(definition->pid >= RDM_PID_MANUFACTURER_SPECIFIC_BEGIN &&
           definition->pid <= RDM_PID_MANUFACTURER_SPECIFIC_END) ||
         definition->description == NULL);

  // Return early if the definition already exists
  if (rdm_parameter_lookup(definition->pid) != NULL) {
    return 0;
  }

  // Add the definition and increment the definition count
  const uint32_t i = rdm_definition_count;
  rdm_dictionary[i].definition = definition;
  rdm_dictionary[i].callback = NULL;
  // Don't need to set the value of the context pointer yet
  ++rdm_definition_count;

  return rdm_definition_count;
}

void rdm_pd_handle_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_header_t *request_header,
                            rdm_header_t *response_header) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0);
  assert(request_header != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  // Search for a dictionary entry for the parameter
  const struct rdm_pd_dictionary_s *dict_entry = NULL;
  for (int i = 0; i < rdm_definition_count; ++i) {
    if (rdm_dictionary[i].definition->pid == pid) {
      dict_entry = &rdm_dictionary[i];
    }
  }
  if (dict_entry == NULL || dict_entry->callback == NULL) {
    return;
  }

  void *context = dict_entry->context;
  dict_entry->callback(dmx_num, request_header, response_header, context);
}

bool rdm_pd_set_callback(rdm_pid_t pid, rdm_callback_t callback,
                         void *context) {
  assert(pid > 0);

  // Search for the definition and add the callback
  for (int i = 0; i < rdm_definition_count; ++i) {
    if (rdm_dictionary[i].definition->pid == pid) {
      rdm_dictionary[i].callback = callback;
      rdm_dictionary[i].context = context;
      return true;
    }
  }

  return false;
}

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
    .pdl = pdl
  };

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
    .pdl = pdl
  };

  return rdm_write(dmx_num, &response_header, "w", &nack_reason);
}

void rdm_set_boot_loader(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
}

size_t rdm_pd_format_get_max_size(const char *format) {
  size_t parameter_size = 0;

  bool format_is_terminated = false;
  for (char c = *format; c != '\0'; c = *(++format)) {
    // Skip spaces
    if (c == ' ') {
      continue;
    }

    // Get the size of the current token
    size_t token_size;
    switch (c) {
      case 'b':
      case 'B':
        token_size = sizeof(uint8_t);
        break;
      case 'w':
      case 'W':
        token_size = sizeof(uint16_t);
        break;
      case 'd':
      case 'D':
        token_size = sizeof(uint32_t);
        break;
      case 'u':
      case 'U':
        token_size = sizeof(rdm_uid_t);
        break;
      case 'v':
      case 'V':
        token_size = sizeof(rdm_uid_t);
        format_is_terminated = true;
        break;
      case 'x':
      case 'X':
        token_size = sizeof(uint8_t);
        for (int i = 0; i < 2; ++i) {
          c = *(++format);
          if (!isxdigit(c)) {
            return 0;  // Hex literals must be 2 characters wide
          }
        }
        break;
      case 'a':
      case 'A':
        token_size = 32;  // ASCII fields can be up to 32 bytes
        format_is_terminated = true;
        break;
      case '$':
        token_size = 0;
        format_is_terminated = true;
        break;
      default:
        return 0;  // Unknown symbol
    }

    // Update the parameter size with the new token
    parameter_size += token_size;
    if (parameter_size > 231) {
      return 0;  // Parameter size is too big
    }

    // End loop if parameter is terminated
    if (format_is_terminated) {
      break;
    }
  }

  if (format_is_terminated) {
    ++format;
    if (*format != '\0' && *format != '$') {
      return 0;  // Invalid token after terminator
    }
  } else {
    // Get the maximum possible size if parameter is unterminated
    parameter_size = 231 - (231 % parameter_size);
  }

  return parameter_size;
}

bool rdm_parameter_exists(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid) {
  return (rdm_pd_get_entry(dmx_num, sub_device, pid) != NULL);
}

bool rdm_parameter_add_dynamic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               rdm_pid_t pid, bool non_volatile,
                               const void *init, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(size > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if the variable already exists
  if (rdm_parameter_exists(dmx_num, sub_device, pid)) {
    return true;
  }

  // Return early if there is no heap space available
  void *data = malloc(size);
  if (data == NULL) {
    // TODO: log error
    return false;
  }

  // Return early if there are no available parameter entries
  rdm_parameter_t *entry = rdm_pd_add_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    free(data);
    return false;
  }

  // Configure parameter
  entry->size = size;
  entry->data = data;
  entry->is_heap_allocated = true;
  entry->non_volatile = non_volatile ? RDM_PD_STORAGE_TYPE_NON_VOLATILE
                                     : RDM_PD_STORAGE_TYPE_VOLATILE;

  // Set the initial value of the variable
  if (init != NULL) {
    memcpy(entry->data, init, size);
  } else {
    memset(entry->data, 0, size);
  }

  return true;
}

bool rdm_parameter_add_static(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, bool non_volatile, void *data,
                              size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if the variable already exists
  if (rdm_parameter_exists(dmx_num, sub_device, pid)) {
    return true;
  }

  // Return early if there are no available parameter entries
  rdm_parameter_t *entry = rdm_pd_add_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return false;
  }

  // Configure parameter
  entry->size = size;
  entry->data = data;
  entry->is_heap_allocated = false;
  entry->non_volatile = non_volatile ? RDM_PD_STORAGE_TYPE_NON_VOLATILE
                                     : RDM_PD_STORAGE_TYPE_VOLATILE;

  return true;
}

const void *rdm_parameter_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Find the parameter and return it
  rdm_parameter_t *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return NULL;
  }

  return entry->data;
}

size_t rdm_parameter_copy(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                  rdm_pid_t pid, void *destination, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Find the parameter
  rdm_parameter_t *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return 0;
  }

  // Clamp the parameter size
  if (size > entry->size) {
    size = entry->size;
  } 

  // Copy the parameter
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(destination, entry->data, size);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

size_t rdm_parameter_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                  rdm_pid_t pid, const void *source, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if there is nothing to write
  if (source == NULL || size == 0) {
    return 0;
  }

  rdm_parameter_t *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return 0;
  }
  assert(entry->data != NULL);

  // Clamp the write size to the definition size
  if (size > entry->size) {
    size = entry->size;
  }

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(entry->data, source, size);
  if (entry->non_volatile == RDM_PD_STORAGE_TYPE_NON_VOLATILE) {
    entry->non_volatile = RDM_PD_STORAGE_TYPE_NON_VOLATILE_STAGED;
    ++dmx_driver[dmx_num]->rdm.staged_count;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

size_t rdm_parameter_set_and_queue(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, const void *source, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // FIXME: remove this function?

  const rdm_pd_definition_t *definition = rdm_parameter_lookup(pid);
  assert(definition != NULL);

  // Return early if there is nothing to write
  if (source == NULL || size == 0) {
    return 0;
  }

  // Clamp the write size to the definition size
  if (size > definition->alloc_size) {
    size = definition->alloc_size;
  }

  rdm_parameter_t *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return 0;
  }
  assert(entry->data != NULL);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(entry->data, source, size);
  entry->is_queued = true;
  ++driver->rdm.queue_count;
  if (entry->non_volatile == RDM_PD_STORAGE_TYPE_NON_VOLATILE) {
    entry->non_volatile = RDM_PD_STORAGE_TYPE_NON_VOLATILE_STAGED;
    ++dmx_driver[dmx_num]->rdm.staged_count;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

rdm_pid_t rdm_queue_pop(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // TODO: reduce potential time spent in critical section
  rdm_pid_t pid = 0;
  if (rdm_queue_size(dmx_num) > 0) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    rdm_device_t *device = &driver->rdm.root_device;
    int parameter_count = driver->rdm.root_device_parameter_max;
    while (device != NULL) {
      for (int i = 0; i < parameter_count; ++i) {
        if (device->parameters[i].is_queued) {
          device->parameters[i].is_queued = false;
          --driver->rdm.queue_count;
          driver->rdm.previous_popped = device->parameters[i].pid;
          pid = device->parameters[i].pid;
          break;
        }
      }
      parameter_count = driver->rdm.sub_device_parameter_max;
    }
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  return pid;
}

uint8_t rdm_queue_size(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  uint32_t size;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  size = driver->rdm.queue_count;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (size > 255) {
    size = 255;  // RDM requires queue size to be clamped
  }

  return size;
}

rdm_pid_t rdm_queue_previous(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  rdm_pid_t pid;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  pid = driver->rdm.previous_popped;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}

rdm_pid_t rdm_parameter_commit(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Guard against unnecessarily iterating through all parameters
  if (driver->rdm.staged_count == 0) {
    return 0;
  }

  rdm_sub_device_t sub_device;
  rdm_pid_t pid = 0;
  void *data = NULL;

  // Iterate through parameters and commit the first found value to NVS
  rdm_device_t *device = &driver->rdm.root_device;
  for (; device != NULL && pid == 0; device = device->next) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    for (int i = 0; i < driver->rdm.root_device_parameter_max; ++i) {
      if (device->parameters[i].non_volatile ==
          RDM_PD_STORAGE_TYPE_NON_VOLATILE_STAGED) {
        device->parameters[i].non_volatile = RDM_PD_STORAGE_TYPE_NON_VOLATILE;
        --driver->rdm.staged_count;
        sub_device = device->device_num;
        pid = device->parameters[i].pid;
        data = device->parameters[i].data;
        break;
      }
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  if (pid > 0) {
    const rdm_pd_definition_t *definition = rdm_parameter_lookup(pid);
    assert(definition != NULL);
    dmx_nvs_set(dmx_num, pid, sub_device, definition->ds, data,
                definition->alloc_size);
  }

  return pid;
}

size_t rdm_simple_response_handler(dmx_port_t dmx_num,
                                   const rdm_pd_definition_t *definition,
                                   const rdm_header_t *header) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    return rdm_write_nack_reason(dmx_num, header,
                                 RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  }
  
  const char *format;
  if (header->cc == RDM_CC_GET_COMMAND) {
    // Get the parameter and write it to the RDM bus
    const void *pd = rdm_parameter_get(dmx_num, header->sub_device, header->pid);
    format = definition->get.response.format;
    return rdm_write_ack(dmx_num, header, format, pd, definition->alloc_size);
  } else {
    // Get the parameter from the request and write it to the RDM driver
    uint8_t pd[231];
    format = definition->set.request.format;
    size_t size = rdm_read_pd(dmx_num, format, pd, header->pdl);
    rdm_parameter_set(dmx_num, header->sub_device, header->pid, pd, size);
    return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
  }
}