#include "include/parameter.h"

#include <ctype.h>
#include <string.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/driver.h"
#include "dmx/include/struct.h"

enum {
  DMX_PARAMETER_STORAGE_TYPE_VOLATILE = 0,
  DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE,
  DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE_STAGED
};

static dmx_parameter_t *dmx_parameter_get_entry(dmx_port_t dmx_num,
                                                rdm_sub_device_t sub_device,
                                                rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the sub-device
  rdm_device_t *device = &driver->device.root;
  while (device->num != sub_device) {
    device = device->next;
    if (device == NULL) {
      return NULL;  // Sub-device does not exist
    }
  }

  // Iterate through parameters until the correct parameter is found
  const uint32_t parameter_count =
      sub_device == RDM_SUB_DEVICE_ROOT
          ? driver->device.parameter_count.root
          : driver->device.parameter_count.sub_devices;
  for (int i = 0; i < parameter_count; ++i) {
    if (device->parameters[i].pid == pid) {
      return &device->parameters[i];
    } else if (device->parameters[i].pid == 0) {
      break;  // No more parameters on this sub-device
    }
  }

  return NULL;  // Parameter was not found
}

static dmx_parameter_t *dmx_parameter_add_entry(dmx_port_t dmx_num,
                                                rdm_sub_device_t sub_device,
                                                rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the sub-device
  rdm_device_t *device = &driver->device.root;
  while (device->num != sub_device) {
    device = device->next;
    if (device == NULL) {
      return NULL;  // Sub-device does not exist
    }
  }

  // Iterate through parameters until an empty parameter is found
  const uint32_t parameter_count =
      sub_device == RDM_SUB_DEVICE_ROOT
          ? driver->device.parameter_count.root
          : driver->device.parameter_count.sub_devices;
  for (int i = 0; i < parameter_count; ++i) {
    if (device->parameters[i].pid == pid) {
      return NULL;  // Parameter already exists
    } else if (device->parameters[i].pid == 0) {
      device->parameters[i].pid = pid;
      device->parameters[i].definition = NULL;
      device->parameters[i].callback = NULL;
      return &device->parameters[i];
    }
  }

  return NULL;  // No more parameters available on this sub-device
}

bool dmx_parameter_add_dynamic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               rdm_pid_t pid, bool non_volatile,
                               const void *init, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if the variable already exists
  if (dmx_parameter_exists(dmx_num, sub_device, pid)) {
    return true;
  }

  // Allocate parameter data if desired
  void *data;
  if (size > 0) {
    data = malloc(size);
    if (data == NULL) {
      DMX_ERR("RDM parameter malloc error")
      return false;
    }
  } else {
    non_volatile = false;
    data = NULL;
  }

  // Return early if there are no available parameter entries
  dmx_parameter_t *entry = dmx_parameter_add_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    if (data != NULL) {
      free(data);
    }
    return false;
  }

  // Configure parameter
  entry->size = size;
  entry->data = data;
  entry->is_heap_allocated = (size > 0);
  entry->storage_type = non_volatile ? DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE
                                     : DMX_PARAMETER_STORAGE_TYPE_VOLATILE;

  // Set the initial value of the variable
  if (entry->data != NULL) {
    if (init != NULL) {
      memcpy(entry->data, init, size);
    } else {
      memset(entry->data, 0, size);
    }
  }

  return true;
}

bool dmx_parameter_add_static(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, bool non_volatile, void *data,
                              size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if the variable already exists
  if (dmx_parameter_exists(dmx_num, sub_device, pid)) {
    return true;
  }

  // Return early if there are no available parameter entries
  dmx_parameter_t *entry = dmx_parameter_add_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return false;
  }

  // Configure parameter
  entry->size = size;
  entry->data = data;
  entry->is_heap_allocated = false;
  entry->storage_type = non_volatile ? DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE
                                     : DMX_PARAMETER_STORAGE_TYPE_VOLATILE;

  return true;
}

bool dmx_parameter_add_null(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if the variable already exists
  if (dmx_parameter_exists(dmx_num, sub_device, pid)) {
    return true;
  }

  // Return early if there are no available parameter entries
  dmx_parameter_t *entry = dmx_parameter_add_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return false;
  }

  // Configure parameter
  entry->size = 0;
  entry->data = NULL;
  entry->is_heap_allocated = false;
  entry->storage_type = DMX_PARAMETER_STORAGE_TYPE_VOLATILE;

  return true;
}

bool dmx_parameter_exists(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  return (dmx_parameter_get_entry(dmx_num, sub_device, pid) != NULL);
}

rdm_pid_t dmx_parameter_at(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           uint32_t index) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Return early if the parameter index is out of bounds
  if ((sub_device == RDM_SUB_DEVICE_ROOT &&
       index > driver->device.parameter_count.root) ||
      (sub_device > RDM_SUB_DEVICE_ROOT &&
       index > driver->device.parameter_count.sub_devices)) {
    return 0;
  }

  // Find the desired sub-device number
  rdm_device_t *device = &driver->device.root;
  while (device->num != sub_device) {
    device = device->next;
    if (device == NULL) {
      return 0;  // Sub-device does not exist
    }
  }

  return device->parameters[index].pid;
}

size_t dmx_parameter_size(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return 0;
  }

  return entry->size;
}

void *dmx_parameter_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                        rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Find the parameter and return it
  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return NULL;
  }

  return entry->data;
}

size_t dmx_parameter_copy(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                          rdm_pid_t pid, void *destination, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Find the parameter
  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
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

size_t dmx_parameter_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                         rdm_pid_t pid, const void *source, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if there is nothing to write
  if (source == NULL || size == 0) {
    return 0;
  }

  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
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
  if (entry->storage_type == DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE) {
    entry->storage_type = DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE_STAGED;
    ++dmx_driver[dmx_num]->device.parameter_count.staged;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

rdm_pid_t dmx_parameter_commit(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Guard against unnecessarily iterating through all parameters
  if (driver->device.parameter_count.staged == 0) {
    return 0;
  }

  rdm_sub_device_t sub_device;
  rdm_pid_t pid = 0;
  void *data = NULL;

  // Iterate through parameters and commit the first found value to NVS
  rdm_device_t *device = &driver->device.root;
  for (; device != NULL && pid == 0; device = device->next) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    for (int i = 0; i < driver->device.parameter_count.root; ++i) {
      if (device->parameters[i].storage_type ==
          DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE_STAGED) {
        device->parameters[i].storage_type =
            DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE;
        --driver->device.parameter_count.staged;
        sub_device = device->num;
        pid = device->parameters[i].pid;
        data = device->parameters[i].data;
        break;
      }
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  if (pid > 0) {
    dmx_nvs_set(dmx_num, sub_device, pid, RDM_DS_NOT_DEFINED, data,
                dmx_parameter_size(dmx_num, sub_device, pid));
  }

  return pid;
}

bool dmx_parameter_rdm_format_is_valid(const char *format) {
  if (format == NULL) {
    return true;
  }

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
            return false;  // Hex literals must be 2 characters wide
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
        return false;  // Unknown symbol
    }

    // Update the parameter size with the new token
    parameter_size += token_size;
    if (parameter_size > 231) {
      return false;  // Parameter size is too big
    }

    // End loop if parameter is terminated
    if (format_is_terminated) {
      break;
    }
  }

  if (format_is_terminated) {
    ++format;
    if (*format != '\0' && *format != '$') {
      return false;  // Invalid token after terminator
    }
  } else {
    // Get the maximum possible size if parameter is unterminated
    parameter_size = 231 - (231 % parameter_size);
  }

  return parameter_size > 0;
}

bool dmx_parameter_rdm_define(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid,
                              const rdm_parameter_definition_t *definition) {
  assert(definition != NULL);
  assert(definition->pid_cc >= RDM_CC_DISC &&
         definition->pid_cc <= RDM_CC_GET_SET);
  assert((definition->ds >= RDM_DS_NOT_DEFINED &&
          definition->ds <= RDM_DS_SIGNED_DWORD) ||
         (definition->ds >= 0x80 && definition->ds <= 0xdf));
  assert(dmx_parameter_rdm_format_is_valid(definition->get.request.format) &&
         dmx_parameter_rdm_format_is_valid(definition->get.response.format));
  assert(dmx_parameter_rdm_format_is_valid(definition->set.request.format) &&
         dmx_parameter_rdm_format_is_valid(definition->set.response.format));
  assert(
      (definition->get.handler != NULL && (definition->pid_cc == RDM_CC_DISC ||
                                           definition->pid_cc == RDM_CC_GET)) ||
      (definition->set.handler != NULL && definition->pid_cc == RDM_CC_SET) ||
      (definition->get.handler != NULL && definition->set.handler != NULL &&
       definition->pid_cc == RDM_CC_GET_SET));
  assert(definition->pdl_size < RDM_PD_SIZE_MAX);
  assert(definition->units <= RDM_UNITS_BYTES ||
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

const rdm_parameter_definition_t *dmx_parameter_rdm_lookup(
    dmx_port_t dmx_num, rdm_sub_device_t sub_device, rdm_pid_t pid) {
  assert(pid > 0);

  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return NULL;
  }

  return entry->definition;
}

bool dmx_parameter_rdm_set_callback(dmx_port_t dmx_num,
                                    rdm_sub_device_t sub_device, rdm_pid_t pid,
                                    rdm_callback_t callback, void *context) {
  assert(pid > 0);

  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return false;
  }

  entry->callback = callback;
  entry->context = context;

  return true;
}

bool dmx_parameter_rdm_handle_callback(dmx_port_t dmx_num,
                                       rdm_sub_device_t sub_device,
                                       rdm_pid_t pid,
                                       rdm_header_t *request_header,
                                       rdm_header_t *response_header) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0);
  assert(request_header != NULL);
  assert(response_header != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  // Search for a dictionary entry for the parameter
  dmx_parameter_t *entry = dmx_parameter_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return false;
  }

  if (entry->callback != NULL) {
    entry->callback(dmx_num, request_header, response_header, entry->context);
  }

  return true;
}
