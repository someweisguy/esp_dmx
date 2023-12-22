#include "include/parameter.h"

#include <ctype.h>
#include <string.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/driver.h"
#include "dmx/include/struct.h"

// TODO: docs
enum dmx_parameter_storage_type_t {
  DMX_PARAMETER_STORAGE_TYPE_VOLATILE = 0,
  DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE,
  DMX_PARAMETER_STORAGE_TYPE_NON_VOLATILE_STAGED
};

static struct dmx_parameter_s *rdm_parameter_get_entry(
    dmx_port_t dmx_num, rdm_sub_device_t sub_device, rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO: Implement sub-devices
  assert(sub_device == 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get the RDM sub-device
  rdm_device_t *device = &driver->device.root;
  // TODO: loop through sub-device until the correct sub-device is found

  // Iterate through parameters until the correct parameter is found
  const uint32_t parameter_count =
      sub_device == RDM_SUB_DEVICE_ROOT
          ? driver->device.parameter_count.root
          : driver->device.parameter_count.sub_devices;
  for (int i = 0; i < parameter_count; ++i) {
    if (device->parameters[i].pid == pid) {
      return &device->parameters[i];
    } else if (device->parameters[i].pid == 0) {
      break;
    }
  }

  return NULL;
}

static struct dmx_parameter_s *rdm_parameter_add_entry(
    dmx_port_t dmx_num, rdm_sub_device_t sub_device, rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO: Implement sub-devices
  assert(sub_device == 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get the RDM sub-device
  rdm_device_t *device = &driver->device.root;
  // TODO: loop through sub-device until the correct sub-device is found

  // Iterate through parameters until the correct parameter is found
  struct dmx_parameter_s *entry = NULL;
  const uint32_t parameter_count =
      sub_device == RDM_SUB_DEVICE_ROOT
          ? driver->device.parameter_count.root
          : driver->device.parameter_count.sub_devices;
  for (int i = 0; i < parameter_count; ++i) {
    if (device->parameters[i].pid == 0) {
      entry = &device->parameters[i];
      break;
    }
  }

  // Set default parameter values
  if (entry != NULL) {
    entry->pid = pid;
  }

  return entry;
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
  rdm_parameter_t *entry = rdm_parameter_add_entry(dmx_num, sub_device, pid);
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
  rdm_parameter_t *entry = rdm_parameter_add_entry(dmx_num, sub_device, pid);
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
  rdm_parameter_t *entry = rdm_parameter_add_entry(dmx_num, sub_device, pid);
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

  return (rdm_parameter_get_entry(dmx_num, sub_device, pid) != NULL);
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
  while (device->device_num != sub_device) {
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

  rdm_parameter_t *entry = rdm_parameter_get_entry(dmx_num, sub_device, pid);
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
  rdm_parameter_t *entry = rdm_parameter_get_entry(dmx_num, sub_device, pid);
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
  rdm_parameter_t *entry = rdm_parameter_get_entry(dmx_num, sub_device, pid);
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

  rdm_parameter_t *entry = rdm_parameter_get_entry(dmx_num, sub_device, pid);
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
        sub_device = device->device_num;
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

size_t dmx_parameter_format_size(const char *format) {
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
