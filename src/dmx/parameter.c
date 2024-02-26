#include "include/parameter.h"

#include <string.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/include/driver.h"

int dmx_sub_device_get_count(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  int count = -1;  // Don't count root device
  dmx_device_t *device = &driver->device.root;
  do {
    ++count;
    device = device->next;
  } while (device != NULL);

  return count;
}

bool dmx_sub_device_exists(dmx_port_t dmx_num, dmx_device_num_t device_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(device_num < RDM_SUB_DEVICE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_device_t *device = dmx_device_get(dmx_num, device_num);

  return device != NULL;
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
  dmx_device_t *device = &driver->device.root;
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

void *dmx_parameter_get_data(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
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
  if (entry->type == DMX_PARAMETER_TYPE_NON_VOLATILE) {
    entry->type = DMX_PARAMETER_TYPE_NON_VOLATILE_STAGED;
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
  dmx_device_t *device = &driver->device.root;
  for (; device != NULL && pid == 0; device = device->next) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    for (int i = 0; i < driver->device.parameter_count.root; ++i) {
      if (device->parameters[i].type ==
          DMX_PARAMETER_TYPE_NON_VOLATILE_STAGED) {
        device->parameters[i].type = DMX_PARAMETER_TYPE_NON_VOLATILE;
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
    dmx_nvs_set(dmx_num, sub_device, pid, data,
                dmx_parameter_size(dmx_num, sub_device, pid));
  }

  return pid;
}
