#include "dmx/include/service.h"

#include <string.h>

#include "dmx/include/driver.h"

dmx_device_t *dmx_device_get(dmx_port_t dmx_num, dmx_device_num_t device_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(device_num < RDM_SUB_DEVICE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_device_t *device = &dmx_driver[dmx_num]->device.root;
  while (device->num != device_num) {
    device = device->next;
    if (device == NULL) {
      return NULL;  // Sub-device does not exist
    }
  }

  return device;
}

bool dmx_parameter_add(dmx_port_t dmx_num, dmx_device_num_t device_num,
                       rdm_pid_t pid, int type, void *data, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(device_num < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the sub-device
  dmx_device_t *device = dmx_device_get(dmx_num, device_num);
  if (device == NULL) {
    return false;  // Device does not exist
  }

  // Iterate through parameters until an empty parameter is found
  const uint32_t parameter_count =
      device_num == RDM_SUB_DEVICE_ROOT
          ? driver->device.parameter_count.root
          : driver->device.parameter_count.sub_devices;
  for (int i = 0; i < parameter_count; ++i) {
    if (device->parameters[i].pid == pid) {
      return true;  // Parameter already exists
    } else if (device->parameters[i].pid == 0) {
      // Initialize parameter memory
      switch (type) {
        case DMX_PARAMETER_TYPE_DYNAMIC:
        case DMX_PARAMETER_TYPE_NON_VOLATILE:
          device->parameters[i].data = malloc(size);
          if (device->parameters[i].data == NULL) {
            DMX_ERR("parameter malloc error");
            return false;
          }
          if (data == NULL) {
            memset(device->parameters[i].data, 0, size);
          } else {
            memcpy(device->parameters[i].data, data, size);
          }
          break;
        case DMX_PARAMETER_TYPE_STATIC:
          device->parameters[i].data = data;
          break;
        case DMX_PARAMETER_TYPE_NULL:
          device->parameters[i].data = NULL;
          break;
        default:
          return false;
      }

      device->parameters[i].pid = pid;
      device->parameters[i].size = size;
      device->parameters[i].type = type;
      device->parameters[i].definition = NULL;
      device->parameters[i].callback = NULL;
      return true;
    }
  }

  return false;  // No more parameters available on this sub-device
}

dmx_parameter_t *dmx_parameter_get_entry(dmx_port_t dmx_num,
                                         dmx_device_num_t device_num,
                                         rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(device_num < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_device_t *const device = dmx_device_get(dmx_num, device_num);
  if (device == NULL) {
    return NULL;  // Sub-device does not exist
  }

  // Get the parameter count for the requested device
  const int param_count =
      (device_num == RDM_SUB_DEVICE_ROOT)
          ? dmx_driver[dmx_num]->device.parameter_count.root
          : dmx_driver[dmx_num]->device.parameter_count.sub_devices;

  // Iterate the device's parameters
  for (int i = 0; i < param_count; ++i) {
    if (device->parameters[i].pid == pid) {
      return &device->parameters[i];
    }
  }

  return NULL;  // Parameter does not exist
}
