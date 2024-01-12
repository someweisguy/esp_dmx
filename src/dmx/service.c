#include "dmx/include/service.h"

#include "dmx/include/driver.h"

dmx_device_t *dmx_driver_get_device(dmx_port_t dmx_num,
                                    dmx_device_num_t device_num) {
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

bool dmx_driver_add_parameter(dmx_port_t dmx_num, dmx_device_num_t device_num,
                              rdm_pid_t pid, int type, void *data,
                              size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(device_num < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  // TODO: assert type
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the sub-device
  dmx_device_t *device = dmx_driver_get_device(dmx_num, device_num);
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

      // TODO: Initialize parameter memory; dynamic, nvs, static, or null

      device->parameters[i].pid = pid;
      device->parameters[i].definition = NULL;
      device->parameters[i].callback = NULL;
      return true;
    }
  }

  return false;  // No more parameters available on this sub-device
}

dmx_parameter_t *dmx_driver_get_parameter(dmx_port_t dmx_num,
                                          dmx_device_num_t device_num,
                                          rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(device_num < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_device_t *const device = dmx_driver_get_device(dmx_num, device_num);
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
