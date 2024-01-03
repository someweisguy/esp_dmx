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
