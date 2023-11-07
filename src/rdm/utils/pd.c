#include "rdm/utils/pd.h"

#include <string.h>

#include "dmx/bus_ctl.h"
#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"
#include "endian.h"
#include "esp_dmx.h"
#include "rdm/types.h"
#include "rdm/utils/uid.h"

const void *rdm_pd_add_new(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           rdm_pid_t pid, const rdm_pd_schema_t *schema,
                           const rdm_pd_dimensions_t *dimensions,
                           const void *init_value) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0 && pid <= 0xffff);
  assert(schema != NULL);
  assert(schema->data_type <= 0xdf);
  assert(schema->cc >= RDM_CC_DISC && schema->cc <= RDM_CC_GET_SET);
  assert(schema->size > 0);
  assert(schema->response_handler != NULL);
  if (dimensions != NULL && pid >= 0x8000) {
    assert((dimensions->units >= RDM_UNITS_NONE &&
            dimensions->units <= RDM_UNITS_BYTES) ||
           (dimensions->units >= 0x80 && dimensions->units <= 0xff));
    assert(dimensions->prefix >= RDM_PREFIX_NONE &&
           dimensions->prefix <= RDM_PREFIX_YOTTA);
  }
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  void *pd = NULL;

  // Ensure that the parameter has not already been defined
  uint32_t pdi = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; pdi < driver->num_parameters; ++pdi) {
    if (driver->params[pdi].pid == pid) {
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->params[pdi].pid == pid) {
    return pd;  // Parameter already exists
  }

  // Check if there is space to add a new parameter definition
  if (pdi == RDM_RESPONDER_NUM_PIDS_MAX) {
    return pd;  // No space for new parameter definitions
  }

  // Reserve space for the parameter data in the driver
  rdm_pd_dimensions_t *dims_ptr = NULL;
  const size_t dims_size = pid < 0x8000 ? sizeof(rdm_pd_dimensions_t) : 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const size_t pdl_available = driver->pd_size - driver->pd_head;
  if (schema->size + dims_size <= pdl_available) {
    pd = driver->pd + driver->pd_head;
    driver->pd_head += schema->size;
    if (dims_size > 0) {
      dims_ptr = driver->pd + driver->pd_head;
      driver->pd_head += dims_size;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (schema->size + dims_size > pdl_available) {
    return pd;  // No more reservable parameter data space
  }
  if (dims_ptr != NULL) {
    memcpy(dims_ptr, dimensions, dims_size);
  }

  // Set the parameter to the default value
  if (schema->data_type == RDM_DS_ASCII) {
    strncpy(pd, init_value, schema->size);
  } else if (init_value == NULL) {
    memset(pd, 0, schema->size);
  } else {
    memcpy(pd, init_value, schema->size);
  }

  // Add the new parameter to the driver
  driver->params[pdi].data = pd;
  driver->params[pdi].schema = *schema;
  driver->params[pdi].dims = dims_ptr;
  driver->params[pdi].callback = NULL;
  // driver->params[pdi].context does not need to be set to NULL yet
  ++driver->num_parameters;

  return pd;
}

const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             rdm_pid_t pid, const rdm_pd_schema_t *schema,
                             const rdm_pd_dimensions_t *dimensions,
                             rdm_pid_t alias, size_t offset) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0 && pid <= 0xffff);
  assert(schema != NULL);
  assert(schema->data_type <= 0xdf);
  assert(schema->cc >= RDM_CC_DISC && schema->cc <= RDM_CC_GET_SET);
  assert(schema->size > 0);
  assert(schema->response_handler != NULL);
  if (dimensions != NULL && pid >= 0x8000) {
    assert((dimensions->units >= RDM_UNITS_NONE &&
            dimensions->units <= RDM_UNITS_BYTES) ||
           (dimensions->units >= 0x80 && dimensions->units <= 0xff));
    assert(dimensions->prefix >= RDM_PREFIX_NONE &&
           dimensions->prefix <= RDM_PREFIX_YOTTA);
  }
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  void *pd = NULL;

  // Ensure that the parameter has not already been defined
  uint32_t pdi = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; pdi < driver->num_parameters; ++pdi) {
    if (driver->params[pdi].pid == pid) {
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->params[pdi].pid == pid) {
    return pd;  // Parameter already exists
  }

  // Check if there is space to add a new parameter definition
  if (pdi == RDM_RESPONDER_NUM_PIDS_MAX) {
    return pd;  // No space for new parameter definitions
  }

  // Find the parameter data to alias
  uint32_t apdi = 0;  // Alias parameter data index
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; apdi < driver->num_parameters; ++apdi) {
    if (driver->params[apdi].pid == alias) {
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->params[apdi].pid != alias) {
    return pd;  // The alias has not been declared
  } else if (driver->params[apdi].schema.size < offset) {
    return pd;  // The alias offset is larger than the parameter pdl_size
  }
  pd = driver->params[apdi].data + offset;

  // Add the new parameter to the driver
  driver->params[pdi].data = pd;
  driver->params[pdi].schema = *schema;
  // driver->params[pdi].dims = dims_ptr; // TODO
  driver->params[pdi].callback = NULL;
  // driver->params[pdi].context does not need to be set to NULL yet
  ++driver->num_parameters;

  return pd;
}

bool rdm_pd_add_deterministic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, const rdm_pd_schema_t *schema,
                              const rdm_pd_dimensions_t *dimensions) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0 && pid <= 0xffff);
  assert(schema != NULL);
  assert(schema->data_type <= 0xdf);
  assert(schema->cc >= RDM_CC_DISC && schema->cc <= RDM_CC_GET_SET);
  assert(schema->size > 0);
  assert(schema->response_handler != NULL);
  if (dimensions != NULL && pid >= 0x8000) {
    assert((dimensions->units >= RDM_UNITS_NONE &&
            dimensions->units <= RDM_UNITS_BYTES) ||
           (dimensions->units >= 0x80 && dimensions->units <= 0xff));
    assert(dimensions->prefix >= RDM_PREFIX_NONE &&
           dimensions->prefix <= RDM_PREFIX_YOTTA);
  }
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  bool ret = false;

  // Ensure that the parameter has not already been defined
  uint32_t pdi = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; pdi < driver->num_parameters; ++pdi) {
    if (driver->params[pdi].pid == pid) {
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->params[pdi].pid == pid) {
    return ret;  // Parameter already exists
  }

  // Check if there is space to add a new parameter definition
  if (pdi == RDM_RESPONDER_NUM_PIDS_MAX) {
    return ret;  // No space for new parameter definitions
  }

  // Add the new parameter to the driver
  driver->params[pdi].data = NULL;
  driver->params[pdi].schema = *schema;
  // driver->params[pdi].dims = dims_ptr; // TODO
  driver->params[pdi].callback = NULL;
  // driver->params[pdi].context does not need to be set to NULL yet
  ++driver->num_parameters;
  ret = true;

  return ret;
}

bool rdm_pd_update_response_handler(dmx_port_t dmx_num,
                                    rdm_sub_device_t sub_device, rdm_pid_t pid,
                                    rdm_response_handler_t response_handler) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0);
  assert(response_handler != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  bool ret = false;

  // Find the parameter
  uint32_t pdi = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; pdi < driver->num_parameters; ++pdi) {
    if (driver->params[pdi].pid == pid) {
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->params[pdi].pid != pid) {
    return ret;  // Parameter does not exist
  }

  // The response handler can be updated
  driver->params[pdi].schema.response_handler = response_handler;
  ret = true;

  return ret;
}

bool rdm_pd_update_callback(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, rdm_callback_t callback,
                            void *context) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  bool ret = false;

  // Find the parameter
  uint32_t pdi = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; pdi < driver->num_parameters; ++pdi) {
    if (driver->params[pdi].pid == pid) {
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->params[pdi].pid != pid) {
    return ret;  // Parameter does not exist
  }

  // The callback and context can be updated
  driver->params[pdi].callback = callback;
  driver->params[pdi].context = context;
  ret = true;

  return ret;
}

const void *rdm_pd_get(dmx_port_t dmx_num, rdm_pid_t pid,
                       rdm_sub_device_t sub_device) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  void *pd = NULL;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  // Find parameter data and its descriptor
  for (int i = 0; i < driver->num_parameters; ++i) {
    if (driver->params[i].pid == pid) {
      pd = driver->params[i].data;
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pd;
}

bool rdm_pd_set(dmx_port_t dmx_num, rdm_pid_t pid, rdm_sub_device_t sub_device,
                const void *data, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513 || sub_device == RDM_SUB_DEVICE_ALL);
  assert(pid > 0);
  assert(data != NULL);
  assert(dmx_driver_is_installed(dmx_num));
  
  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  bool ret = false;

  if (size == 0) {
    return ret;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the parameter
  uint32_t pdi = 0;  // Parameter data index
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; pdi < driver->num_parameters; ++pdi) {
    if (driver->params[pdi].pid == pid) {
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (pdi == driver->num_parameters) {
    return ret;  // Requested parameter does not exist
  }

  // Copy the user data to the parameter
  if (driver->params[pdi].data != NULL) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (driver->params[pdi].schema.data_type == RDM_DS_ASCII) {
      strncpy(driver->params[pdi].data, data, size);
    } else {
      memcpy(driver->params[pdi].data, data, size);
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    ret = true;
  }

  return ret;
}

int rdm_pd_enqueue(dmx_port_t dmx_num, rdm_pid_t pid,
                   rdm_sub_device_t sub_device) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513 || sub_device == RDM_SUB_DEVICE_ALL);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, -1,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the parameter
  uint32_t pdi = 0;  // Parameter data index
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; pdi < driver->num_parameters; ++pdi) {
    if (driver->params[pdi].pid == pid) {
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (pdi == driver->num_parameters) {
    return -1;  // Requested parameter does not exist
  }
  
  int ret = -1;
 
  // Enqueue the parameter if it is not already queued
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->rdm_queue_size < RDM_RESPONDER_QUEUE_SIZE_MAX) {
    for (int i = 0; i < driver->rdm_queue_size; ++i) {
      if (driver->rdm_queue[i] == pid) {
        ret = i;  // PID is already queued
        break;
      }
    }
    if (ret == -1) {
      driver->rdm_queue[driver->rdm_queue_size] = pid;
      ret = driver->rdm_queue_size;
      ++driver->rdm_queue_size;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (ret == -1) {
    // This is not a hardware failure so don't set the bootloader flag
    DMX_WARN("Unable to add PID 0x%04x to the RDM queue", pid);
  }

  return ret;
}

bool rdm_pd_get_description(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_sub_device_t sub_device,
                            rdm_pid_description_t *description) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0);
  assert(sub_device < 513);
  assert(description != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  // 0x8000 to 0xFFDF is the allowed range for manufacturer specific PIDs
  if (pid < RDM_PID_MANUFACTURER_SPECIFIC_BEGIN ||
      pid > RDM_PID_MANUFACTURER_SPECIFIC_END) {
    return false;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find parameter data and its descriptor
  bool success = false;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (int i = 0; i < driver->num_parameters; ++i) {
    if (driver->params[i].pid == pid) {
      const rdm_pd_schema_t *schema = &driver->params[i].schema;
      const rdm_pd_dimensions_t *dims = &driver->params[i].dims;
      if (schema != NULL && dims != NULL) {
        description->pid = pid;
        description->pdl_size = schema->size;
        description->data_type = schema->data_type;
        description->cc = schema->cc;
        description->unit = dims->units;
        description->prefix = dims->prefix;
        description->min_value = dims->min_value;
        description->max_value = dims->max_value;
        description->default_value = dims->default_value;
        strncpy(description->description, dims->description, 32);
      }
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return success;
}

uint32_t rdm_pd_list(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                     uint16_t *pids, uint32_t num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  // Stop writes to a null pointer array
  if (pids == NULL) {
    num = 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Copy the PIDs into the buffer
  uint32_t num_pids = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (; num_pids < driver->num_parameters; ++num_pids) {
    if (num_pids < num) {
      pids[num_pids] = driver->params[num_pids].pid;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return num_pids;
}