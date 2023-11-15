#include "rdm/utils/pd.h"

#include <string.h>

#include "dmx/bus_ctl.h"
#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"
#include "endian.h"
#include "esp_dmx.h"
#include "rdm/types.h"
#include "rdm/utils/bus_ctl.h"
#include "rdm/utils/uid.h"

const void *rdm_pd_add_new(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           rdm_pid_t pid, const rdm_pd_definition_t *def,
                           const void *init_value) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0 && pid <= 0xffff);
  assert(def != NULL);
  assert(def->schema.data_type <= 0xdf);
  assert(def->schema.cc >= RDM_CC_DISC && def->schema.cc <= RDM_CC_GET_SET);
  assert(def->pd_size > 0);
  assert(def->response_handler != NULL);
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
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const size_t pdl_available = driver->pd_size - driver->pd_head;
  if (def->pd_size <= pdl_available) {
    pd = driver->pd + driver->pd_head;
    driver->pd_head += def->pd_size;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (def->pd_size > pdl_available) {
    return pd;  // No more reservable parameter data space
  }

  // Set the parameter to the default value
  if (def->schema.data_type == RDM_DS_ASCII) {
    strncpy(pd, init_value, def->schema.pdl_size);
  } else if (init_value == NULL) {
    memset(pd, 0, def->schema.pdl_size);
  } else {
    memcpy(pd, init_value, def->schema.pdl_size);
  }

  // Add the new parameter to the driver
  driver->params[pdi].pid = pid;
  driver->params[pdi].data = pd;
  driver->params[pdi].definition = *def;
  driver->params[pdi].callback = NULL;
  // driver->params[pdi].context does not need to be set to NULL yet
  ++driver->num_parameters;

  return pd;
}

const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             rdm_pid_t pid, const rdm_pd_definition_t *def,
                             rdm_pid_t alias, size_t offset) {
  assert(sub_device < 513);
  assert(pid > 0 && pid <= 0xffff);
  assert(def != NULL);
  assert(def->schema.data_type <= 0xdf);
  assert(def->schema.cc >= RDM_CC_DISC && def->schema.cc <= RDM_CC_GET_SET);
  assert(def->pd_size > 0);
  assert(def->response_handler != NULL);
  assert(alias > 0 && alias <= 0xffff);
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
  } else if (driver->params[apdi].definition.pd_size < offset) {
    return pd;  // The alias offset is larger than the parameter pdl_size
  }
  pd = driver->params[apdi].data + offset;

  // Add the new parameter to the driver
  driver->params[pdi].pid = pid;
  driver->params[pdi].data = pd;
  driver->params[pdi].definition = *def;
  driver->params[pdi].callback = NULL;
  // driver->params[pdi].context does not need to be set to NULL yet
  ++driver->num_parameters;

  return pd;
}

bool rdm_pd_add_deterministic(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, const rdm_pd_definition_t *def) {
  assert(sub_device < 513);
  assert(pid > 0 && pid <= 0xffff);
  assert(def != NULL);
  assert(def->schema.data_type <= 0xdf);
  assert(def->schema.cc >= RDM_CC_DISC && def->schema.cc <= RDM_CC_GET_SET);
  assert(def->response_handler != NULL);
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
  driver->params[pdi].pid = pid;
  driver->params[pdi].data = NULL;
  driver->params[pdi].definition = *def;
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
  driver->params[pdi].definition.response_handler = response_handler;
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

bool rdm_pd_exists(dmx_port_t dmx_num, rdm_pid_t pid,
                   rdm_sub_device_t sub_device) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the parameter data
  bool pd_exists = false;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (int i = 0; i < driver->num_parameters; ++i) {
    if (driver->params[i].pid == pid) {
      pd_exists = true;
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pd_exists;
}

void *rdm_pd_get(dmx_port_t dmx_num, rdm_pid_t pid,
                 rdm_sub_device_t sub_device) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the parameter data
  void *pd = NULL;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (int i = 0; i < driver->num_parameters; ++i) {
    if (driver->params[i].pid == pid) {
      pd = driver->params[i].data;
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pd;
}

size_t rdm_pd_set(dmx_port_t dmx_num, rdm_pid_t pid,
                  rdm_sub_device_t sub_device, const void *data, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513 || sub_device == RDM_SUB_DEVICE_ALL);
  assert(pid > 0);
  assert(data != NULL);
  assert(dmx_driver_is_installed(dmx_num));
  
  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  size_t written = 0;

  // Return early if nothing to write
  if (size == 0) {
    return written;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the parameter and copy the data
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (int i = 0; i < driver->num_parameters; ++i) {
    if (driver->params[i].pid == pid) {
      void *pd = driver->params[i].data;
      if (pd == NULL) {
        break;  // Parameter data does not exist
      }
      if (driver->params[i].definition.schema.data_type == RDM_DS_ASCII) {
        strncpy(pd, data, size);
      } else {
        memcpy(pd, data, size);
      }
      written = size;
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return written;
}

size_t rdm_pd_set_and_queue(dmx_port_t dmx_num, rdm_pid_t pid,
                            rdm_sub_device_t sub_device, const void *data,
                            size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513 || sub_device == RDM_SUB_DEVICE_ALL);
  assert(pid > 0);
  assert(data != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  const size_t written = rdm_pd_set(dmx_num, pid, sub_device, data, size);
  if (written > 0) {
    dmx_driver_t *const driver = dmx_driver[dmx_num];

    // Enqueue the parameter if it is not already queued
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (driver->rdm_queue_size < RDM_RESPONDER_QUEUE_SIZE_MAX) {
      bool pid_already_queued = false;
      for (int i = 0; i < driver->rdm_queue_size; ++i) {
        if (driver->rdm_queue[i] == pid) {
          pid_already_queued = true;
          break;
        }
      }
      if (!pid_already_queued) {
        driver->rdm_queue[driver->rdm_queue_size] = pid;
        ++driver->rdm_queue_size;
      }
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  return written;
}

const rdm_pd_schema_t *rdm_pd_get_schema(dmx_port_t dmx_num, rdm_pid_t pid,
                                         rdm_sub_device_t sub_device) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the parameter schema
  rdm_pd_schema_t *schema = NULL;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (int i = 0; i < driver->num_parameters; ++i) {
    if (driver->params[i].pid == pid) {
      schema = &driver->params[i].definition.schema;
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return schema;
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
      const rdm_pd_definition_t *def = &driver->params[i].definition;
      if (def != NULL) {
        description->pid = pid;
        description->pdl_size = def->schema.pdl_size;
        description->data_type = def->schema.data_type;
        description->cc = def->schema.cc;
        description->unit = def->units;
        description->prefix = def->prefix;
        description->min_value = def->schema.min_value;
        description->max_value = def->schema.max_value;
        description->default_value = def->default_value;
        strncpy(description->description, def->description, 32);
        success = true;
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

int rdm_pd_call_response_handler(dmx_port_t dmx_num, rdm_header_t *header,
                                 void *pd, uint8_t *pdl_out) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(header != NULL);
  assert(pdl_out != NULL);
  assert(pd != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the parameter schema
  const rdm_pd_definition_t *def = NULL;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (int i = 0; i < driver->num_parameters; ++i) {
    if (driver->params[i].pid == header->pid) {
      def = &driver->params[i].definition;
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Guard against unknown PID
  if (def == NULL) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_UNKNOWN_PID);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  const rdm_pd_schema_t *schema = &def->schema;
  return def->response_handler(dmx_num, header, pd, pdl_out, schema);
}

int rdm_response_handler_simple(dmx_port_t dmx_num, rdm_header_t *header, void *pd,
                           uint8_t *pdl_out, const rdm_pd_schema_t *schema) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // TODO: if schema->data_type is byte/word/dword, check min/max

  void *data = rdm_pd_get(dmx_num, header->pid, header->sub_device);
  if (header->cc == RDM_CC_GET_COMMAND) {
    *pdl_out = rdm_emplace(pd, schema->format, data, 231, false);
  } else {
    rdm_emplace(data, schema->format, pd, header->pdl, true);
  }

  return RDM_RESPONSE_TYPE_ACK;
}
