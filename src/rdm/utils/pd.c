#include "rdm/utils/pd.h"

#include <ctype.h>
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

enum rdm_pd_flags_e {
  RDM_PD_FLAGS_UPDATED = BIT0,
  RDM_PD_FLAGS_QUEUED = BIT1,
  RDM_PD_FLAGS_NON_VOLATILE = BIT2,
  RDM_PD_FLAGS_CONST = BIT3,
};

static struct rdm_pd_s *rdm_pd_get_entry(dmx_port_t dmx_num,
                                         rdm_sub_device_t sub_device,
                                         rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO: Implement sub-devices
  assert(sub_device == 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  for (int i = 0; i < driver->rdm.parameter_count; ++i) {
    if (driver->rdm.parameter[i].id == pid) {
      return &driver->rdm.parameter[i];
    }
  }

  return NULL;
}

static struct rdm_pd_s *rdm_pd_add_entry(dmx_port_t dmx_num,
                                         rdm_sub_device_t sub_device,
                                         rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO: Implement sub-devices
  assert(sub_device == 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  if (driver->rdm.parameter_count == RDM_RESPONDER_NUM_PIDS_MAX) {
    return NULL;
  }

  struct rdm_pd_s *entry = &driver->rdm.parameter[driver->rdm.parameter_count];
  ++driver->rdm.parameter_count;

  return entry;
}

int rdm_pd_set_definition(dmx_port_t dmx_num, rdm_pid_t pid,
                          const rdm_pd_definition_t *definition) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(definition != NULL);
  assert(definition->pid > 0);
  assert((definition->ds >= RDM_DS_NOT_DEFINED &&
          definition->ds <= RDM_DS_SIGNED_DWORD) ||
         (definition->ds >= 0x80 || definition->ds <= 0xdf));
  assert(definition->pid_cc >= RDM_CC_DISC &&
         definition->pid_cc <= RDM_CC_GET_SET);
  assert(definition->response_handler != NULL);
  assert(!(definition->pid >= RDM_PID_MANUFACTURER_SPECIFIC_BEGIN &&
           definition->pid <= RDM_PID_MANUFACTURER_SPECIFIC_END) ||
         definition->description == NULL);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Return early if the definition already exists
  if (rdm_pd_get_definition(dmx_num, pid) != NULL) {
    return 0;
  }

  // Add the definition and increment the definition count
  const uint32_t i = driver->rdm.definition_count;
  driver->rdm.dictionary[i].definition = definition;
  driver->rdm.dictionary[i].callback = NULL;
  // Don't need to set the value of the context pointer yet
  ++driver->rdm.definition_count;

  return driver->rdm.definition_count;
}

bool rdm_pd_set_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                         rdm_callback_t callback, void *context) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Search for the definition and add the callback
  for (int i = 0; i < driver->rdm.definition_count; ++i) {
    if (driver->rdm.dictionary[i].definition->pid == pid) {
      driver->rdm.dictionary[i].callback = callback;
      driver->rdm.dictionary[i].context = context;
      return true;
    }
  }

  return false;
}

const rdm_pd_definition_t *rdm_pd_get_definition(dmx_port_t dmx_num,
                                                 rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Search for and return a pointer to the definition
  for (int i = 0; i < driver->rdm.definition_count; ++i) {
    if (driver->rdm.dictionary[i].definition->pid == pid) {
      return driver->rdm.dictionary[i].definition;
    }
  }

  return NULL;
}

const void *rdm_pd_add_variable(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                                rdm_pid_t pid, bool non_volatile,
                                const void *init_value, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(size > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Fail early if a definition has not been defined or PDL == 0
  const rdm_pd_definition_t *definition = rdm_pd_get_definition(dmx_num, pid);
  assert(definition != NULL && definition->alloc_size > 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Return early if the variable already exists
  if (rdm_pd_get_entry(dmx_num, sub_device, pid) != NULL) {
    return NULL;
  }

  // Return early if there is no heap space available
  if (driver->rdm.heap_available < definition->alloc_size) {
    return NULL;
  }

  // Return early if there are no available entries
  struct rdm_pd_s *entry = rdm_pd_add_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return NULL;
  }

  // Decrement the heap pointer
  driver->rdm.heap_available -= definition->alloc_size;
  driver->rdm.heap_ptr -= definition->alloc_size;

  // Assign the parameter
  entry->id = pid;
  entry->flags = non_volatile ? RDM_PD_FLAGS_NON_VOLATILE : 0;
  entry->data = driver->rdm.heap_ptr;

  // Clamp the size parameter
  if (size > definition->alloc_size) {
    size = definition->alloc_size;
  }

  // Set the initial value of the variable
  if (definition->ds == RDM_DS_ASCII) {
    strncpy(entry->data, init_value, size);
  } else if (init_value != NULL) {
    memcpy(entry->data, init_value, size);
  } else {
    memset(entry->data, 0, size);
  }

  return entry->data;
}

const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             rdm_pid_t pid, bool non_volatile, rdm_pid_t alias,
                             size_t offset) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Fail early if a definition has not been defined or PDL == 0
  const rdm_pd_definition_t *definition = rdm_pd_get_definition(dmx_num, pid);
  assert(definition != NULL && definition->alloc_size > 0);

  // Fail early if the alias has not been defined or PDL == 0
  const rdm_pd_definition_t *alias_def = rdm_pd_get_definition(dmx_num, alias);
  assert(alias_def != NULL && alias_def->alloc_size > 0);

  // Fail early if there is no space for the alias parameter data
  assert(definition->alloc_size < offset + alias_def->alloc_size);

  // Return early if the variable already exists
  if (rdm_pd_get_entry(dmx_num, sub_device, pid) != NULL) {
    return NULL;
  }

  // Return early if the alias variable does not exists or does not have space
  struct rdm_pd_s *alias_entry = rdm_pd_get_entry(dmx_num, sub_device, alias);
  if (alias_entry == NULL) {
    return NULL;
  }

  // Return early if there is not enough space for a new entry
  struct rdm_pd_s *entry = rdm_pd_add_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return NULL;
  }

  // Assign the parameter
  entry->id = pid;
  entry->data = alias_entry->data + offset;
  entry->flags = non_volatile ? RDM_PD_FLAGS_NON_VOLATILE : 0;

  return entry->data;
}

const void *rdm_pd_add_static(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid, void *data) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if the variable already exists
  if (rdm_pd_get_pointer(dmx_num, sub_device, pid) != NULL) {
    return NULL;
  }

  // Return early if there is not enough space for a new entry
  struct rdm_pd_s *entry = rdm_pd_add_entry(dmx_num, sub_device, pid);
  if (entry == NULL) {
    return NULL;
  }

  // Assign the parameter
  entry->id = pid;
  entry->flags = RDM_PD_FLAGS_CONST;
  entry->data = data;

  return data;
}

const void *rdm_pd_get_pointer(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                               rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Find the parameter and return it
  for (int i = 0; i < driver->rdm.parameter_count; ++i) {
    if (driver->rdm.parameter[i].id == pid) {
      return driver->rdm.parameter[i].data;
    }
  }

  return NULL;
}

size_t rdm_pd_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                  rdm_pid_t pid, void *destination, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  const rdm_pd_definition_t *definition = rdm_pd_get_definition(dmx_num, pid);
  assert(definition != NULL);

  const void *pd_ptr = rdm_pd_get_pointer(dmx_num, sub_device, pid);
  if (pd_ptr == NULL) {
    return 0;
  }

  if (size > definition->alloc_size) {
    size = definition->alloc_size;
  }

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(destination, pd_ptr, size);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

size_t rdm_pd_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                  rdm_pid_t pid, const void *source, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  const rdm_pd_definition_t *definition = rdm_pd_get_definition(dmx_num, pid);
  assert(definition != NULL);

  // Return early if there is nothing to write
  if (source == NULL || size == 0) {
    return 0;
  }

  // Clamp the write size to the definition size
  if (size > definition->alloc_size) {
    size = definition->alloc_size;
  }

  struct rdm_pd_s *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL || entry->flags & RDM_PD_FLAGS_CONST) {
    return 0;
  }

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(entry->data, source, size);
  entry->flags |= RDM_PD_FLAGS_UPDATED;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

size_t rdm_pd_set_and_queue(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, const void *source, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  const rdm_pd_definition_t *definition = rdm_pd_get_definition(dmx_num, pid);
  assert(definition != NULL);

  // Return early if there is nothing to write
  if (source == NULL || size == 0) {
    return 0;
  }

  // Clamp the write size to the definition size
  if (size > definition->alloc_size) {
    size = definition->alloc_size;
  }

  struct rdm_pd_s *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry == NULL || entry->flags & RDM_PD_FLAGS_CONST) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(entry->data, source, size);
  entry->flags |= RDM_PD_FLAGS_UPDATED | RDM_PD_FLAGS_QUEUED;
  ++driver->rdm.queue_size;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

uint8_t rdm_pd_queue_size(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  uint32_t size;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  size = driver->rdm.queue_size;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (size > 255) {
    size = 255;  // RDM requires queue size to be clamped
  }

  return size;
}

rdm_pid_t rdm_pd_queue_pop(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  rdm_pid_t pid = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->rdm.queue_size > 0) {
    for (int i = 0; i < driver->rdm.parameter_count; ++i) {
      if (driver->rdm.parameter[i].flags & RDM_PD_FLAGS_QUEUED) {
        pid = driver->rdm.parameter[i].id;
        driver->rdm.parameter[i].flags &= ~RDM_PD_FLAGS_QUEUED;
        --driver->rdm.queue_size;
        break;
      }
    }
  }
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}

rdm_pid_t rdm_queue_get_last_sent(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  rdm_pid_t pid;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  pid = driver->rdm.queue_last_sent;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}