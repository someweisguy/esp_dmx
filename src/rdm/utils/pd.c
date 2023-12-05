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
  RDM_PD_FLAGS_VARIABLE = 0,
  RDM_PD_FLAGS_NON_VOLATILE,
  RDM_PD_FLAGS_NON_VOLATILE_STAGED,
  RDM_PD_FLAGS_CONST,
};

// TODO: docs
static uint32_t rdm_definition_count = 0;
static struct rdm_pd_dictionary_s {
  const rdm_pd_definition_t *definition;
  rdm_callback_t callback;
  void *context;
} rdm_dictionary[RDM_RESPONDER_NUM_PIDS_MAX];

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
                                         rdm_pid_t pid, int flags) {
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
  entry->id = pid;
  entry->flags = flags;
  entry->is_queued = false;

  return entry;
}

int rdm_pd_set_definition(dmx_port_t dmx_num, rdm_pid_t pid,
                          const rdm_pd_definition_t *definition) {
  assert(dmx_num < DMX_NUM_MAX);
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
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if the definition already exists
  if (rdm_pd_get_definition(dmx_num, pid) != NULL) {
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

bool rdm_pd_set_callback(dmx_port_t dmx_num, rdm_pid_t pid,
                         rdm_callback_t callback, void *context) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

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

const rdm_pd_definition_t *rdm_pd_get_definition(dmx_port_t dmx_num,
                                                 rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Search for and return a pointer to the definition
  for (int i = 0; i < rdm_definition_count; ++i) {
    if (rdm_dictionary[i].definition->pid == pid) {
      return rdm_dictionary[i].definition;
    }
  }

  return NULL;
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
  struct rdm_pd_s *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry != NULL) {
    return entry->data;
  }

  // Return early if there is no heap space available
  if (driver->rdm.heap_available < definition->alloc_size) {
    return NULL;
  }

  // Return early if there are no available entries
  int flags = RDM_PD_FLAGS_VARIABLE;
  flags |= non_volatile ? RDM_PD_FLAGS_NON_VOLATILE : 0;
  entry = rdm_pd_add_entry(dmx_num, sub_device, pid, flags);
  if (entry == NULL) {
    return NULL;
  }

  // Decrement the heap pointer
  driver->rdm.heap_available -= definition->alloc_size;
  driver->rdm.heap_ptr -= definition->alloc_size;

  // Assign the parameter data
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
  assert(definition->alloc_size <= offset + alias_def->alloc_size);

  // Return early if the variable already exists
  struct rdm_pd_s *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry != NULL) {
    return entry->data;
  }

  // Return early if the alias variable does not exists or does not have space
  struct rdm_pd_s *alias_entry = rdm_pd_get_entry(dmx_num, sub_device, alias);
  if (alias_entry == NULL) {
    return NULL;
  }

  // Non-volatile parameters cannot alias const parameters
  if (alias_entry->flags == RDM_PD_FLAGS_CONST && non_volatile) {
    non_volatile = false;
  }

  // Return early if there is not enough space for a new entry
  int flags = non_volatile ? RDM_PD_FLAGS_NON_VOLATILE : alias_entry->flags;
  entry = rdm_pd_add_entry(dmx_num, sub_device, pid, flags);
  if (entry == NULL) {
    return NULL;
  }

  // Assign the parameter
  entry->data = alias_entry->data + offset;

  return entry->data;
}

const void *rdm_pd_add_const(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             rdm_pid_t pid, void *data) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(pid > 0);
  assert(dmx_driver_is_installed(dmx_num));

  // Return early if the variable already exists
  struct rdm_pd_s *entry = rdm_pd_get_entry(dmx_num, sub_device, pid);
  if (entry != NULL) {
    return entry->data;
  }

  // Return early if there is not enough space for a new entry
  entry = rdm_pd_add_entry(dmx_num, sub_device, pid, RDM_PD_FLAGS_CONST);
  if (entry == NULL) {
    return NULL;
  }

  // Assign the parameter
  entry->data = data;

  return data;
}

const void *rdm_pd_get_ptr(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
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

  const void *pd_ptr = rdm_pd_get_ptr(dmx_num, sub_device, pid);
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
  if (entry == NULL || entry->flags == RDM_PD_FLAGS_CONST) {
    return 0;
  }

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(entry->data, source, size);
  if (entry->flags == RDM_PD_FLAGS_NON_VOLATILE) {
    entry->flags = RDM_PD_FLAGS_NON_VOLATILE_STAGED;
    ++dmx_driver[dmx_num]->rdm.staged_count;
  }
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
  if (entry == NULL || entry->flags == RDM_PD_FLAGS_CONST) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(entry->data, source, size);
  entry->is_queued = true;
  ++driver->rdm.queue_count;
  if (entry->flags == RDM_PD_FLAGS_NON_VOLATILE) {
    entry->flags = RDM_PD_FLAGS_NON_VOLATILE_STAGED;
    ++dmx_driver[dmx_num]->rdm.staged_count;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

rdm_pid_t rdm_pd_queue_pop(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  rdm_pid_t pid = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->rdm.queue_count > 0) {
    for (int i = 0; i < driver->rdm.parameter_count; ++i) {
      if (driver->rdm.parameter[i].is_queued) {
        pid = driver->rdm.parameter[i].id;
        driver->rdm.parameter[i].is_queued = false;
        --driver->rdm.queue_count;
        break;
      }
    }
  }
  driver->rdm.previous_popped = pid;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}

uint8_t rdm_pd_queue_get_size(dmx_port_t dmx_num) {
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

rdm_pid_t rdm_pd_queue_get_last_message(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  rdm_pid_t pid;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  pid = driver->rdm.previous_popped;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}

rdm_pid_t rdm_pd_nvs_commit(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Guard against unnecessarily iterating through all parameters
  if (driver->rdm.staged_count == 0) {
    return 0;
  }

  // Iterate through parameters and commit the first found value to NVS
  for (int i = 0; i < driver->rdm.parameter_count; ++i) {
    if (driver->rdm.parameter[i].flags == RDM_PD_FLAGS_NON_VOLATILE_STAGED) {
      const rdm_pid_t pid = driver->rdm.parameter[i].id;
      const rdm_pd_definition_t *def = rdm_pd_get_definition(dmx_num, pid);
      // TODO: implement sub-devices
      bool success = dmx_nvs_set(dmx_num, pid, RDM_SUB_DEVICE_ROOT, def->ds,
                  driver->rdm.parameter[i].data, def->alloc_size);
      if (success) {
        taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
        driver->rdm.parameter[i].flags = RDM_PD_FLAGS_NON_VOLATILE;
        --driver->rdm.staged_count;
        taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      }
      return driver->rdm.parameter[i].id;
    }
  }

  return 0;
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
    const void *pd = rdm_pd_get_ptr(dmx_num, header->sub_device, header->pid);
    format = definition->get.response.format;
    return rdm_write_ack(dmx_num, header, format, pd, definition->alloc_size);
  } else {
    // Get the parameter from the request and write it to the RDM driver
    uint8_t pd[231];
    format = definition->set.request.format;
    size_t size = rdm_read_pd(dmx_num, format, pd, header->pdl);
    rdm_pd_set(dmx_num, header->sub_device, header->pid, pd, size);
    return rdm_write_ack(dmx_num, header, NULL, NULL, 0);
  }
}