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

static int rdm_pd_get_index(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid) {
  int pd_index = 0;
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  for (; pd_index < driver->rdm.param_count; ++pd_index) {
    if (driver->rdm.params[pd_index].definition->pid == pid) {
      break;
    }
  }
  return pd_index;
}

static size_t rdm_pd_set_variable(dmx_port_t dmx_num,
                                  rdm_sub_device_t sub_device, rdm_pid_t pid,
                                  const void *data, size_t size, bool enqueue) {
  // Guard against writing to null pointer
  if (data == NULL) {
    size = 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Ensure the parameter has already been defined
  const int pd_index = rdm_pd_get_index(dmx_num, sub_device, pid);
  if (pd_index == driver->rdm.param_count) {
    return false;
  }
  struct rdm_pd_vector_s *const pd_vector = &driver->rdm.params[pd_index];

  // Attempt to memcpy
  if (pd_vector->storage_type == RDM_PD_FLAG_VARIABLE) {
    if (size > pd_vector->size) {
      size = pd_vector->size;
    }
    if (size > 0) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      memcpy(pd_vector->data.value, data, size);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      int flags = RDM_PD_FLAG_UPDATED | (enqueue ? RDM_PD_FLAG_QUEUED : 0);
      pd_vector->flags |= flags;
    }
  } else {
    size = 0;  // Don't write to a non-variable value
  }

  return size;
}

const void *rdm_pd_add_variable(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                                const rdm_pd_definition_t *definition,
                                const void *init_value, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
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

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, NULL,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Ensure the parameter has not already been defined
  const int pd_index = rdm_pd_get_index(dmx_num, sub_device, definition->pid);
  if (pd_index < dmx_driver[dmx_num]->rdm.param_count) {
    return NULL;
  }

  // Ensure there is space for new parameter definitions
  if (pd_index == RDM_RESPONDER_NUM_PIDS_MAX) {
    return NULL;
  }

  // Reserve space for the parameter in the RDM driver
  void *pd = NULL;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (size > 0 && size <= driver->rdm.pd_available) {
    // Allocate space in the parameter data buffer
    pd = driver->rdm.pd;
    driver->rdm.pd_available -= size;
    driver->rdm.pd += size;

    // Set the parameter to the default value
    if (definition->ds == RDM_DS_ASCII) {
      strncpy(pd, init_value, size);
    } else if (init_value != NULL) {
      memcpy(pd, init_value, size);
    } else {
      memset(pd, 0, size);
    }
  }
  if (pd != NULL) {
    // Update the parameter vectors
    struct rdm_pd_vector_s *const pd_vector = &driver->rdm.params[pd_index];
    pd_vector->callback = NULL;
    // Don't need to set context to NULL until callback is not NULL
    pd_vector->flags = 0;
    pd_vector->size = size;
    pd_vector->data.value = pd;
    pd_vector->storage_type = RDM_PD_FLAG_VARIABLE;
    pd_vector->definition = definition;
    ++driver->rdm.param_count;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pd;
}

const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             const rdm_pd_definition_t *definition,
                             rdm_pid_t alias, size_t offset, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
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
  assert(alias > 0);
  assert(size > 0);

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, NULL,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Ensure the parameter has not already been defined
  const int pd_index = rdm_pd_get_index(dmx_num, sub_device, definition->pid);
  if (pd_index < dmx_driver[dmx_num]->rdm.param_count) {
    return NULL;
  }

  // Ensure there is space for new parameter definitions
  if (pd_index == RDM_RESPONDER_NUM_PIDS_MAX) {
    return NULL;
  }

  // Add the parameter vector
  void *pd = NULL;
  // Find the parameter data alias
  for (int i = 0; i < driver->rdm.param_count; ++i) {
    if (driver->rdm.params[i].definition->pid == alias) {
      if (offset + size >= driver->rdm.params[i].size) {
        break;  // Alias offset or size is too big
      }
      if (driver->rdm.params[i].definition->non_volatile &&
          definition->non_volatile) {
        break;  // Non-volatile parameter can't alias non-volatile parameter
      }
      pd = driver->rdm.params[i].data.value + offset;
      break;
    }
  }
  if (pd != NULL) {
    // Update the parameter vectors
    struct rdm_pd_vector_s *const pd_vector = &driver->rdm.params[pd_index];
    pd_vector->callback = NULL;
    // Don't need to set context to NULL until callback is not NULL
    pd_vector->flags = 0;
    pd_vector->size = size;
    pd_vector->data.value = pd;
    pd_vector->storage_type = RDM_PD_FLAG_VARIABLE;
    pd_vector->definition = definition;
    ++driver->rdm.param_count;
  }

  return pd;
}

rdm_pd_getter_t rdm_pd_add_deterministic(dmx_port_t dmx_num,
                                         rdm_sub_device_t sub_device,
                                         const rdm_pd_definition_t *definition,
                                         rdm_pd_getter_t getter) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
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
  assert(getter != NULL);

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, NULL,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Ensure the parameter has not already been defined
  const int pd_index = rdm_pd_get_index(dmx_num, sub_device, definition->pid);
  if (pd_index < dmx_driver[dmx_num]->rdm.param_count) {
    return NULL;
  }

  // Ensure there is space for new parameter definitions
  if (pd_index == RDM_RESPONDER_NUM_PIDS_MAX) {
    return NULL;
  }

  // Add the parameter vector
  struct rdm_pd_vector_s *const pd_vector = &driver->rdm.params[pd_index];
  pd_vector->callback = NULL;
  // Don't need to set context to NULL until callback is not NULL
  pd_vector->flags = 0;
  pd_vector->size = 0;
  pd_vector->data.getter = getter;
  pd_vector->storage_type = RDM_PD_FLAG_DETERMINISTIC;
  pd_vector->definition = definition;
  ++driver->rdm.param_count;

  return getter;
}

bool rdm_pd_update_callback(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, rdm_callback_t callback,
                            void *context) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Ensure the parameter has already been defined
  const int pd_index = rdm_pd_get_index(dmx_num, sub_device, pid);
  if (pd_index == driver->rdm.param_count) {
    return false;
  }
  struct rdm_pd_vector_s *const pd_vector = &driver->rdm.params[pd_index];

  // Update the callback and context
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  pd_vector->callback = callback;
  pd_vector->context = context;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return true;
}

bool rdm_pd_exists(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                   rdm_pid_t pid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  return rdm_pd_get_index(dmx_num, sub_device, pid) <
         dmx_driver[dmx_num]->rdm.param_count;
}

uint32_t rdm_pd_list(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                     void *destination, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Guard against writes to null pointer
  if (destination == NULL) {
    size = 0;
  }

  uint32_t count = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  // FIXME: implement
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return count;
}

const void *rdm_pd_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                       rdm_pid_t pid, void *destination, size_t dest_size,
                       const void *args) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Ensure the parameter has already been defined
  const int pd_index = rdm_pd_get_index(dmx_num, sub_device, pid);
  if (pd_index == driver->rdm.param_count) {
    return NULL;
  }
  struct rdm_pd_vector_s *const pd_vector = &driver->rdm.params[pd_index];

  // Guard against writing to null pointer
  if (destination == NULL) {
    dest_size = 0;
  }

  // Return early if there is nowhere to write data
  const void *ret = driver->rdm.params[pd_index].data.value;
  if (dest_size == 0) {
    return ret;
  }

  // Copy the parameter data to the destination buffer
  if (pd_vector->storage_type == RDM_PD_FLAG_VARIABLE) {
    if (dest_size > pd_vector->size) {
      dest_size = pd_vector->size;  // Guard against out-of-bounds error
    }
    if (dest_size > 0) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      memcpy(destination, pd_vector->data.value, dest_size);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  } else if (pd_vector->storage_type == RDM_PD_FLAG_DETERMINISTIC) {
    pd_vector->data.getter(dmx_num, sub_device, destination, dest_size, args);
  } else {
    __unreachable();
  }

  return ret;
}

size_t rdm_pd_get_size(const char *format) {
  size_t param_size = 0;
  for (const char *c = format; *c != '\0'; ++c) {
    size_t field_size = 0;
    switch (*c) {
      case 'b':
      case 'B':
        field_size = sizeof(uint8_t);
        break;
      case 'w':
      case 'W':
        field_size = sizeof(uint16_t);
        break;
      case 'd':
      case 'D':
        field_size = sizeof(uint32_t);
        break;
      case 'v':
      case 'V':
        if (c[1] != '\0' && c[1] != '$') {
          return 0;  // Optional UID not at end of parameter
        }
        field_size = sizeof(rdm_uid_t);
        break;
      case 'u':
      case 'U':
        field_size = sizeof(rdm_uid_t);
        break;
      case 'a':
      case 'A':
        if (c[1] != '\0' && c[1] != '$') {
          return 0;  // ASCII not at end of parameter
        }
        field_size = 32;  // Size of ASCII string
        break;
      case '#':
        ++c;  // Ignore '#' character
        size_t num_chars = 0;
        for (; num_chars <= 16; ++num_chars) {
          if (!isxdigit((int)c[num_chars])) break;
        }
        if (num_chars > 16) {
          return 0;  // Integer literal too big
        }
        field_size = (num_chars / 2) + (num_chars % 2);
        c += num_chars;  // Skip integer literal and 'h' terminator
        break;
      case '$':
        if (c[1] != '\0') {
          return 0;  // Improper end-of-parameter anchor
        }
        break;
      default:
        return 0;  // Invalid character in format string
    }
    param_size += field_size;

    if (param_size > 231) {
      return 0;  // Parameter is too big
    }
  }

  return param_size;
}

size_t rdm_pd_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                  rdm_pid_t pid, const void *data, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  const bool enqueue = false;
  return rdm_pd_set_variable(dmx_num, sub_device, pid, data, size, enqueue);
}

size_t rdm_pd_set_and_queue(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, const void *data, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  // TODO
  DMX_CHECK(sub_device == RDM_SUB_DEVICE_ROOT, 0,
            "Multiple sub-devices are not yet supported.");

  const bool enqueue = true;
  return rdm_pd_set_variable(dmx_num, sub_device, pid, data, size, enqueue);
}

rdm_response_type_t rdm_pd_handle_response(dmx_port_t dmx_num,
                                           rdm_header_t *header, void *pd,
                                           uint8_t *pdl_out) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(header != NULL);
  assert(pdl_out != NULL);
  assert(pd != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Ensure the source UID is valid and packet targets this device
  rdm_uid_t this_uid;
  rdm_uid_get(dmx_num, &this_uid);
  if (rdm_uid_is_broadcast(&header->src_uid) ||
      !rdm_uid_is_target(&this_uid, &header->dest_uid)) {
    return RDM_RESPONSE_TYPE_NONE;
  }

  // Ensure header is formatted properly
  if (header->pdl > 231 || header->port_id == 0 ||
      (header->cc != RDM_CC_DISC_COMMAND && header->cc != RDM_CC_GET_COMMAND &&
       header->cc != RDM_CC_SET_COMMAND)) {
    *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_FORMAT_ERROR);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Ensure the parameter has already been defined
  int pd_index = rdm_pd_get_index(dmx_num, header->sub_device, header->pid);
  if (pd_index == driver->rdm.param_count) {
    *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_UNKNOWN_PID);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }
  const rdm_pd_definition_t *def = &driver->rdm.params[pd_index].definition;

  // Ensure the command class is valid
  if ((header->cc == RDM_CC_DISC_COMMAND && !(def->pid_cc & RDM_CC_DISC)) ||
      (header->cc == RDM_CC_GET_COMMAND && !(def->pid_cc & RDM_CC_GET)) ||
      (header->cc == RDM_CC_SET_COMMAND && !(def->pid_cc & RDM_CC_SET))) {
    *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_UNSUPPORTED_COMMAND_CLASS);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // The header is valid - call the response handler
  return def->response_handler(dmx_num, &def, header, pd, pdl_out);
}

static size_t rdm_pd_encode(void *destination, size_t len, const char *format,
                            const void *source, bool encode_nulls) {
  assert(destination != NULL);
  assert(format != NULL);
  assert(source != NULL);

  // Get the max size of the parameter
  const size_t format_size = rdm_pd_get_size(format);
  assert(format_size > 0);

  // Determine how many parameters to write to the destination
  uint32_t num_params;
  switch (format[strlen(format)]) {
    case '$':
    case 'a':
    case 'A':
    case 'v':
    case 'V':
      num_params = 1;  // Parameter is singleton
      break;
    default:
      if (format_size <= len) {
        num_params = len / format_size;
      } else {
        num_params = 0;
      }
      break;
  }

  size_t written = 0;
  for (int params_written = 0; params_written < num_params; ++params_written) {
    for (char c = *format; c != '\0'; c = *(++format)) {
      size_t field_size;
      if (c == 'b' || c == 'B') {
        field_size = sizeof(uint8_t);
        memmove(destination, source, field_size);
      } else if (c == 'w' || c == 'W') {
        uint16_t temp;
        field_size = sizeof(uint16_t);
        memcpy(&temp, source, field_size);
        temp = bswap16(temp);
        memcpy(destination, &temp, field_size);
      } else if (c == 'd' || c == 'D') {
        uint32_t temp;
        field_size = sizeof(uint32_t);
        memcpy(&temp, source, field_size);
        temp = bswap32(temp);
        memcpy(destination, &temp, field_size);
      } else if (c == 'u' || c == 'U' || c == 'v' || c == 'V') {
        rdm_uid_t temp;
        field_size = sizeof(rdm_uid_t);
        memcpy(&temp, source, field_size);
        if ((c == 'v' || c == 'V') && !encode_nulls && rdm_uid_is_null(&temp)) {
          break;
        }
        temp.man_id = bswap16(temp.man_id);
        temp.dev_id = bswap32(temp.dev_id);
        memcpy(destination, &temp, field_size);
      } else if (c == 'a' || c == 'A') {
        field_size = strnlen(source, 32);
        memmove(destination, source, field_size);
        if (encode_nulls) {
          memset(destination + field_size, '\0', 1);
        }
        field_size += (encode_nulls ? 1 : 0);
      } else if (c == '#') {
        ++format;
        char *end_ptr;
        uint64_t temp = strtol(format, &end_ptr, 16);
        field_size = ((end_ptr - format) / 2) + ((end_ptr - format) % 2);
        for (int i = 0, j = field_size - 1; i < field_size; ++i, --j) {
          ((uint8_t *)destination)[i] = ((uint8_t *)&temp)[j];
        }
        format = end_ptr;
      } else if (c == '$') {
        break;
      } else {
        __unreachable();
      }
      destination += field_size;
      source += field_size;
      written += field_size;
    }
  }

  return written;
}

size_t rdm_pd_serialize(void *destination, size_t len, const char *format,
                        const void *source) {
  assert(destination != NULL);
  assert(format != NULL);
  assert(source != NULL);

  return rdm_pd_encode(destination, len, format, source, false);
}

size_t rdm_pd_deserialize(void *destination, size_t len, const char *format,
                          const void *source) {
  assert(destination != NULL);
  assert(format != NULL);
  assert(source != NULL);

  return rdm_pd_encode(destination, len, format, source, true);
}

size_t rdm_pd_serialize_word(void *destination, uint16_t word) {
  assert(destination != NULL);
  word = bswap16(word);
  memmove(destination, &word, sizeof(word));
  return sizeof(word);
}

size_t rdm_pd_get_description(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid,
                              rdm_pid_description_t *description) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < RDM_SUB_DEVICE_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Only manufacturer-specific parameters may have a description
  if (pid < RDM_PID_MANUFACTURER_SPECIFIC_BEGIN ||
      pid > RDM_PID_MANUFACTURER_SPECIFIC_END) {
    return 0;
  }

  size_t written = 0;
  // FIXME: implement
  // for (int i = 0; i < driver->num_parameters; ++i) {
  //   if (driver->params[i].pid == pid) {
  //     const rdm_pd_definition_t *def = &driver->params[i].definition;
  //     if (description != NULL) {
  //       description->pid = pid;
  //       description->pdl_size = def->schema.pdl_size;
  //       description->data_type = def->schema.data_type;
  //       description->cc = def->schema.cc;
  //       description->unit = def->units;
  //       description->prefix = def->prefix;
  //       description->min_value = def->schema.min_value;
  //       description->max_value = def->schema.max_value;
  //       description->default_value = def->default_value;
  //       strncpy(description->description, def->description, 32);
  //     }
  //     written = 20 + strnlen(def->description, 32);
  //     break;
  //   }
  // }

  return written;
}

rdm_response_type_t rdm_response_handler_simple(dmx_port_t dmx_num,
                                                const rdm_pd_definition_t *def,
                                                rdm_header_t *header, void *pd,
                                                uint8_t *pdl_out) {
  // Return early if the sub-device is out of range
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // FIXME: implement
  // TODO: if schema->data_type is byte/word/dword, check min/max
  // if (header->cc == RDM_CC_GET_COMMAND) {
  //   const void *data = rdm_pd_get(dmx_num, header->pid, header->sub_device);
  //   *pdl_out = rdm_pd_serialize(pd, 231, schema->format, data);
  // } else {
  //   // Deserialize the packet parameter data in place
  //   rdm_pd_deserialize(pd, header->pdl, schema->format, pd);
  //   if (!rdm_pd_set(dmx_num, header->pid, header->sub_device, pd,
  //   header->pdl)) {
  //     *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_HARDWARE_FAULT);
  //     return RDM_RESPONSE_TYPE_NACK_REASON;
  //   }
  // }

  return RDM_RESPONSE_TYPE_ACK;
}
