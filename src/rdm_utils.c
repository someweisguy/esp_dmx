#include "rdm_utils.h"

#include <ctype.h>
#include <string.h>

#include "dmx/config.h"
#include "dmx/hal.h"
#include "dmx/nvs.h"
#include "dmx/struct.h"
#include "endian.h"
#include "esp_dmx.h"
#include "rdm_types.h"

void *rdm_uidcpy(void *restrict destination, const void *restrict source) {
  assert(destination != NULL);
  assert(source != NULL);
  *(uint16_t *)destination = bswap16(*(uint16_t *)source);
  *(uint32_t *)(destination + 2) = bswap32(*(uint32_t *)(source + 2));
  return destination;
}

void *rdm_uidmove(void *destination, const void *source) {
  assert(destination != NULL);
  assert(source != NULL);
  const rdm_uid_t temp = {.man_id = ((rdm_uid_t *)source)->man_id,
                          .dev_id = ((rdm_uid_t *)source)->dev_id};
  return rdm_uidcpy(destination, &temp);
}

void DMX_ISR_ATTR rdm_uid_get(dmx_port_t dmx_num, rdm_uid_t *uid) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(uid != NULL);

  // Copy the binding UID and increment the final octet by dmx_num
  uid->man_id = rdm_device_uid.man_id;
  uid->dev_id = rdm_device_uid.dev_id;
  if (!rdm_uid_is_null(uid)) {
    uint8_t last_octet = (uint8_t)uid->dev_id;
    last_octet += dmx_num;
    uid->dev_id &= 0xffffff00;
    uid->dev_id |= last_octet;
  }
}

void rdm_uid_get_binding(rdm_uid_t *uid) {
  rdm_uid_get(rdm_binding_port, uid);
}

static size_t rdm_pd_parse(const char *format) {
  int param_size = 0;
  for (const char *f = format; *f != '\0'; ++f) {
    size_t field_size = 0;
    if (*f == 'b' || *f == 'B') {
      field_size = sizeof(uint8_t);  // Handle 8-bit byte
    } else if (*f == 'w' || *f == 'W') {
      field_size = sizeof(uint16_t);  // Handle 16-bit word
    } else if (*f == 'd' || *f == 'D') {
      field_size = sizeof(uint32_t);  // Handle 32-bit dword
    } else if (*f == 'u' || *f == 'U') {
      field_size = sizeof(rdm_uid_t);  // Handle 48-bit UID
    } else if (*f == 'v' || *f == 'V') {
      DMX_CHECK(f[1] == '\0' || f[1] == '$', 0,
                "Optional UID not at end of parameter.");
      field_size = sizeof(rdm_uid_t);
    } else if (*f == 'a' || *f == 'A') {
      // Handle ASCII string
      DMX_CHECK(f[1] == '\0' || f[1] == '$', 0,
                "String not at end of parameter.");
      field_size = 32;
    } else if (*f == '#') {
      // Handle integer literal
      ++f;  // Ignore '#' character
      int num_chars = 0;
      for (; num_chars <= 16; ++num_chars) {
        if (!isxdigit((int)f[num_chars])) break;
      }
      DMX_CHECK(num_chars <= 16, 0, "Integer literal is too big");
      field_size = (num_chars / 2) + (num_chars % 2);
      f += num_chars;
      DMX_CHECK(*f == 'h' || *f == 'H', 0,
                "Improperly terminated integer literal.");
    } else if (*f == '$') {
      DMX_CHECK(f[1] == '\0', 0, "Improperly placed end-of-parameter anchor.");
    } else {
      DMX_CHECK(1, 0, "Unknown symbol '%c' found at index %i.", *f, f - format);
    }

    // Ensure format size doesn't exceed MDB size.
    DMX_CHECK(param_size + field_size <= 231, 0, "Parameter is too big.");
    param_size += field_size;
  }
  return param_size;
}

size_t rdm_pd_emplace(void *destination, const char *format, const void *source,
                      size_t num, bool emplace_nulls) {
  assert(destination != NULL);
  assert(format != NULL);
  assert(source != NULL);

  // Clamp the size to the maximum parameter data length
  if (num > 231) {
    num = 231;
  }

  // Ensure that the format string syntax is correct
  const int param_size = rdm_pd_parse(format);
  if (param_size == 0) {
    return 0;
  }
  
  // Get the number of parameters that can be encoded
  int num_params_to_copy;
  if (format[strlen(format)] == '$' || format[strlen(format)] == 'a' ||
       format[strlen(format)] == 'A' || format[strlen(format)] == 'v' ||
       format[strlen(format)] == 'V') {
    num_params_to_copy = 1;  // Format string is a singleton
  } else {
    num_params_to_copy = num / param_size;
  }

  // Encode the fields into the destination
  size_t n = 0;
  for (int i = 0; i < num_params_to_copy; ++i) {
    for (const char *f = format; *f != '\0'; ++f) {
      if (*f == 'b' || *f == 'B') {
        if (destination != NULL) {
          *(uint8_t *)(destination + n) = *(uint8_t *)(source + n);
        }
        n += sizeof(uint8_t);
      } else if (*f == 'w' || *f == 'W') {
        if (destination != NULL) {
          *(uint16_t *)(destination + n) = bswap16(*(uint16_t *)(source + n));
        }
        n += sizeof(uint16_t);
      } else if (*f == 'd' || *f == 'D') {
        if (destination != NULL) {
          *(uint32_t *)(destination + n) = bswap32(*(uint32_t *)(source + n));
        }
        n += sizeof(uint32_t);
      } else if (*f == 'u' || *f == 'U' || *f == 'v' || *f == 'V') {
        if ((*f == 'v' || *f == 'V') && !emplace_nulls && source != NULL &&
            rdm_uid_is_null(source + n)) {
          break;  // Optional UIDs will be at end of parameter string
        }
        if (destination != NULL && source != NULL) {
          rdm_uidmove(destination + n, source + n);
        }
        n += sizeof(rdm_uid_t);
      } else if (*f == 'a' || *f == 'A') {
        size_t len;
        if (source != NULL) {
          const size_t max_len = (num - n) < 32 ? (num - n) : 32;
          len = strnlen(source + n, max_len);
        } else {
          len = 32;
        }
        if (destination != NULL && source != NULL) {
          memmove(destination + n, source + n, len);
          if (emplace_nulls) {
            *(uint8_t *)(destination + n + len) = 0;
          }
        }
        n += len + emplace_nulls;
        break;
      } else if (*f == '#') {
        ++f;  // Skip '#' character
        char *end_ptr;
        const uint64_t literal = strtol(f, &end_ptr, 16);
        const int literal_len = ((end_ptr - f) / 2) + ((end_ptr - f) % 2);
        if (destination != NULL) {
          for (int j = 0, k = literal_len - 1; j < literal_len; ++j, --k) {
            ((uint8_t *)destination + n)[j] = ((uint8_t *)&literal)[k];
          }
        }
        f = end_ptr;
        n += literal_len;
      }
    }
  }
  return n;
}

size_t rdm_pd_emplace_word(void *destination, uint16_t word) {
  assert(destination != NULL);

  *(uint16_t *)destination = bswap16(word);
  return sizeof(word);
}

void *rdm_pd_alloc(dmx_port_t dmx_num, size_t size) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  if (size == 0) {
    return NULL;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  void *ret = NULL;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->pd_head + size <= driver->pd_size) {
    ret = driver->pd + driver->pd_head;
    driver->pd_head += size;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return ret;
}

bool rdm_pd_register(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            const rdm_pid_description_t *desc,
                            const char *param_str, rdm_driver_cb_t driver_cb,
                            void *param, rdm_responder_cb_t user_cb,
                            void *context, bool nvs) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(sub_device < 513);
  assert(desc != NULL);
  assert(driver_cb != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  if (sub_device != RDM_SUB_DEVICE_ROOT) {
    ESP_LOGE(TAG, "Multiple sub-devices are not yet supported.");
    return false;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Iterate the callback list to see if a callback with this PID exists
  int i = 0;
  for (; i < driver->num_rdm_cbs; ++i) {
    if (driver->rdm_cbs[i].desc.pid == desc->pid) break;
  }

  // Check if there is space for callbacks
  if (i == RDM_RESPONDER_PIDS_MAX) {
    ESP_LOGE(TAG, "No more space for RDM callbacks");
    return false;
  }
  
  // Add the requested callback to the callback list
  driver->rdm_cbs[i].param_str = param_str;
  driver->rdm_cbs[i].param = param;
  driver->rdm_cbs[i].context = context;
  driver->rdm_cbs[i].user_cb = user_cb;
  driver->rdm_cbs[i].driver_cb = driver_cb;
  driver->rdm_cbs[i].desc = *desc;
  driver->rdm_cbs[i].non_volatile = nvs;
  const bool added_cb = (i == driver->num_rdm_cbs);
  if (added_cb) {
    ++driver->num_rdm_cbs;
  }

  return true;
}

void *rdm_pd_get(dmx_port_t dmx_num, rdm_pid_t pid,
                        rdm_sub_device_t sub_device) {
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  void *pd = NULL;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  // Find parameter data and its descriptor
  for (int i = 0; i < driver->num_rdm_cbs; ++i) {
    if (driver->rdm_cbs[i].desc.pid == pid) {
      pd = driver->rdm_cbs[i].param;
      break;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pd;
}

bool rdm_pd_set(dmx_port_t dmx_num, rdm_pid_t pid,
                       rdm_sub_device_t sub_device, const void *param,
                       size_t size, bool add_to_queue) {
  assert(param != NULL);
  assert(size > 0);

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  bool ret;
  void *pd = NULL;
  bool save_to_nvs = false;
  const rdm_pid_description_t *desc = NULL;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  // Find parameter data and its descriptor
  for (int i = 0; i < driver->num_rdm_cbs; ++i) {
    if (driver->rdm_cbs[i].desc.pid == pid) {
      pd = driver->rdm_cbs[i].param;
      desc = &driver->rdm_cbs[i].desc;
      save_to_nvs = driver->rdm_cbs[i].non_volatile;
      break;
    }
  }

  // Copy the user's variable to the parameter data
  if (pd != NULL && (desc->cc & RDM_CC_SET)) {
    size = size < desc->pdl_size ? size : desc->pdl_size;
    if (desc->data_type == RDM_DS_ASCII) {
      strncpy(pd, param, size);
    } else {
      memcpy(pd, param, size);
    }
    ret = true;
  } else {
    ret = false;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Handle NVS and RDM queueing
  if (ret) {
    // Save to NVS if needed
    if (save_to_nvs) {
      if (!dmx_nvs_set(dmx_num, pid, desc->data_type, param, size)) {
        taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
        driver->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
        taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
        DMX_WARN("unable to save PID 0x%04x to NVS", pid)
      }
    }

    // Enqueue the RDM packet if desired
    if (add_to_queue) {
      bool success; 
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      if (driver->rdm_queue_size < RDM_RESPONDER_MAX_QUEUE_SIZE) {
        driver->rdm_queue[driver->rdm_queue_size] = pid;
        ++driver->rdm_queue_size;
        success = true;
      } else {
        success = false;
      }
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      if (!success) {
        DMX_WARN("out of queue space");
      }
    }
  }

  return ret;
}

bool rdm_send_request(dmx_port_t dmx_num, rdm_header_t *header,
                      const void *pd_in, void *pd_out, size_t *pdl,
                      rdm_ack_t *ack) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(header != NULL);
  assert(!rdm_uid_is_null(&header->dest_uid));
  assert(!rdm_uid_is_broadcast(&header->src_uid));
  assert(header->cc == RDM_CC_DISC_COMMAND ||
         header->cc == RDM_CC_GET_COMMAND || header->cc == RDM_CC_SET_COMMAND);
  assert(header->sub_device < 513 ||
         (header->sub_device == RDM_SUB_DEVICE_ALL &&
          header->cc != RDM_CC_GET_COMMAND));
  assert(header->pdl <= 231);
  assert(pdl != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Update the optional components of the header to allowed values
  if (header->port_id == 0) {
    header->port_id = dmx_num + 1;
  }
  if (rdm_uid_is_null(&header->src_uid)) {
    rdm_uid_get(dmx_num, &header->src_uid);
  }

  // Set header values that the user cannot set themselves
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  header->tn = driver->tn;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  header->message_count = 0;  // Required for all controller requests

  // Determine if a response is expected
  const bool response_expected = !rdm_uid_is_broadcast(&header->dest_uid) ||
                                 (header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
                                  header->cc == RDM_CC_DISC_COMMAND);

  // Block until the mutex can be taken
  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return 0;
  }

  // Block until the driver is done sending
  if (!dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23))) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Write and send the request
  size_t size = dmx_write_rdm(dmx_num, header, pd_in);
  dmx_send(dmx_num, size);

  // Return early if a packet error occurred or if no response was expected
  if (response_expected) {
    dmx_packet_t packet;
    // FIXME: setting the wait_ticks <= 3 causes instability on Arduino
    size = dmx_receive(dmx_num, &packet, 10);
    if (ack != NULL) {
      ack->err = packet.err;
      ack->size = size;
    }
    if (packet.err && packet.err != DMX_ERR_TIMEOUT) {
      if (ack != NULL) {
        ack->src_uid = (rdm_uid_t){0, 0};
        ack->message_count = 0;
        ack->type = RDM_RESPONSE_TYPE_INVALID;
        ack->pid = 0;
      }
      xSemaphoreGiveRecursive(driver->mux);
      return false;
    } else if (size == 0) {
      // TODO: remove this else if when refactoring dmx_receive()
      if (ack != NULL) {
        ack->src_uid = (rdm_uid_t){0, 0};
        ack->message_count = 0;
        ack->type = RDM_RESPONSE_TYPE_NONE;
        ack->pid = 0;
      }
      xSemaphoreGiveRecursive(driver->mux);
      return false;
    }
  } else {
    if (ack != NULL) {
      ack->err = DMX_OK;
      ack->size = size;
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->message_count = 0;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->pid = 0;
    }
    dmx_wait_sent(dmx_num, 2);
    xSemaphoreGiveRecursive(driver->mux);
    return false;
  }

  // Handle the RDM response packet
  rdm_header_t resp;
  rdm_response_type_t response_type;
  if (!dmx_read_rdm(dmx_num, &resp, pd_out, *pdl)) {
    response_type = RDM_RESPONSE_TYPE_INVALID;  // Data or checksum error
    resp.pdl = 0;
  } else if (resp.response_type != RDM_RESPONSE_TYPE_ACK &&
             resp.response_type != RDM_RESPONSE_TYPE_ACK_TIMER &&
             resp.response_type != RDM_RESPONSE_TYPE_NACK_REASON &&
             resp.response_type != RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
    response_type = RDM_RESPONSE_TYPE_INVALID;  // Invalid response_type
  } else if (header->pid != RDM_PID_DISC_UNIQUE_BRANCH &&
             (header->cc != (resp.cc - 1) || header->pid != resp.pid ||
              header->tn != resp.tn ||
              !rdm_uid_is_target(&resp.src_uid, &header->dest_uid) ||
              !rdm_uid_is_eq(&resp.dest_uid, &header->src_uid))) {
    response_type = RDM_RESPONSE_TYPE_INVALID;  // Invalid response format
  } else {
    response_type = resp.response_type;  // Response is ok
  }
  if (pdl != NULL) {
    *pdl = resp.pdl;
  }

  uint32_t decoded;
  // Handle the response based on the response type
  if (response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
    // Get and convert the estimated response time to FreeRTOS ticks
    decoded = pdDMX_MS_TO_TICKS(bswap16(*(uint16_t *)pd_out) * 10);
  } else if (response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
    // Get and report the received NACK reason
    decoded = bswap16(*(uint16_t *)pd_out);
  } else if (response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
    DMX_WARN("RDM_RESPONSE_TYPE_ACK_OVERFLOW is not yet supported.");  // TODO
    decoded = 0;
  } else {
    // Do nothing when response_type is RDM_RESPONSE_TYPE_ACK
    decoded = 0;
  }

  // Report the results back to the caller
  if (ack != NULL) {
    ack->type = response_type;
    ack->src_uid = resp.src_uid;
    ack->message_count = resp.message_count;
    ack->timer = decoded;
    ack->pid = resp.pid;
  }

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return (response_type == RDM_RESPONSE_TYPE_ACK);
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
  for (; num_pids < driver->num_rdm_cbs; ++num_pids) {
    if (num_pids < num) {
      pids[num_pids] = driver->rdm_cbs->desc.pid;
    }
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return num_pids;
}