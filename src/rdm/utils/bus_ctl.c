#include "rdm/utils/bus_ctl.h"

#include <ctype.h>

#include "dmx/bus_ctl.h"
#include "dmx/driver.h"
#include "dmx/struct.h"
#include "endian.h"
#include "rdm/utils/uid.h"

size_t DMX_ISR_ATTR rdm_read(dmx_port_t dmx_num, rdm_header_t *header, void *pd,
                             size_t num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t read = 0;

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get pointers to driver data buffer locations and declare checksum
  uint16_t checksum = 0;
  const uint8_t *header_ptr = driver->data;
  const void *pd_ptr = header_ptr + 24;

  // Verify start code and sub-start code are correct
  if (*(uint16_t *)header_ptr != (RDM_SC | (RDM_SUB_SC << 8)) &&
      *header_ptr != RDM_PREAMBLE && *header_ptr != RDM_DELIMITER) {
    return read;
  }

  // Get and verify the preamble_len if packet is a discovery response
  size_t preamble_len = 0;
  if (*header_ptr == RDM_PREAMBLE || *header_ptr == RDM_DELIMITER) {
    for (; preamble_len <= 7; ++preamble_len) {
      if (header_ptr[preamble_len] == RDM_DELIMITER) break;
    }
    if (preamble_len > 7) {
      return read;
    }
  }

  // Handle packets differently if a DISC_UNIQUE_BRANCH packet was received
  if (*header_ptr == RDM_SC) {
    // Verify checksum is correct
    const uint8_t message_len = header_ptr[2];
    for (int i = 0; i < message_len; ++i) {
      checksum += header_ptr[i];
    }
    if (checksum != bswap16(*(uint16_t *)(header_ptr + message_len))) {
      return read;
    }

    // Copy the header and pd from the driver
    if (header != NULL) {
      // Copy header without emplace so this function can be used in IRAM ISR
      for (int i = 0; i < sizeof(rdm_header_t); ++i) {
        ((uint8_t *)header)[i] = header_ptr[i];
      }
      header->dest_uid.man_id = bswap16(header->dest_uid.man_id);
      header->dest_uid.dev_id = bswap32(header->dest_uid.dev_id);
      header->src_uid.man_id = bswap16(header->src_uid.man_id);
      header->src_uid.dev_id = bswap32(header->src_uid.dev_id);
      header->sub_device = bswap16(header->sub_device);
      header->pid = bswap16(header->pid);
    }
    if (pd != NULL) {
      const uint8_t pdl = header_ptr[23];
      const size_t copy_size = pdl < num ? pdl : num;
      memcpy(pd, pd_ptr, copy_size);
    }

    // Update the read size
    read = message_len + 2;

  } else {
    // Verify the checksum is correct
    header_ptr += preamble_len + 1;
    for (int i = 0; i < 12; ++i) {
      checksum += header_ptr[i];
    }
    if (checksum != (((header_ptr[12] & header_ptr[13]) << 8) |
                     (header_ptr[14] & header_ptr[15]))) {
      return read;
    }

    // Decode the EUID
    uint8_t buf[6];
    for (int i = 0, j = 0; i < 6; ++i, j += 2) {
      buf[i] = header_ptr[j] & header_ptr[j + 1];
    }

    // Copy the data into the header
    if (header != NULL) {
      // Copy header without emplace so this function can be used in IRAM ISR
      for (int i = 0; i < sizeof(rdm_uid_t); ++i) {
        ((uint8_t *)&header->src_uid)[i] = buf[i];
      }
      header->src_uid.man_id = bswap16(header->src_uid.man_id);
      header->src_uid.dev_id = bswap32(header->src_uid.dev_id);
      header->dest_uid = (rdm_uid_t){0, 0};
      header->tn = 0;
      header->response_type = RDM_RESPONSE_TYPE_ACK;
      header->message_count = 0;
      header->sub_device = RDM_SUB_DEVICE_ROOT;
      header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
      header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
      header->pdl = preamble_len + 1 + 16;
    }

    // Update the read size
    read = preamble_len + 1 + 16;
  }

  return read;
}

size_t rdm_write(dmx_port_t dmx_num, rdm_header_t *header, const void *pd) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL || pd != NULL, 0,
            "header is null and pd does not contain a UID");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t written = 0;

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get pointers to driver data buffer locations and declare checksum
  uint16_t checksum = 0;
  uint8_t *header_ptr = driver->data;
  void *pd_ptr = header_ptr + 24;

  // RDM writes must be synchronous to prevent data corruption
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) {
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    return written;
  } else if (dmx_uart_get_rts(driver->uart) == 1) {
    dmx_uart_set_rts(driver->uart, 0);  // Stops writes from being overwritten
  }

  if (header != NULL && !(header->cc == RDM_CC_DISC_COMMAND_RESPONSE &&
                          header->pid == RDM_PID_DISC_UNIQUE_BRANCH)) {
    // Copy the header, pd, message_len, and pdl into the driver
    const size_t copy_size = header->pdl <= 231 ? header->pdl : 231;
    header->message_len = copy_size + 24;
    rdm_emplace(header_ptr, "#cc01hbuubbbwbwb", header, sizeof(*header), false);
    memcpy(pd_ptr, pd, copy_size);

    // Calculate and copy the checksum
    checksum = RDM_SC + RDM_SUB_SC;
    for (int i = 2; i < header->message_len; ++i) {
      checksum += header_ptr[i];
    }
    *(uint16_t *)(header_ptr + header->message_len) = bswap16(checksum);

    // Update written size
    written = header->message_len + 2;
  } else {
    // Encode the preamble bytes
    const size_t preamble_len = 7;
    for (int i = 0; i < preamble_len; ++i) {
      header_ptr[i] = RDM_PREAMBLE;
    }
    header_ptr[preamble_len] = RDM_DELIMITER;
    header_ptr += preamble_len + 1;

    // Encode the UID and calculate the checksum
    uint8_t uid[6];
    if (header == NULL) {
      memcpy(uid, pd, sizeof(rdm_uid_t));
    } else {
      rdm_uidcpy(uid, &header->src_uid);
    }
    for (int i = 0, j = 0; j < sizeof(rdm_uid_t); i += 2, ++j) {
      header_ptr[i] = uid[j] | 0xaa;
      header_ptr[i + 1] = uid[j] | 0x55;
      checksum += uid[j] + (0xaa | 0x55);
    }
    header_ptr += sizeof(rdm_uid_t) * 2;

    // Encode the checksum
    header_ptr[0] = (uint8_t)(checksum >> 8) | 0xaa;
    header_ptr[1] = (uint8_t)(checksum >> 8) | 0x55;
    header_ptr[2] = (uint8_t)(checksum) | 0xaa;
    header_ptr[3] = (uint8_t)(checksum) | 0x55;

    // Update written size
    written = preamble_len + 1 + 16;
  }

  // Update driver transmission size
  driver->tx_size = written;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return written;
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

size_t rdm_emplace(void *destination, const char *format, const void *source,
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

size_t rdm_emplace_word(void *destination, uint16_t word) {
  assert(destination != NULL);

  *(uint16_t *)destination = bswap16(word);
  return sizeof(word);
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
  size_t size = rdm_write(dmx_num, header, pd_in);
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
  if (!rdm_read(dmx_num, &resp, pd_out, *pdl)) {
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

rdm_pid_t rdm_queue_pop(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  rdm_pid_t pid;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  pid = rdm_queue_peek(dmx_num);
  --dmx_driver[dmx_num]->rdm_queue_size;
  dmx_driver[dmx_num]->rdm_queue_last_sent = pid;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}

rdm_pid_t rdm_queue_peek(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  rdm_pid_t pid;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  pid = driver->rdm_queue[driver->rdm_queue_size - 1];
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}

rdm_pid_t rdm_queue_get_last_sent(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  rdm_pid_t pid;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  pid = driver->rdm_queue_last_sent;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}

void rdm_set_boot_loader(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
}

bool rdm_status_push(dmx_port_t dmx_num, rdm_status_t status,
                     const rdm_status_message_t *message) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(message != NULL);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  bool ret = false;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->rdm_status_queue_size < RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX) {
    memcpy(&driver->rdm_status_queue[driver->rdm_status_queue_size], message,
           sizeof(rdm_status_message_t));
    ++driver->rdm_status_queue_size;
    ret = true;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return ret;
}

int rdm_status_peek(dmx_port_t dmx_num, rdm_status_t status,
                    rdm_status_message_t *message) {
  // TODO:
  return 0;
}

int rdm_status_pop(dmx_port_t dmx_num, rdm_status_t status,
                   rdm_status_message_t *message) {

  // FIXME: how to pop only messages of a specific status?

  // TODO:
  return 0;
}