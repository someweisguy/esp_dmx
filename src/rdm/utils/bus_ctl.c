#include "rdm/utils/bus_ctl.h"

#include <ctype.h>

#include "dmx/bus_ctl.h"
#include "dmx/driver.h"
#include "dmx/struct.h"
#include "endian.h"
#include "rdm/utils/uid.h"

static void *rdm_format_encode(void *restrict dest, const char *restrict format,
                               const void *restrict src, size_t src_size,
                               bool encode_nulls) {
  assert(dest != NULL);
  assert(src != NULL);

  // This function assumes that the format string is valid

  while (src_size > 0) {
    const char *f = format;
    for (char c = *f; c != '\0'; c = *(++f)) {
      // Skip whitespaces
      if (c == ' ') {
        continue;
      }

      // Check for terminator
      if (c == '$') {
        return dest;
      }

      // Copy the token to the destination buffer
      size_t token_size;
      if (c >= 'A' && c <= 'Z') {
        c += 'A'; // Convert token to lowercase
      }
      if (c == 'b') {
        token_size = sizeof(uint8_t);
        memcpy(dest, src, token_size);
        // Don't need to swap endianness on single byte
      } else if (c == 'w') {
        token_size = sizeof(uint16_t);
        memcpy(dest, src, token_size);
        *(uint16_t *)dest = bswap16(*(uint16_t *)dest);
      } else if (c == 'd') {
        token_size = sizeof(uint32_t);
        memcpy(dest, src, token_size);
        *(uint32_t *)dest = bswap32(*(uint32_t *)dest);
      } else if (c == 'u' || c == 'v') {
        token_size = sizeof(rdm_uid_t);
        if (c == 'v' && (src_size < token_size || rdm_uid_is_null(src))) {
          // Handle condition where an optional UID was not provided
          if (encode_nulls) {
            memset(dest, 0, token_size);
          }
          dest += token_size;
          return dest;
        }
        memcpy(dest, src, token_size);
        rdm_uid_t *const uid = dest;
        uid->man_id = bswap16(uid->man_id);
        uid->dev_id = bswap32(uid->dev_id);
        if (c == 'v') {
          dest += token_size;
          return dest;
        }
      } else if (c == 'a') {
        token_size = strnlen(src, (src_size < 32 ? src_size : 32));
        memcpy(dest, src, token_size);
        if (encode_nulls) {
          // Only null-terminate the string if desired by the caller
          ((uint8_t *)dest)[token_size] = '\0';
          token_size += 1;
        }
        dest += token_size;
        return dest;
      } else if (c == 'x') {
        // Copy the hex literal format string to a null-terminated string
        char str[3];
        str[2] = '\0';
        memcpy(str, (f + 1), 2);

        // Get the hex literal as an integer and copy it to the destination
        token_size = sizeof(uint8_t);
        const uint8_t literal = (uint8_t)strtol(str, NULL, 16);
        memcpy(dest, &literal, token_size);
        f += 2;  // Skip to the next token
      } else {
        __unreachable();  // Unknown symbol
      }

      // Update cursor
      dest += token_size;
      src += token_size;
      src_size -= token_size;
    }
  }

  return dest;
}

bool DMX_ISR_ATTR rdm_read_header(dmx_port_t dmx_num, rdm_header_t *header) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver[dmx_num] != NULL, 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  const uint8_t *data = driver->data;
  uint16_t checksum = 0;

  // Check if packet is standard RDM packet or RDM discovery response packet
  if (*(uint16_t *)data == (RDM_SC | (RDM_SUB_SC << 8))) {
    // Verify checksum
    const uint8_t message_len = data[2];
    for (int i = 0; i < message_len; ++i) {
      checksum += data[i];
    }
    if (checksum != bswap16(*(uint16_t *)(data + message_len))) {
      return false;
    }

    // Copy the header without function calls for IRAM ISR
    if (header != NULL) {
      for (int i = 0; i < sizeof(rdm_header_t); ++i) {
        ((uint8_t *)header)[i] = data[i];
      }
      header->dest_uid.man_id = bswap16(header->dest_uid.man_id);
      header->dest_uid.dev_id = bswap32(header->dest_uid.dev_id);
      header->src_uid.man_id = bswap16(header->src_uid.man_id);
      header->src_uid.dev_id = bswap32(header->src_uid.dev_id);
      header->sub_device = bswap16(header->sub_device);
      header->pid = bswap16(header->pid);
    }

    return true;
  } else if (*data == RDM_PREAMBLE || *data == RDM_DELIMITER) {
    // Get and verify the preamble length (must be <= 7 bytes)
    int preamble_len = 0;
    for (; preamble_len <= 7; ++preamble_len) {
      if (data[preamble_len] == RDM_DELIMITER) break;
    }
    if (preamble_len > 7) {
      return false;
    }
    data += preamble_len + 1;

    // Verify checksum
    for (int i = 0; i < 12; ++i) {
      checksum += data[i];
    }
    if (checksum != (((data[12] & data[13]) << 8) |
                     (data[14] & data[15]))) {
      return false;
    }

    // Copy the header without function calls for IRAM ISR
    if (header != NULL) {
      // Decode the EUID
      uint8_t euid_buf[6];
      for (int i = 0, j = 0; i < sizeof(euid_buf); ++i, j += 2) {
        euid_buf[i] = data[j] & data[j + 1];
      }

      for (int i = 0; i < sizeof(rdm_uid_t); ++i) {
        ((uint8_t *)&header->src_uid)[i] = euid_buf[i];
      }
      header->src_uid.man_id = bswap16(header->src_uid.man_id);
      header->src_uid.dev_id = bswap32(header->src_uid.dev_id);
      header->dest_uid.man_id = bswap16(RDM_UID_BROADCAST_ALL.man_id);
      header->dest_uid.dev_id = bswap32(RDM_UID_BROADCAST_ALL.dev_id);
      header->tn = 0;
      header->response_type = RDM_RESPONSE_TYPE_ACK;
      header->message_count = 0;
      header->sub_device = RDM_SUB_DEVICE_ROOT;
      header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
      header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
      header->pdl = 0;
    }

    return true;
  }

  return false;
}

size_t rdm_read_pd(dmx_port_t dmx_num, const char *format, void *destination,
                   size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(format == NULL || rdm_format_get_max_size(format) > 0, 0,
            "format is invalid");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Guard against invalid PDL
  const size_t pdl = driver->data[23];
  if (pdl == 0 || pdl > 231) {
    return 0;
  }

  // Deserialize the parameter data into the destination buffer
  if (destination != NULL) {
    size = pdl < size ? pdl : size;
    const bool encode_nulls = true;
    const uint8_t *pd = &driver->data[24];
    rdm_format_encode(destination, format, pd, size, encode_nulls);
  }

  return pdl;
}

size_t rdm_write(dmx_port_t dmx_num, const rdm_header_t *header,
                 const char *format, const void *pd) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(
      header->message_len >= 24 && header->message_len == 24 + header->pdl, 0,
      "header->message_len error");
  DMX_CHECK(header->sub_device < RDM_SUB_DEVICE_MAX ||
                header->sub_device == RDM_SUB_DEVICE_ALL,
            0, "header->sub_device error");
  DMX_CHECK(rdm_cc_is_valid(header->cc), 0, "header->cc error");
  DMX_CHECK(header->pdl < 231, 0, "header->pdl error");
  if (rdm_cc_is_request(header->cc)) {
    DMX_CHECK(header->port_id > 0, 0, "header->port_id error");
  }  else {
    DMX_CHECK(rdm_response_type_is_valid(header->response_type), 0,
              "header->response_type error");
  }
  DMX_CHECK(rdm_pd_format_is_valid(format), 0, "format is invalid");
  DMX_CHECK(header->pdl == 0 || (format != NULL && pd != NULL), 0,
            "pd or format is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Encode a standard RDM packet or a RDM_CC_DISC_COMMAND_RESPONSE packet
  size_t written;
  const bool encode_nulls = false;
  if (header->cc != RDM_CC_DISC_COMMAND_RESPONSE) {
    // Serialize the header and pd into the driver buffer
    const char *header_format = "xCCx01buubbbwbwb";
    void *data = rdm_format_encode(driver->data, header_format, header,
                                  sizeof(*header), encode_nulls);
    if (header->pdl > 0) {
      data = rdm_format_encode(data, format, pd, header->pdl, encode_nulls);
    }

    // Calculate and serialize the checksum
    uint16_t checksum = RDM_SC + RDM_SUB_SC;
    for (int i = 2; i < header->message_len; ++i) {
      checksum += driver->data[i];
    }
    checksum = bswap16(checksum);
    memcpy(data, &checksum, sizeof(checksum));
    
    written = header->message_len + 2;
  } else {
    // Encode the preamble bytes
    const size_t preamble_len = 7;
    memset(driver->data, RDM_PREAMBLE, preamble_len);
    driver->data[preamble_len] = RDM_DELIMITER;
    uint8_t *data = &driver->data[preamble_len + 1];
    
    // Encode the UID and calculate the checksum
    uint8_t uid[6];
    rdm_uidcpy(uid, &header->src_uid);
    uint16_t checksum = 0;
    for (int i = 0, j = 0; j < sizeof(rdm_uid_t); i += 2, ++j) {
      data[i] = uid[j] | 0xaa;
      data[i + 1] = uid[j] | 0x55;
      checksum += uid[j] + (0xaa | 0x55);
    }

    // Encode the checksum
    const int cs_offset = sizeof(rdm_uid_t) * 2;
    data[cs_offset + 0] = (uint8_t)(checksum >> 8) | 0xaa;
    data[cs_offset + 1] = (uint8_t)(checksum >> 8) | 0x55;
    data[cs_offset + 2] = (uint8_t)(checksum) | 0xaa;
    data[cs_offset + 3] = (uint8_t)(checksum) | 0x55;

    // Update written size
    written = preamble_len + 1 + 16;
  }

  return written;
}

size_t rdm_write_ack(dmx_port_t dmx_num, const rdm_header_t *header,
                     const char *format, const void *pd, size_t pdl) {
  // TODO: assert header->cc is a request
  /*
    // Rewrite the header for the response packet
header.message_len = 24 + pdl_out;  // Set for user callback
header.dest_uid = header.src_uid;
header.src_uid = my_uid;
header.response_type = response_type;
header.message_count = rdm_pd_queue_get_size(dmx_num)
header.cc |= 1;  // Set to RDM_CC_x_COMMAND_RESPONSE
header.pdl = pdl_out;
// These fields should not change: tn, sub_device, and pid
  */
  return 0;
}

/*
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
    // TODO: setting the wait_ticks <= 3 causes instability on Arduino
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
*/

void rdm_set_boot_loader(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
}

/* TODO
bool rdm_status_push(dmx_port_t dmx_num, const rdm_status_message_t *message) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(message != NULL);
  assert((message->type >= RDM_STATUS_ADVISORY &&
          message->type <= RDM_STATUS_ERROR) ||
         (message->type >= RDM_STATUS_ADVISORY_CLEARED &&
          message->type <= RDM_STATUS_ERROR_CLEARED));
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  bool ret = false;

  const uint32_t qi = (message->type & 0xf) - 2;  // Queue index
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const uint16_t new_tail =
      (driver->rdm_status[qi].tail + 1) % RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX;
  if (new_tail != driver->rdm_status[qi].head) {
    driver->rdm_status[qi].queue[driver->rdm_status[qi].tail] = *message;
    driver->rdm_status[qi].tail = new_tail;
    ret = true;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return ret;
}

bool rdm_status_pop(dmx_port_t dmx_num, rdm_status_t status,
                    rdm_status_message_t *message) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(message != NULL);
  assert((status >= RDM_STATUS_ADVISORY && status <= RDM_STATUS_ERROR) ||
         (status >= RDM_STATUS_ADVISORY_CLEARED &&
          status <= RDM_STATUS_ERROR_CLEARED));
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  bool ret = false;

  uint32_t qi = (status & 0xf) - 2;  // Queue index
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (;qi < 3; ++qi) {
    if (driver->rdm_status[qi].head == driver->rdm_status[qi].tail) {
      continue;  // Nothing to pop in this queue
    }
    const uint16_t new_head =
        (driver->rdm_status[qi].head + 1) % RDM_RESPONDER_STATUS_QUEUE_SIZE_MAX;
    *message = driver->rdm_status[qi].queue[driver->rdm_status[qi].head];
    driver->rdm_status[qi].head = new_head;
    ret = true;
    break;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return ret;
}

void rdm_status_clear(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  for (int i = 0; i < 3; ++i) {
    driver->rdm_queue[i].tail = driver->rdm_queue[i].head;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
}


rdm_status_t rdm_status_get_threshold(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  rdm_status_t threshold;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  threshold = driver->rdm_status_threshold
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return threshold;
}

void rdm_status_set_threshold(dmx_port_t dmx_num, rdm_status_t status);
*/