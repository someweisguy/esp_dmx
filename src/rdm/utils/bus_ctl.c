#include "rdm/utils/include/bus_ctl.h"

#include <ctype.h>
#include <string.h>

#include "dmx/include/io.h"
#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "endian.h"
#include "rdm/utils/include/pd.h"
#include "rdm/utils/include/uid.h"

static size_t rdm_format_encode(void *restrict dest,
                                const char *restrict format,
                                const void *restrict src, size_t src_size,
                                bool encode_nulls) {
  assert(dest != NULL);
  // assert(rdm_pd_format_is_valid(format));  // TODO
  assert(src != NULL);

  size_t encoded = 0;
  while (src_size > 0) {
    const char *f = format;
    for (char c = *f; c != '\0'; c = *(++f)) {
      // Skip whitespaces
      if (c == ' ') {
        continue;
      }

      // Check for terminator
      if (c == '$') {
        return encoded;
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
        encoded += token_size;
      } else if (c == 'w') {
        token_size = sizeof(uint16_t);
        memcpy(dest, src, token_size);
        *(uint16_t *)dest = bswap16(*(uint16_t *)dest);
        encoded += token_size;
      } else if (c == 'd') {
        token_size = sizeof(uint32_t);
        memcpy(dest, src, token_size);
        *(uint32_t *)dest = bswap32(*(uint32_t *)dest);
        encoded += token_size;
      } else if (c == 'u' || c == 'v') {
        token_size = sizeof(rdm_uid_t);
        if (c == 'v' && (src_size < token_size || rdm_uid_is_null(src))) {
          // Handle condition where an optional UID was not provided
          if (encode_nulls) {
            memset(dest, 0, token_size);
            encoded += token_size;
          }
          return encoded;
        }
        memcpy(dest, src, token_size);
        rdm_uid_t *const uid = dest;
        uid->man_id = bswap16(uid->man_id);
        uid->dev_id = bswap32(uid->dev_id);
        encoded += token_size;
        if (c == 'v') {
          return encoded;
        }
      } else if (c == 'a') {
        token_size = strnlen(src, (src_size < 32 ? src_size : 32));
        memcpy(dest, src, token_size);
        if (encode_nulls) {
          // Only null-terminate the string if desired by the caller
          ((uint8_t *)dest)[token_size] = '\0';
          token_size += 1;
        }
        encoded += token_size;
        return encoded;
      } else if (c == 'x') {
        // Copy the hex literal format string to a null-terminated string
        char str[3];
        str[2] = '\0';
        memcpy(str, (f + 1), 2);

        // Get the hex literal as an integer and copy it to the destination
        token_size = sizeof(uint8_t);
        const uint8_t literal = (uint8_t)strtol(str, NULL, 16);
        memcpy(dest, &literal, token_size);
        encoded += token_size;
        // Don't need to swap endianness on single byte      
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

  return encoded;
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
  DMX_CHECK(rdm_pd_format_is_valid(format), 0, "format is invalid");
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
  // DMX_CHECK(rdm_pd_format_is_valid(format), 0, "format is invalid"); // TODO
  DMX_CHECK(header->pdl == 0 || (format != NULL && pd != NULL), 0,
            "pd or format is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Encode a standard RDM packet or a RDM_CC_DISC_COMMAND_RESPONSE packet
  size_t written;
  const bool encode_nulls = false;
  if (header->cc == RDM_CC_DISC_COMMAND_RESPONSE &&
      header->pid == RDM_PID_DISC_UNIQUE_BRANCH) {
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
  } else {
    // Serialize the header and pd into the driver buffer
    const char *header_format = "xCCx01buubbbwbwb";
    rdm_format_encode(driver->data, header_format, header, sizeof(*header),
                      encode_nulls);
    size_t message_len;
    void *data = &driver->data[24];
    if (pd != NULL && header->pdl > 0) {
      size_t pdl =
          rdm_format_encode(data, format, pd, header->pdl, encode_nulls);
      if (header->pdl != pdl) {
        message_len = 24 + pdl;
        driver->data[2] = message_len;  // Encode updated header->message_len
        driver->data[23] = pdl;         // Encode updated header->pdl
      } else {
        message_len = header->message_len;
      }
      data += pdl;
    } else {
      message_len = sizeof(rdm_header_t);
    }

    // Calculate and serialize the checksum
    uint16_t checksum = RDM_SC + RDM_SUB_SC;
    for (int i = 2; i < message_len; ++i) {
      checksum += driver->data[i];
    }
    checksum = bswap16(checksum);
    memcpy(data, &checksum, sizeof(checksum));

    written = message_len + 2;
  }

  return written;
}

size_t rdm_write_ack(dmx_port_t dmx_num, const rdm_header_t *header,
                     const char *format, const void *pd, size_t pdl) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(header != NULL);
  assert(rdm_cc_is_request(header->cc));
  // assert(format == NULL || rdm_pd_format_is_valid(format)); // TODO
  assert(format != NULL || pd == NULL);
  assert(pd != NULL || pdl == 0);
  assert(pdl < 231);
  assert(dmx_driver_is_installed(dmx_num));
  
  // Build the response header
  rdm_header_t response_header = {
    .message_len = 24 + pdl,
    .dest_uid = header->src_uid,
    .tn = header->tn,
    .response_type = RDM_RESPONSE_TYPE_ACK,
    .message_count = rdm_pd_queue_get_size(dmx_num),
    .sub_device = header->sub_device,
    .cc = (header->cc | 0x1),  // Set to RDM_CC_x_COMMAND_RESPONSE
    .pid = header->pid,
    .pdl = pdl
  };
  rdm_uid_get(dmx_num, &response_header.src_uid);

  return rdm_write(dmx_num, &response_header, format, pd);
}

size_t rdm_write_nack_reason(dmx_port_t dmx_num, const rdm_header_t *header, 
                             rdm_nr_t nack_reason) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(header != NULL);
  assert(rdm_cc_is_request(header->cc));
  assert(dmx_driver_is_installed(dmx_num));

  // PDL is a single word
  const size_t pdl = sizeof(uint16_t);
  
  // Build the response header
  rdm_header_t response_header = {
    .message_len = 24 + pdl,
    .dest_uid = header->src_uid,
    .tn = header->tn,
    .response_type = RDM_RESPONSE_TYPE_NACK_REASON,
    .message_count = rdm_pd_queue_get_size(dmx_num),
    .sub_device = header->sub_device,
    .cc = (header->cc | 0x1),  // Set to RDM_CC_x_COMMAND_RESPONSE
    .pid = header->pid,
    .pdl = pdl
  };
  rdm_uid_get(dmx_num, &response_header.src_uid);

  return rdm_write(dmx_num, &response_header, "w", &nack_reason);
}

size_t rdm_send_generic(dmx_port_t dmx_num, const rdm_uid_t *dest_uid,
                        rdm_sub_device_t sub_device, rdm_pid_t pid, rdm_cc_t cc,
                        const char *format, const void *pd, size_t pdl,
                        rdm_ack_t *ack) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dest_uid != NULL);
  assert(sub_device < RDM_SUB_DEVICE_MAX || sub_device == RDM_SUB_DEVICE_ALL);
  assert(pid > 0);
  assert(rdm_cc_is_valid(cc) && rdm_cc_is_request(cc));
  // assert(rdm_pd_format_is_valid(format)); // TODO
  assert(format != NULL || pd == NULL);
  assert(pd != NULL || pdl == 0);
  assert(pdl < 231);
  assert(dmx_driver_is_installed(dmx_num));

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Attempt to take the mutex and wait until the driver is done sending
  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return 0;
  }
  if (!dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23))) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Write the header using the default arguments and the caller's arguments
  rdm_header_t header = {
    .message_len = 24 + pdl,
    .tn = rdm_get_transaction_num(dmx_num),
    .port_id = dmx_num + 1,
    .message_count = 0,
    .sub_device = sub_device,
    .cc = cc,
    .pid = pid,
    .pdl = pdl,
  };
  rdm_uid_get(dmx_num, &header.src_uid);
  memcpy(&header.dest_uid, dest_uid, sizeof(header.dest_uid));

  // Write and send the RDM request
  const size_t written = rdm_write(dmx_num, &header, format, pd);
  if (!dmx_send(dmx_num, written)) {
    xSemaphoreGiveRecursive(driver->mux);
    if (ack != NULL) {
      ack->err = DMX_OK;
      ack->size = 0;
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->pid = 0;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->message_count = 0;
      ack->pdl = 0;
    }
    return 0;
  }

  // Return early if no response is expected
  if (rdm_uid_is_broadcast(dest_uid) && pid != RDM_PID_DISC_UNIQUE_BRANCH) {
    dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23));
    xSemaphoreGiveRecursive(driver->mux);
    if (ack != NULL) {
      ack->err = DMX_OK;
      ack->size = 0;
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->pid = 0;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->message_count = 0;
      ack->pdl = 0;
    }
    return 0;
  }

  // Attempt to receive the RDM response
  dmx_packet_t packet;
  size_t size = dmx_receive(dmx_num, &packet, pdDMX_MS_TO_TICKS(23));
  if (ack != NULL) {
    ack->err = packet.err;
    ack->size = size;
  }

  // Return early if no response was received
  if (size == 0) {
    xSemaphoreGiveRecursive(driver->mux);
    if (ack != NULL) {
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->pid = 0;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->message_count = 0;
      ack->pdl = 0;
    }
    return 0;
  }

  // Return early if the response checksum was invalid
  if (!rdm_read_header(dmx_num, &header)) {
    xSemaphoreGiveRecursive(driver->mux);
    if (ack != NULL) {
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->pid = 0;
      ack->type = RDM_RESPONSE_TYPE_INVALID;
      ack->message_count = 0;
      ack->pdl = 0;
    }
    return 0;
  }

  // Copy the results into the ack struct
  if (ack != NULL) {
    memcpy(&ack->src_uid, &header.src_uid, sizeof(rdm_uid_t));
    ack->pid = header.pid;
    if (!rdm_response_type_is_valid(header.response_type)) {
      ack->type = RDM_RESPONSE_TYPE_INVALID;
      ack->pdl = header.pdl;
    } else {
      const char *word_format = "w";
      ack->type = header.response_type;
      if (header.response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
        uint16_t timer;
        rdm_read_pd(dmx_num, word_format, &timer, sizeof(timer));
        ack->timer = pdDMX_MS_TO_TICKS(timer * 10);
      } else if (header.response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
        uint16_t nack_reason;
        rdm_read_pd(dmx_num, word_format, &nack_reason, sizeof(nack_reason));
        ack->nack_reason = nack_reason;
      } else {
        ack->pdl = header.pdl;
      }
    }
    ack->message_count = header.message_count;
  }

  xSemaphoreGiveRecursive(driver->mux);
  return header.pdl;
}

void rdm_set_boot_loader(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
}

size_t rdm_get_transaction_num(dmx_port_t dmx_num) {
  assert(dmx_num < DMX_NUM_MAX);
  assert(dmx_driver_is_installed(dmx_num));

  size_t tn;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  tn = dmx_driver[dmx_num]->tn;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return tn;
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