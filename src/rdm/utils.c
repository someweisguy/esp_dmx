#include "utils.h"

#include <ctype.h>
#include <string.h>

#include "dmx/driver.h"
#include "dmx/hal.h"
#include "endian.h"
#include "esp_dmx.h"
#include "esp_log.h"
#include "rdm/types.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_mac.h"
#endif

/**
 * @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID (as
 * long as it is used responsibly) or may choose to register their own
 * manufacturer ID.
 */
#define RDM_MAN_ID_DEFAULT (0x05e0)

#ifndef CONFIG_RDM_DEVICE_UID_MAN_ID
#define CONFIG_RDM_DEVICE_UID_MAN_ID RDM_MAN_ID_DEFAULT
#endif
#ifndef CONFIG_RDM_DEVICE_UID_DEV_ID
#define CONFIG_RDM_DEVICE_UID_DEV_ID 0xffffffff
#endif

static spinlock_t rdm_spinlock = portMUX_INITIALIZER_UNLOCKED;
static bool rdm_is_identifying = false;
static rdm_uid_t rdm_binding_uid = {};

static const char *TAG = "rdm_utils";

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

void *uidcpy(void *restrict destination, const void *restrict source) {
  *(uint16_t *)destination = bswap16(*(uint16_t *)source);
  *(uint32_t *)(destination + 2) = bswap32(*(uint32_t *)(source + 2));
  return destination;
}

void *uidmove(void *destination, const void *source) {
  const rdm_uid_t temp = {.man_id = ((rdm_uid_t *)source)->man_id,
                          .dev_id = ((rdm_uid_t *)source)->dev_id};
  return uidcpy(destination, &temp);
}

void uid_get(dmx_port_t dmx_num, rdm_uid_t *uid) {
  // Initialize the binding UID if it isn't initialized
  taskENTER_CRITICAL(&rdm_spinlock);
  if (uid_is_null(&rdm_binding_uid)) {
    uint32_t dev_id;
#if CONFIG_RDM_DEVICE_UID_DEV_ID == 0xffffffff
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    dev_id = bswap32(*(uint32_t *)(mac + 2));
#else
    dev_id = CONFIG_RDM_DEVICE_UID_DEV_ID;
#endif
    rdm_binding_uid.man_id = CONFIG_RDM_DEVICE_UID_MAN_ID;
    rdm_binding_uid.dev_id = dev_id;
  }
  taskEXIT_CRITICAL(&rdm_spinlock);

  // Return early if there is an argument error
  if (dmx_num >= DMX_NUM_MAX || uid == NULL) {
    return;
  }

  // Copy the binding UID and increment the final octet by dmx_num
  uid->man_id = rdm_binding_uid.man_id;
  uid->dev_id = rdm_binding_uid.dev_id;
  uint8_t last_octet = (uint8_t)rdm_binding_uid.dev_id;
  last_octet += dmx_num;
  uid->dev_id &= 0xffffff00;
  uid->dev_id |= last_octet;
}

static size_t rdm_param_parse(const char *format, bool *is_singleton) {
  *is_singleton = (*format == '\0');
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
      if (f[1] != '\0' && f[1] != '$') {
        ESP_LOGE(TAG, "Optional UID not at end of parameter.");
        return 0;
      }
      *is_singleton = true;  // Can't declare parameter array with optional UID
      field_size = sizeof(rdm_uid_t);
    } else if (*f == 'a' || *f == 'A') {
      // Handle ASCII string
      char *end_ptr;
      const bool str_has_fixed_len = isdigit((int)f[1]);
      field_size = str_has_fixed_len ? (size_t)strtol(&f[1], &end_ptr, 10) : 32;
      if (!str_has_fixed_len && (f[1] != '\0' && f[1] != '$')) {
        ESP_LOGE(TAG, "Variable-length string not at end of parameter.");
        return -1;
      } else if (str_has_fixed_len) {
        if (field_size == 0) {
          ESP_LOGE(TAG, "Fixed-length string has no size.");
          return 0;
        } else if (field_size > (231 - param_size)) {
          ESP_LOGE(TAG, "Fixed-length string is too big.");
          return 0;
        }
      }
      if (str_has_fixed_len) {
        f = end_ptr;
      } else {
        *is_singleton = true;
      }
    } else if (*f == '#') {
      // Handle integer literal
      ++f;  // Ignore '#' character
      int num_chars = 0;
      for (; num_chars <= 16; ++num_chars) {
        if (!isxdigit((int)f[num_chars])) break;
      }
      if (num_chars > 16) {
        ESP_LOGE(TAG, "Integer literal is too big");
        return 0;
      }
      field_size = (num_chars / 2) + (num_chars % 2);
      f += num_chars;
      if (*f != 'h' && *f != 'H') {
        ESP_LOGE(TAG, "Improperly terminated integer literal.");
        return 0;
      }
    } else if (*f == '$') {
      if (f[1] != '\0') {
        ESP_LOGE(TAG, "Improperly placed end-of-parameter anchor.");
        return 0;
      }
      *is_singleton = true;
    } else {
      ESP_LOGE(TAG, "Unknown symbol '%c' found at index %i.", *f, f - format);
      return 0;
    }

    // Ensure format size doesn't exceed MDB size.
    if (param_size + field_size > 231) {
      ESP_LOGE(TAG, "Parameter is too big.");
      return 0;
    }
    param_size += field_size;
  }
  return param_size;
}

size_t pd_emplace(void *destination, const char *format, const void *source,
                  size_t num, bool emplace_nulls) {
  // Clamp the size to the maximum parameter data length
  if (num > 231) {
    num = 231;
  }

  // Ensure that the format string syntax is correct
  bool param_is_singleton;
  const int param_size = rdm_param_parse(format, &param_is_singleton);
  if (param_size < 1) {
    return 0;
  }

  // Get the number of parameters that can be encoded
  const int num_params_to_copy = param_is_singleton ? 1 : num / param_size;

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
            uid_is_null(source + n)) {
          break;  // Optional UIDs will be at end of parameter string
        }
        if (destination != NULL && source != NULL) {
          uidmove(destination + n, source + n);
        }
        n += sizeof(rdm_uid_t);
      } else if (*f == 'a' || *f == 'A') {
        size_t len = atoi(f + 1);
        if (len == 0 && source != NULL) {
          // Field is a variable-length string
          const size_t str_size = num - (emplace_nulls ? 1 : 0);
          const size_t max_len = (str_size - n) < 32 ? (str_size - n) : 32;
          len = strnlen(source + n, max_len);
        }
        if (destination != NULL && source != NULL) {
          memmove(destination + n, source + n, len);
        }
        if (emplace_nulls) {
          if (destination != NULL) {
            *((uint8_t *)destination + len) = '\0';
          }
          ++n;  // Null terminator was encoded
        }
        n += len;
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

size_t pd_emplace_word(void *destination, uint16_t word) {
  *(uint16_t *)destination = bswap16(word);
  return sizeof(word);
}

bool rdm_disc_mute_get(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  bool is_muted;
  taskENTER_CRITICAL(spinlock);
  is_muted = driver->rdm.discovery_is_muted;
  taskEXIT_CRITICAL(spinlock);

  return is_muted;
}

void rdm_disc_mute_set(dmx_port_t dmx_num, const bool mute) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  
  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);
  driver->rdm.discovery_is_muted = mute;
  taskEXIT_CRITICAL(spinlock);
}

bool rdm_identify_get() {
  bool identify;
  taskENTER_CRITICAL(&rdm_spinlock);
  identify = rdm_is_identifying;
  taskEXIT_CRITICAL(&rdm_spinlock);

  return identify;
}

void rdm_identify_set(const bool identify) {
  taskENTER_CRITICAL(&rdm_spinlock);
  rdm_is_identifying = identify;
  taskEXIT_CRITICAL(&rdm_spinlock);
}

size_t rdm_read(dmx_port_t dmx_num, rdm_header_t *header, void *pd,
                size_t num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t read = 0;

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get pointers to driver data buffer locations and declare checksum
  uint16_t checksum = 0;
  const uint8_t *header_ptr = driver->data.buffer;
  const void *pd_ptr = header_ptr + 24;

  taskENTER_CRITICAL(spinlock);

  // Verify start code and sub-start code are correct
  if (*(uint16_t *)header_ptr != (RDM_SC | (RDM_SUB_SC << 8)) &&
      *header_ptr != RDM_PREAMBLE && *header_ptr != RDM_DELIMITER) {
    taskEXIT_CRITICAL(spinlock);
    return read;
  }

  // Get and verify the preamble_len if packet is a discovery response
  size_t preamble_len = 0;
  if (*header_ptr == RDM_PREAMBLE || *header_ptr == RDM_DELIMITER) {
    for (; preamble_len <= 7; ++preamble_len) {
      if (header_ptr[preamble_len] == RDM_DELIMITER) break;
    }
    if (preamble_len > 7) {
      taskEXIT_CRITICAL(spinlock);
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
      taskEXIT_CRITICAL(spinlock);
      return read;
    }

    // Copy the header and pd from the driver
    if (header != NULL) {
      pd_emplace(header, "#cc01hbuubbbwbwb", header_ptr, sizeof(*header), true);
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
      taskEXIT_CRITICAL(spinlock);
      return read;
    }

    // Decode the EUID
    uint8_t buf[6];
    for (int i = 0, j = 0; i < 6; ++i, j += 2) {
      buf[i] = header_ptr[j] & header_ptr[j + 1];
    }

    // Copy the data into the header
    if (header != NULL) {
      uidcpy(&header->src_uid, buf);
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

  taskEXIT_CRITICAL(spinlock);

  return read;
}

size_t rdm_write(dmx_port_t dmx_num, rdm_header_t *header, const void *pd) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL || pd != NULL, 0,
            "header is null and pd does not contain a UID");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t written = 0;

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get pointers to driver data buffer locations and declare checksum
  uint16_t checksum = 0;
  uint8_t *header_ptr = driver->data.buffer;
  void *pd_ptr = header_ptr + 24;

  taskENTER_CRITICAL(spinlock);

  // RDM writes must be synchronous to prevent data corruption
  if (driver->is_sending) {
    taskEXIT_CRITICAL(spinlock);
    return written;
  } else if (dmx_uart_get_rts(driver->uart) == 1) {
    dmx_uart_set_rts(driver->uart, 0);  // Stops writes from being overwritten
  }

  if (header != NULL && !(header->cc == RDM_CC_DISC_COMMAND_RESPONSE &&
                          header->pid == RDM_PID_DISC_UNIQUE_BRANCH)) {
    // Copy the header, pd, message_len, and pdl into the driver
    const size_t copy_size = header->pdl <= 231 ? header->pdl : 231;
    header->message_len = copy_size + 24;
    pd_emplace(header_ptr, "#cc01hbuubbbwbwb", header, sizeof(*header), false);
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
      uidcpy(uid, &header->src_uid);
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
  driver->data.tx_size = written;

  taskEXIT_CRITICAL(spinlock);

  return written;
}

bool rdm_request(dmx_port_t dmx_num, rdm_header_t *header, const void *pd_in,
                 void *pd_out, size_t num, rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(!uid_is_null(&header->dest_uid), 0, "dest_uid is invalid");
  DMX_CHECK(!uid_is_broadcast(&header->src_uid), 0, "src_uid is invalid");
  DMX_CHECK(header->cc == RDM_CC_DISC_COMMAND ||
                header->cc == RDM_CC_GET_COMMAND ||
                header->cc == RDM_CC_SET_COMMAND,
            0, "cc is invalid");
  DMX_CHECK(
      header->sub_device < 513 || (header->sub_device == RDM_SUB_DEVICE_ALL &&
                                   header->cc != RDM_CC_GET_COMMAND),
      0, "sub_device is invalid");
  DMX_CHECK(header->pdl <= 231, 0, "pdl is invalid");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Update the optional components of the header to allowed values
  if (header->port_id == 0) {
    header->port_id = dmx_num + 1;
  }
  if (uid_is_null(&header->src_uid)) {
    uid_get(dmx_num, &header->src_uid);
  }

  // Set header values that the user cannot set themselves
  taskENTER_CRITICAL(spinlock);
  header->tn = driver->rdm.tn;
  taskEXIT_CRITICAL(spinlock);
  header->message_count = 0;

  // Write and sdn the response and determind if a response is expected
  const bool response_expected = !uid_is_broadcast(&header->dest_uid) ||
                                 (header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
                                  header->cc == RDM_CC_DISC_COMMAND);
  size_t size = rdm_write(dmx_num, header, pd_in);
  dmx_send(dmx_num, size);

  // Return early if a packet error occurred or if no response was expected
  if (response_expected) {
    dmx_packet_t packet;
    size = dmx_receive(dmx_num, &packet, 2);
    if (ack != NULL) {
      ack->err = packet.err;
      ack->size = size;
    }
    if (packet.err) {
      if (ack != NULL) {
        ack->src_uid = (rdm_uid_t){0, 0};
        ack->message_count = 0;
        ack->type = RDM_RESPONSE_TYPE_INVALID;
      }
      return false;
    } else if (size == 0) {
      // TODO: remove this else if when refactoring dmx_receive()
      if (ack != NULL) {
        ack->src_uid = (rdm_uid_t){0, 0};
        ack->message_count = 0;
        ack->type = RDM_RESPONSE_TYPE_NONE;
      }
      return false;
    }
  } else {
    if (ack != NULL) {
      ack->err = ESP_OK;
      ack->size = size;
      ack->src_uid = (rdm_uid_t){0, 0};
      ack->message_count = 0;
      ack->type = RDM_RESPONSE_TYPE_NONE;
    }
    dmx_wait_sent(dmx_num, 2);
    return false;
  }

  // Handle the RDM response packet
  rdm_header_t resp;
  rdm_response_type_t response_type;
  if (!rdm_read(dmx_num, &resp, pd_out, num)) {
    response_type = RDM_RESPONSE_TYPE_INVALID;  // Data or checksum error
  } else if (resp.response_type != RDM_RESPONSE_TYPE_ACK &&
             resp.response_type != RDM_RESPONSE_TYPE_ACK_TIMER &&
             resp.response_type != RDM_RESPONSE_TYPE_NACK_REASON &&
             resp.response_type != RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
    response_type = RDM_RESPONSE_TYPE_INVALID;  // Invalid response_type
  } else if (header->pid != RDM_PID_DISC_UNIQUE_BRANCH &&
             (header->cc != (resp.cc - 1) || header->pid != resp.pid ||
              header->tn != resp.tn ||
              !uid_is_target(&resp.src_uid, &header->dest_uid) ||
              !uid_is_eq(&resp.dest_uid, &header->src_uid))) {
    response_type = RDM_RESPONSE_TYPE_INVALID;  // Invalid response format
  } else {
    response_type = resp.response_type;  // Response is ok
  }

  uint32_t decoded;
  // Handle the response based on the response type
  if (response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
    // Get and convert the estimated response time to FreeRTOS ticks
    decoded = pdMS_TO_TICKS(bswap16(*(uint16_t *)pd_out) * 10);
  } else if (response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
    // Get and report the received NACK reason
    decoded = bswap16(*(uint16_t *)pd_out);
  } else if (response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
    ESP_LOGW(TAG, "RDM_RESPONSE_TYPE_ACK_OVERFLOW is not yet supported.");
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
  }

  return (response_type == RDM_RESPONSE_TYPE_ACK);
}

bool rdm_register_response(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                           const rdm_pid_description_t *desc,
                           rdm_response_cb_t callback, void *param,
                           void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(sub_device < 513, false, "sub_device error");
  DMX_CHECK(desc != NULL, false, "desc is null");
  DMX_CHECK(callback != NULL, false, "callback is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  if (sub_device != RDM_SUB_DEVICE_ROOT) {
    ESP_LOGE(TAG, "Responses for multiple sub-devices are not yet supported.");
    return false;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Iterate the callback list to see if a callback with this PID exists
  int i = 0;
  for (; i < driver->rdm.num_cbs; ++i) {
    if (driver->rdm.cbs[i].desc.pid == desc->pid) break;
  }

  // Check if there is space for callbacks
  if (i == RDM_RESPONDER_MAX_PIDS) {
    ESP_LOGE(TAG, "No more space for RDM callbacks");
    return false;
  }

  // Add the requested callback to the callback list
  driver->rdm.cbs[i].param = param;
  driver->rdm.cbs[i].context = context;
  driver->rdm.cbs[i].cb = callback;
  driver->rdm.cbs[i].desc = *desc;
  ++driver->rdm.num_cbs;

  return true;
}
