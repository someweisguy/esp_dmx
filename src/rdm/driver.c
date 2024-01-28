#include <ctype.h>
#include <string.h>

#include "dmx/hal/include/timer.h"
#include "dmx/hal/include/uart.h"
#include "dmx/include/driver.h"
#include "dmx/include/parameter.h"
#include "dmx/include/service.h"
#include "endian.h"
#include "rdm/include/driver.h"
#include "rdm/include/uid.h"

static size_t rdm_format_encode(void *restrict dest,
                                const char *restrict format,
                                const void *restrict src, size_t src_size,
                                bool encode_nulls) {
  assert(dest != NULL);
  assert(rdm_format_is_valid(format));
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
        c += 'A';  // Convert token to lowercase
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

  const uint8_t *data = driver->dmx.data;
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
    if (checksum != (((data[12] & data[13]) << 8) | (data[14] & data[15]))) {
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
      header->message_len = preamble_len + 17;
      header->dest_uid.man_id = RDM_UID_BROADCAST_ALL.man_id;
      header->dest_uid.dev_id = RDM_UID_BROADCAST_ALL.dev_id;
      header->src_uid.man_id = bswap16(header->src_uid.man_id);
      header->src_uid.dev_id = bswap32(header->src_uid.dev_id);
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
  DMX_CHECK(rdm_format_is_valid(format), 0, "format is invalid");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Guard against invalid PDL
  const size_t pdl = driver->dmx.data[23];
  if (pdl == 0 || pdl > 231) {
    return 0;
  }

  // Deserialize the parameter data into the destination buffer
  if (destination != NULL) {
    size = pdl < size ? pdl : size;
    const bool encode_nulls = true;
    const uint8_t *pd = &driver->dmx.data[24];
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
  } else {
    DMX_CHECK(rdm_response_type_is_valid(header->response_type), 0,
              "header->response_type error");
  }
  DMX_CHECK(rdm_format_is_valid(format), 0, "format is invalid");
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
    memset(driver->dmx.data, RDM_PREAMBLE, preamble_len);
    driver->dmx.data[preamble_len] = RDM_DELIMITER;
    uint8_t *data = &driver->dmx.data[preamble_len + 1];

    // Encode the UID and calculate the checksum
    uint8_t uid[6];
    ((rdm_uid_t *)uid)->man_id = bswap16(header->src_uid.man_id);
    ((rdm_uid_t *)uid)->dev_id = bswap32(header->src_uid.dev_id);
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
    rdm_format_encode(driver->dmx.data, header_format, header, sizeof(*header),
                      encode_nulls);
    size_t message_len;
    void *data = &driver->dmx.data[24];
    if (pd != NULL && header->pdl > 0) {
      size_t pdl =
          rdm_format_encode(data, format, pd, header->pdl, encode_nulls);
      if (header->pdl != pdl) {
        message_len = 24 + pdl;
        driver->dmx.data[2] = message_len;  // Encode updated message_len
        driver->dmx.data[23] = pdl;         // Encode updated pdl
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
      checksum += driver->dmx.data[i];
    }
    checksum = bswap16(checksum);
    memcpy(data, &checksum, sizeof(checksum));

    written = message_len + 2;
  }

  return written;
}

bool rdm_format_is_valid(const char *format) {
  if (format == NULL) {
    return true;
  }

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
            return false;  // Hex literals must be 2 characters wide
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
        return false;  // Unknown symbol
    }

    // Update the parameter size with the new token
    parameter_size += token_size;
    if (parameter_size > 231) {
      return false;  // Parameter size is too big
    }

    // End loop if parameter is terminated
    if (format_is_terminated) {
      break;
    }
  }

  if (format_is_terminated) {
    ++format;
    if (*format != '\0' && *format != '$') {
      return false;  // Invalid token after terminator
    }
  } else {
    // Get the maximum possible size if parameter is unterminated
    parameter_size = 231 - (231 % parameter_size);
  }

  return parameter_size > 0;
}
