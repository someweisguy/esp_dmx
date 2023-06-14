#include "utils.h"

#include <string.h>
#include <ctype.h>

#include "rdm/types.h"
#include "endian.h"
#include "esp_log.h"

void *uidcpy(void *restrict destination, const void *restrict source) {
  *(uint16_t *)destination = bswap16(*(uint16_t *)source);
  *(uint32_t *)(destination + 2) = bswap32(*(uint32_t *)(source + 2));
  return destination;
}

void *uidmove(void *destination, const void *source) {
  const rdm_uid_t temp = {
    .man_id = ((rdm_uid_t *)source)->man_id,
    .dev_id = ((rdm_uid_t *)source)->dev_id
  };
  return uidcpy(destination, &temp);
}

inline bool uid_is_eq(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id == b->man_id && a->dev_id == b->dev_id;
}

inline bool uid_is_lt(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id < b->man_id ||
         (a->man_id == b->man_id && a->dev_id < b->dev_id);
}

inline bool uid_is_gt(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id > b->man_id ||
         (a->man_id == b->man_id && a->dev_id > b->dev_id);
}

inline bool uid_is_le(const rdm_uid_t *a, const rdm_uid_t *b) {
  return !uid_is_gt(a, b);
}

inline bool uid_is_ge(const rdm_uid_t *a, const rdm_uid_t *b) {
  return !uid_is_lt(a, b);
}

inline bool uid_is_broadcast(const rdm_uid_t *uid) {
  return uid->dev_id == 0xffffffff;
}

bool uid_is_null(const rdm_uid_t *uid) {
  return uid->man_id == 0 && uid->dev_id == 0;
}

inline bool uid_is_target(const rdm_uid_t *uid, const rdm_uid_t *alias) {
  return ((alias->man_id == 0xffff || alias->man_id == uid->man_id) &&
          alias->dev_id == 0xffffffff) ||
         uid_is_eq(uid, alias);
}

static int rdm_param_parse(const char *format, bool *is_singleton) {
  const char *TAG = "RDM Parameter Error";
  *is_singleton = (*format == '\0');
  int param_size = 0;
  for (const char *f = format; *f != '\0'; ++f) {
    size_t field_size = 0;
    if (*f == 'b' || *f == 'B') {
      field_size = sizeof(uint8_t);  // Handle 8-bit byte.
    } else if (*f == 'w' || *f == 'W') {
      field_size = sizeof(uint16_t);  // Handle 16-bit word.
    } else if (*f == 'd' || *f == 'D') {
      field_size = sizeof(uint32_t);  // Handle 32-bit dword.
    } else if (*f == 'u' || *f == 'U') {
      field_size = sizeof(rdm_uid_t);  // Handle 48-bit UID.
    } else if (*f == 'v' || *f == 'V') {
      if (f[1] != '\0' && f[1] != '$') {
        ESP_LOGE(TAG, "Optional UID not at end of parameter.");
        return 0;
      }
      *is_singleton = true;  // Can't declare parameter array with optional UID
      field_size = sizeof(rdm_uid_t);
    } else if (*f == 'a' || *f == 'A') {
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
        ESP_LOGE(TAG, "Improperly placed end-of-string anchor.");
        return 0;
      }
      *is_singleton = true;
    } else {
      ESP_LOGE(TAG, "Unknown symbol '%c' in parameter string at index %i.", *f,
               f - format);
      return 0;
    }

    // Ensure format size doesn't exceed MDB size.
    if (param_size + field_size > 231) {
      ESP_LOGE(TAG, "Parameter string is too big.");
      return 0;
    }
    param_size += field_size;
  }
  return param_size;
}

size_t rdmcpy(void *destination, const char *format, const void *source,
              size_t size, const bool copy_nulls) {
  // TODO: arg check

  // Ensure that the format string syntax is correct
  bool param_is_singleton;
  const int param_size = rdm_param_parse(format, &param_is_singleton);
  if (param_size < 1) return 0;

  // Get the number of parameters that can be encoded
  const int num_params_to_copy = param_is_singleton ? 1 : size / param_size;

  // Encode the fields into the destination
  size_t n = 0;
  for (int i = 0; i < num_params_to_copy; ++i) {
    for (const char *f = format; *f != '\0'; ++f) {
      ESP_LOGW("test", "found %c", *f);
      if (*f == 'b' || *f == 'B') {
        *(uint8_t *)(destination + n) = *(uint8_t *)(source + n);
        n += sizeof(uint8_t);
      } else if (*f == 'w' || *f == 'W') {
        *(uint16_t *)(destination + n) = bswap16(*(uint16_t *)(source + n));
        n += sizeof(uint16_t);
      } else if (*f == 'd' || *f == 'D') {
        *(uint32_t *)(destination + n) = bswap32(*(uint32_t *)(source + n));
        n += sizeof(uint32_t);
      } else if (*f == 'u' || *f == 'U' || *f == 'v' || *f == 'V') {
        if ((*f == 'v' || *f == 'V') && !copy_nulls &&
            uid_is_null(source + n)) {
          break;  // Optional UIDs must be at end of parameter string
        }
        uidcpy(destination + n, source + n);
        n += sizeof(rdm_uid_t);
      } else if (*f == 'a' || *f == 'A') {
        size_t len = atoi(f + 1);
        if (len == 0) {
          // Field is a variable-length string
          const size_t max_len = (size - n) < 32 ? (size - n) : 32;
          len = strnlen(source + n, max_len);
        }
        if (copy_nulls) {
          strncpy(destination + n, source + n, len);
          ++n;  // Null terminator was encoded
        } else {
          memcpy(destination + n, source + n, len);
        }
        n += len;
      } else if (*f == '#') {
        ++f;  // Skip '#' character
        char *end_ptr;
        const uint64_t literal = strtol(f, &end_ptr, 16);
        const int literal_len = ((end_ptr - f) / 2) + ((end_ptr - f) % 2);
        for (int j = 0, k = literal_len - 1; j < literal_len; ++j, --k) {
          ((uint8_t *)destination + n)[j] = ((uint8_t *)&literal)[k];
        }
        f = end_ptr;
        n += literal_len;
      }
    }
  }
  return n;
}

size_t get_preamble_len(const void *data) {
  size_t preamble_len = 0;
  for (const uint8_t *d = data; preamble_len <= 7; ++preamble_len) {
    if (d[preamble_len] == RDM_DELIMITER) break;
  }
  return preamble_len;
}