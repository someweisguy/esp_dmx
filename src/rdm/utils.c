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

static int rdm_check_param_syntax(const char *format) {
  const char *TAG = "RDM Parameter Error";
  int format_size = 0;
  for (const char *f = format; *f != '\0'; ++f) {
    size_t param_size;
    if (*f == 'b') {
      param_size = sizeof(uint8_t);  // Handle 8-bit byte.
    } else if (*f == 'w') {
      param_size = sizeof(uint16_t);  // Handle 16-bit word.
    } else if (*f == 'd') {
      param_size = sizeof(uint32_t);  // Handle 32-bit dword.
    } else if (*f == 'u') {
      param_size = sizeof(rdm_uid_t);  // Handle 48-bit UID.
    } else if (*f == 'v') {
      if (f[1] != '\0') {
        ESP_LOGE(TAG, "Optional UID not at end of parameter.");
        return 0;
      }
      param_size = sizeof(rdm_uid_t);
    } else if (*f == 'a') {
      char *end_ptr;
      const bool str_has_fixed_len = isdigit((int)f[1]);
      param_size = str_has_fixed_len ? (size_t)strtol(&f[1], &end_ptr, 10) : 32;
      if (!str_has_fixed_len && f[1] != '\0') {
        ESP_LOGE(TAG, "Variable-length string not at end of parameter.");
        return 0;
      } else if (str_has_fixed_len) {
        if (param_size == 0) {
          ESP_LOGE(TAG, "Fixed-length string has no size.");
          return 0;
        } else if (param_size > (231 - format_size)) {
          ESP_LOGE(TAG, "Fixed-length string is too big.");
          return 0;
        }
      }
      if (str_has_fixed_len) {
        f = end_ptr;
      }
    } else if (*f == '#') {
      ++f;  // Ignore '#' character.
      int num_chars = 0;
      for (; num_chars <= 16; ++num_chars) {
        if (!isxdigit((int)f[num_chars])) break;
      }
      if (num_chars > 16) {
        ESP_LOGE(TAG, "Integer literal is too big");
        return 0;
      }
      param_size = (num_chars / 2) + (num_chars % 2);
      f += num_chars;
      if (*f != 'h') {
        ESP_LOGE(TAG, "Improperly terminated integer literal.");
        return 0;
      }
      ++f;  // Ignore 'h' character.
    } else {
      ESP_LOGE(TAG, "Unknown symbol '%c' in param string.", *f);
      return 0;
    }

    // Ensure format size doesn't exceed MDB size.
    if (format_size + param_size > 231) {
      ESP_LOGE(TAG, "Param string too big.");
      return 0;
    }
    format_size += param_size;
  }
  return format_size;
}

size_t rdm_encode(rdm_mdb_t *mdb, const char *format, const void *pd,
                  size_t pdl) {

  // Ensure that the format string syntax is correct
  if (strlen(format) == 0) return 0;
  int param_size = rdm_check_param_syntax(format);
  if (param_size == 0) return 0;

  // Get the number of parameters that can be encoded
  int num_params_to_encode;
  if (format[strlen(format) - 1] == 'v' || format[strlen(format) - 1] == 'a') {
    num_params_to_encode = 1;
  } else {
    num_params_to_encode = pdl / param_size;
  }

  // Encode the parameter data into the MDB.
  size_t written = 0;
  size_t pd_index = 0;
  while (num_params_to_encode > 0) {
    for (const char *f = format; *f != '\0'; ++f) {
      if (*f == 'b') {
        // 8-bit
        *(uint8_t *)(&mdb->pd[written]) = *(uint8_t *)(pd + pd_index);
        written += sizeof(uint8_t);
        pd_index += sizeof(uint8_t);
      } else if (*f == 'w') {
        // 16-bit
        *(uint16_t *)(&mdb->pd[written]) =
            bswap16(*(uint16_t *)(pd + pd_index));
        written += sizeof(uint16_t);
        pd_index += sizeof(uint16_t);
      } else if (*f == 'd') {
        // 32-bit
        *(uint32_t *)(&mdb->pd[written]) =
            bswap32(*(uint32_t *)(pd + pd_index));
        written += sizeof(uint32_t);
        pd_index += sizeof(uint32_t);
      } else if (*f == 'u' || *f == 'v') {
        // 48-bit UID
        if (*f == 'v' && uid_is_null((rdm_uid_t *)(pd + pd_index))) {
          pd_index += sizeof(rdm_uid_t);
          continue;
        }
        uidmove((rdm_uid_t *)(&mdb->pd[written]), (rdm_uid_t *)(pd + pd_index));
        written += sizeof(rdm_uid_t);
        pd_index += sizeof(rdm_uid_t);
      } else if (*f == 'a') {
        const size_t max_len = (pdl - written) < 32 ? (pdl - written) : 32;
        size_t len = strnlen((char *)(pd + pd_index), max_len);
        memmove(&mdb->pd[written], pd + pd_index, len);  // Strip terminator
        written += len;
        pd_index += len;
      } else if (*f == '#') {
        // Integer literal
        char *end_ptr;
        uint64_t literal = strtol(f, &end_ptr, 16);
        const int num_bytes = ((end_ptr - f) / 2) + ((end_ptr - f) % 2);
        for (int i = 0; i < num_bytes; ++i) {
          *(uint8_t *)(&mdb->pd[written + i]) = *((&literal) - (num_bytes - i));
        }
        f = end_ptr;
        written += num_bytes;
      }
    }
    --num_params_to_encode;
  }
  mdb->pdl = written;
  return written;
}

int rdm_decode(const rdm_mdb_t *mdb, const char *format, void *pd, int num) {
  int decoded = 0;



  return decoded;
}

size_t get_preamble_len(const void *data) {
  size_t preamble_len = 0;
  for (const uint8_t *d = data; preamble_len <= 7; ++preamble_len) {
    if (d[preamble_len] == RDM_DELIMITER) break;
  }
  return preamble_len;
}