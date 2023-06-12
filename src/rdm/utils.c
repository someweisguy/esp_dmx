#include "utils.h"

#include "rdm/types.h"
#include "endian.h"

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

size_t rdm_encode(rdm_mdb_t *mdb, const char *format, const void *pd,
                  size_t pdl) {
  size_t written = 0;
  bool param_is_optional = false;
  while (written < pdl) {
    for (const char *f = format; *f != '\0' && written < pdl; ++f) {
      if (*f == '?') {
        param_is_optional = true;  // TODO
        continue;
      } else if (*f == 'b') {
        // 8-bit
        *(uint8_t *)(&mdb->pd[written]) = *(uint8_t *)(pd + written);
        written += sizeof(uint8_t);
      } else if (*f == 'w') {
        // 16-bit
        *(uint16_t *)(&mdb->pd[written]) = bswap16(*(uint16_t *)(pd + written));
        written += sizeof(uint16_t);
      } else if (*f == 'd') {
        // 32-bit
        *(uint32_t *)(&mdb->pd[written]) = bswap32(*(uint32_t *)(pd + written));
        written += sizeof(uint32_t);
      } else if (*f == 'u') {
        // 48-bit UID
        uidmove((rdm_uid_t *)(&mdb->pd[written]), (rdm_uid_t *)(pd + written));
        written += sizeof(rdm_uid_t);
      } else if (*f == 'a') {
        const size_t max_len = (pdl - written) < 32 ? (pdl - written) : 32;
        size_t len = strnlen((char *)(pd + written), max_len);
        memmove(&mdb->pd[written], pd + written, len);  // Strip terminator
        written += len;
      } // TODO: encode literals
      param_is_optional = false;
    }
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