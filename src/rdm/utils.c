#include "utils.h"

#include "rdm/types.h"
#include "endian.h"

void *uidcpy(void *restrict destination, const void *restrict source) {
  *(uint16_t *)destination = bswap16(*(uint16_t *)source);
  *(uint32_t *)(destination + 2) = bswap32(*(uint32_t *)(source + 2));
  return destination;
}

bool uid_is_recipient(rdm_uid_t compare_uid, rdm_uid_t recipient_uid) {
  return uid_is_equal(recipient_uid, compare_uid) ||
         ((compare_uid.man_id == 0xffff ||
           compare_uid.man_id == recipient_uid.man_id) &&
          compare_uid.dev_id == 0xffffffff);
}

size_t get_preamble_len(const void *data) {
  size_t preamble_len = 0;
  for (const uint8_t *d = data; preamble_len <= 7; ++preamble_len) {
    if (d[preamble_len] == RDM_DELIMITER) break;
  }
  return preamble_len;
}