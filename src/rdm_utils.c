#include "rdm_utils.h"

#include "rdm_types.h"

void *uidcpy(void *dest, const rdm_uid_t *uid) {
  ((uint8_t *)dest)[0] = ((uint8_t *)uid)[5];
  ((uint8_t *)dest)[1] = ((uint8_t *)uid)[4];
  ((uint8_t *)dest)[2] = ((uint8_t *)uid)[3];
  ((uint8_t *)dest)[3] = ((uint8_t *)uid)[2];
  ((uint8_t *)dest)[4] = ((uint8_t *)uid)[1];
  ((uint8_t *)dest)[5] = ((uint8_t *)uid)[0];
  return dest;
}

size_t get_preamble_len(const void *data) {
  size_t preamble_len = 0;
  for (const uint8_t *d = data; preamble_len <= 7; ++preamble_len) {
    if (d[preamble_len] == RDM_DELIMITER) break;
  }
  return preamble_len;
}