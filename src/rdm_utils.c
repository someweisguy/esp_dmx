#include "rdm_utils.h"

#include "rdm_types.h"

size_t rdm_get_preamble_len(const void *data) {
  size_t preamble_len = 0;
  for (const uint8_t *d = data; preamble_len <= 7; ++preamble_len) {
    if (d[preamble_len] == RDM_DELIMITER) break;
  }
  return preamble_len;
}