#include "utils.h"

#include "rdm/types.h"

void *uidcpy(void *dest, const rdm_uid_t *uid) {
  ((uint8_t *)dest)[0] = ((uint8_t *)uid)[5];
  ((uint8_t *)dest)[1] = ((uint8_t *)uid)[4];
  ((uint8_t *)dest)[2] = ((uint8_t *)uid)[3];
  ((uint8_t *)dest)[3] = ((uint8_t *)uid)[2];
  ((uint8_t *)dest)[4] = ((uint8_t *)uid)[1];
  ((uint8_t *)dest)[5] = ((uint8_t *)uid)[0];
  return dest;
}

bool uid_is_broadcast(rdm_uid_t uid) {
  return (uint32_t)uid == 0xffffffff;
}

bool uid_is_recipient(rdm_uid_t compare_uid, rdm_uid_t recipient_uid) {
  compare_uid &= 0xffffffffffff;
  recipient_uid &= 0xffffffffffff;
  return recipient_uid == compare_uid ||
         ((compare_uid >> 32 == 0xffff ||
           compare_uid >> 32 == recipient_uid >> 32) &&
          (uint32_t)compare_uid == 0xffffffff);
}

size_t get_preamble_len(const void *data) {
  size_t preamble_len = 0;
  for (const uint8_t *d = data; preamble_len <= 7; ++preamble_len) {
    if (d[preamble_len] == RDM_DELIMITER) break;
  }
  return preamble_len;
}