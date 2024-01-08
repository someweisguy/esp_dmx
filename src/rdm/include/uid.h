/**
 * @file rdm/include/uid.h
 * @author Mitch Weisbrod (mitch@theweisbrods.com)
 * @brief This file includes functions used for comparing RDM unique IDs.
 *
 */
#pragma once

#include "rdm/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Returns true if the UIDs are equal to each other. Is equivalent to
 * a == b.
 *
 * @param[in] a A pointer to the first operand.
 * @param[in] b A pointer to the second operand.
 * @return true if the UIDs are equal.
 * @return false if the UIDs are not equal.
 */
static inline bool rdm_uid_is_eq(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id == b->man_id && a->dev_id == b->dev_id;
}

/**
 * @brief Returns true if the first UID is less than the second UID. Is
 * equivalent to a < b.
 *
 * @param[in] a A pointer to the first operand.
 * @param[in] b A pointer to the second operand.
 * @return true if a is less than b.
 * @return false if a is not less than b.
 */
static inline bool rdm_uid_is_lt(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id < b->man_id ||
         (a->man_id == b->man_id && a->dev_id < b->dev_id);
}

/**
 * @brief Returns true if the first UID is greater than the second UID. Is
 * equivalent to a > b.
 *
 * @param[in] a A pointer to the first operand.
 * @param[in] b A pointer to the second operand.
 * @return true if a is greater than b.
 * @return false if a is not greater than b.
 */
static inline bool rdm_uid_is_gt(const rdm_uid_t *a, const rdm_uid_t *b) {
  return a->man_id > b->man_id ||
         (a->man_id == b->man_id && a->dev_id > b->dev_id);
}

/**
 * @brief Returns true if the first UID is less than or equal to the second
 * UID. Is equivalent to a <= b.
 *
 * @param[in] a A pointer to the first operand.
 * @param[in] b A pointer to the second operand.
 * @return true if a is less than or equal to b.
 * @return false if a is not less than or equal to b.
 */
static inline bool rdm_uid_is_le(const rdm_uid_t *a, const rdm_uid_t *b) {
  return !rdm_uid_is_gt(a, b);
}

/**
 * @brief Returns true if the first UID is greater than or equal to the second
 * UID. Is equivalent to a >= b.
 *
 * @param[in] a A pointer to the first operand.
 * @param[in] b A pointer to the second operand.
 * @return true if a is greater than or equal to b.
 * @return false if a is not greater than or equal to b.
 */
static inline bool rdm_uid_is_ge(const rdm_uid_t *a, const rdm_uid_t *b) {
  return !rdm_uid_is_lt(a, b);
}

/**
 * @brief Returns true if the specified UID is a broadcast address.
 *
 * @param[in] uid A pointer to the unary UID operand.
 * @return true if the UID is a broadcast address.
 * @return false if the UID is not a broadcast address.
 */
static inline bool rdm_uid_is_broadcast(const rdm_uid_t *uid) {
  return uid->dev_id == 0xffffffff;
}

/**
 * @brief Returns true if the specified UID is null.
 *
 * @param[in] uid A pointer to the unary UID operand.
 * @return true if the UID is null.
 * @return false if the UID is not null.
 */
static inline bool rdm_uid_is_null(const rdm_uid_t *uid) {
  return uid->man_id == 0 && uid->dev_id == 0;
}

/**
 * @brief Returns true if the first UID is targeted by the second UID. A
 * common usage of this function is `uid_is_target(&my_uid,
 * &destination_uid)`.
 *
 * @param[in] uid A pointer to a UID which to check is targeted.
 * @param[in] alias A pointer to a UID which may alias the first UID.
 * @return true if the UID is targeted by the alias UID.
 * @return false if the UID is not targeted by the alias UID.
 */
static inline bool rdm_uid_is_target(const rdm_uid_t *uid,
                                     const rdm_uid_t *alias) {
  return ((alias->man_id == 0xffff || alias->man_id == uid->man_id) &&
          alias->dev_id == 0xffffffff) ||
         rdm_uid_is_eq(uid, alias);
}

#ifdef __cplusplus
}
#endif
