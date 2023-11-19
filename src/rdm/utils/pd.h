/**
 * @file rdm/utils/pd.h
 * @author Mitch Weisbrod
 * @brief // TODO
 *
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "dmx/types.h"
#include "rdm/responder.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum rdm_pd_storage_type_e {
  RDM_PD_FLAG_VARIABLE,
  RDM_PD_FLAG_DETERMINISTIC,
  RDM_PD_FLAG_STATIC,
};

enum rdm_pd_flags_e {
  RDM_PD_FLAG_NVS = BIT4,
  RDM_PD_FLAG_UPDATED = BIT5,
  RDM_PD_FLAG_QUEUED = BIT6,
};

typedef struct rdm_pd_limit_s {
  uint32_t max;
  uint32_t min;
} rdm_pd_limit_t;

typedef struct rdm_pd_definition_s rdm_pd_definition_t;

/**
 * @brief A function type for RDM responder callbacks. This is the type of
 * function that is called when responding to RDM requests.
 */
typedef rdm_response_type_t (*rdm_response_handler_t)(
    dmx_port_t dmx_num, const rdm_pd_definition_t *def, rdm_header_t *header,
    void *pd, uint8_t *pdl_out);

typedef struct rdm_pd_definition_s {
  rdm_pid_t pid;
  rdm_ds_t ds;
  rdm_pid_cc_t pid_cc;
  struct rdm_pd_format_s {
    struct rdm_pd_format_cc_s {
      const char *request;
      const char *response;
    } get, set;
  } format;
  rdm_pd_limit_t limit;
  rdm_response_handler_t response_handler;
  bool non_volatile;
  rdm_units_t units;
  rdm_prefix_t prefix;
  uint32_t default_value;
  const char *description;
} rdm_pd_definition_t;

// TODO: docs
typedef size_t (*rdm_pd_getter_t)(dmx_port_t dmx_num,
                                  rdm_sub_device_t sub_device,
                                  void *destination, size_t dest_size,
                                  const void *args);

const void *rdm_pd_add_variable(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                                const rdm_pd_definition_t *definition,
                                const void *init_value, size_t size);

const void *rdm_pd_add_alias(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                             const rdm_pd_definition_t *definition,
                             rdm_pid_t alias, size_t offset, size_t size);

rdm_pd_getter_t rdm_pd_add_deterministic(dmx_port_t dmx_num,
                                         rdm_sub_device_t sub_device,
                                         const rdm_pd_definition_t *definition,
                                         rdm_pd_getter_t getter);

bool rdm_pd_update_callback(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, rdm_callback_t callback,
                            void *context);

bool rdm_pd_exists(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                   rdm_pid_t pid);

// TODO: determine if this is the right function signature
uint32_t rdm_pd_list(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                     void *destination, size_t size);

const void *rdm_pd_get(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                       rdm_pid_t pid, void *destination, size_t dest_size,
                       const void *args);

size_t rdm_pd_get_size(const char *format);

size_t rdm_pd_set(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                  rdm_pid_t pid, const void *data, size_t size);

size_t rdm_pd_set_and_queue(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                            rdm_pid_t pid, const void *data, size_t size);

size_t rdm_pd_serialize(void *destination, size_t len, const char *format,
                        const void *source);

size_t rdm_pd_deserialize(void *destination, size_t len, const char *format,
                          const void *source);

/**
 * @brief Serializes a 16-bit word into a destination. Used as a convenience
 * function for quickly writing NACK reasons and timer values.
 *
 * @param[out] destination A pointer to a destination buffer.
 * @param word The word to write.
 * @return The size of the word which was written. Is always 2.
 */
size_t rdm_pd_serialize_word(void *destination, uint16_t word);

size_t rdm_pd_get_description(dmx_port_t dmx_num, rdm_sub_device_t sub_device,
                              rdm_pid_t pid,
                              rdm_pid_description_t *description);

rdm_response_type_t rdm_response_handler_simple(dmx_port_t dmx_num,
                                                const rdm_pd_definition_t *def,
                                                rdm_header_t *header, void *pd,
                                                uint8_t *pdl_out);

#ifdef __cplusplus
}
#endif
