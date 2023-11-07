/**
 * @file rdm/responder/misc.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/types.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO docs
int rdm_response_handler_simple(dmx_port_t dmx_num, rdm_header_t *header,
                                void *pd, uint8_t *pdl_out, const char *format);

#ifdef __cplusplus
}
#endif
