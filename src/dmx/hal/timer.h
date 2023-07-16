#pragma once

#include "dmx/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
typedef struct dmx_timer_handle_t *dmx_timer_handle_t;

// TODO: docs
dmx_timer_handle_t dmx_timer_init(dmx_port_t dmx_num, void *isr_handle,
                           void *isr_context, int isr_flags);

// TODO: docs
void dmx_timer_deinit(dmx_timer_handle_t timer);

// TODO: docs
void dmx_timer_stop(dmx_timer_handle_t timer);

// TODO: docs
void dmx_timer_set_counter(dmx_timer_handle_t timer, uint64_t counter);

// TODO: docs
void dmx_timer_set_alarm(dmx_timer_handle_t timer, uint64_t alarm);

// TODO: docs
void dmx_timer_start(dmx_timer_handle_t timer);

#ifdef __cplusplus
}
#endif
