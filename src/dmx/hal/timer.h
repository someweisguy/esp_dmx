#pragma once

#include "dmx/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dmx_timer_t *dmx_timer_t;

dmx_timer_t dmx_timer_init(dmx_port_t dmx_num, void *isr_handle,
                            void *isr_context, int isr_flags);

void dmx_timer_deinit(dmx_timer_t timer);

void dmx_timer_stop(dmx_timer_t timer);

void dmx_timer_set_counter(dmx_timer_t timer, uint64_t counter);

void dmx_timer_set_alarm(dmx_timer_t timer, uint64_t alarm);

#ifdef __cplusplus
}
#endif
