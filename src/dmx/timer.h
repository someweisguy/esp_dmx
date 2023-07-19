/**
 * @file timer.h
 * @author Mitch Weisbrod
 * @brief This file is the timer Hardware Abstraction Layer (HAL) of esp_dmx. It
 * contains low-level functions to perform tasks relating to the timer hardware.
 * The timer hardware is used for generating the timing for the DMX break and
 * mark-after-break. It is also used for RDM timing to ensure that proper packet
 * spacing is maintained on the RDM bus.
 */
#pragma once

#include "dmx/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RDM packet spacing constants. These values are the length of time in
 * microseconds between RDM packets that must elapse in certain situations.
 * These constants are defined in page nine of the ANSI/ESTA e1.20 document.
 */
enum rdm_packet_spacing_t {
  RDM_PACKET_SPACING_DISCOVERY_NO_RESPONSE = 5800,
  RDM_PACKET_SPACING_REQUEST_NO_RESPONSE = 3000,
  RDM_PACKET_SPACING_BROADCAST = 176,
  RDM_PACKET_SPACING_RESPONSE = 176,

  RDM_PACKET_SPACING_CONTROLLER_NO_RESPONSE = 2800,
  RDM_PACKET_SPACING_RESPONDER_NO_RESPONSE = 2000
};

/**
 * @brief A handle to the DMX timer.
 */
typedef struct dmx_timer_t *dmx_timer_handle_t;

/**
 * @brief Initializes the DMX timer.
 *
 * @param dmx_num The DMX port number.
 * @param[in] isr_handle The ISR function to be called when the timer completes.
 * @param[inout] isr_context Context to be used in in the DMX timer ISR.
 * @param isr_flags Interrupt flags to be used for the DMX timer ISR.
 * @return A handle to the DMX timer or NULL on failure.
 */
dmx_timer_handle_t dmx_timer_init(dmx_port_t dmx_num, void *isr_handle,
                                  void *isr_context, int isr_flags);

/**
 * @brief De-initializes the DMX timer.
 *
 * @param timer A handle to the DMX timer.
 */
void dmx_timer_deinit(dmx_timer_handle_t timer);

/**
 * @brief Pauses the DMX timer.
 *
 * @param timer A handle to the DMX timer.
 */
void dmx_timer_stop(dmx_timer_handle_t timer);

/**
 * @brief Sets the counter value for the DMX timer.
 *
 * @param timer A handle to the DMX timer.
 * @param counter The counter value to which to set the DMX timer.
 */
void dmx_timer_set_counter(dmx_timer_handle_t timer, uint64_t counter);

/**
 * @brief Sets the alarm value for the DMX timer.
 *
 * @param timer A handle to the DMX timer.
 * @param alarm The alarm value to which to set the DMX timer.
 */
void dmx_timer_set_alarm(dmx_timer_handle_t timer, uint64_t alarm);

/**
 * @brief Starts the DMX timer.
 *
 * @param timer A handle to the DMX timer.
 */
void dmx_timer_start(dmx_timer_handle_t timer);

#ifdef __cplusplus
}
#endif
