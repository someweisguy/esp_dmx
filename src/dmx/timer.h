/**
 * @file timer.h
 * @author Mitch Weisbrod
 * @brief This file is the timer Hardware Abstraction Layer (HAL) of esp_dmx. It
 * contains low-level functions to perform tasks relating to the timer hardware.
 * The timer hardware is used for generating the timing for the DMX break and
 * mark-after-break. It is also used for RDM timing to ensure that proper packet
 * spacing is maintained on the RDM bus. This file is not considered part of the
 * API and should not be included by the user.
 */
#pragma once

#include "dmx_types.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/gptimer.h"
#include "esp_timer.h"
#else
#include "driver/timer.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RDM packet spacing constants. These values are the length of time in
 * microseconds between RDM packets that must elapse in certain situations.
 * These constants are defined in page 9 through 11 of the ANSI/ESTA e1.20
 * document.
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
 * @param[inout] isr_context Context to be used in in the DMX timer ISR.
 * @param isr_flags Interrupt flags to be used for the DMX timer ISR.
 * @return A handle to the DMX timer or NULL on failure.
 */
dmx_timer_handle_t dmx_timer_init(dmx_port_t dmx_num, void *isr_context,
                                  int isr_flags);

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
 * @param auto_reload Set to true to automatically reload the alarm when the
 * alarm is triggered.
 */
void dmx_timer_set_alarm(dmx_timer_handle_t timer, uint64_t alarm,
                         bool auto_reload);

/**
 * @brief Starts the DMX timer.
 *
 * @param timer A handle to the DMX timer.
 */
void dmx_timer_start(dmx_timer_handle_t timer);

/**
 * @brief Gets the number of microseconds that have elapsed since boot.
 * 
 * @return The number of microseconds since boot. 
 */
int64_t dmx_timer_get_micros_since_boot();

#ifdef __cplusplus
}
#endif
