#include "timer.h"

#include <stdbool.h>

#include "dmx/hal.h"
#include "dmx/struct.h"

#include "driver/gpio.h"

static struct dmx_timer_t {
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_handle_t gptimer_handle;
#else
  timer_group_t group;  // The timer group to use for DMX functions.
  timer_idx_t idx;      // The timer index to use for DMX functions.
#endif
  bool is_running;
} dmx_timer_context[DMX_NUM_MAX] = {};

dmx_timer_handle_t dmx_timer_init(dmx_port_t dmx_num, void *isr_handle,
                                  void *isr_context, int isr_flags) {
  dmx_timer_handle_t timer = &dmx_timer_context[dmx_num];

  // Initialize hardware timer
#if ESP_IDF_VERSION_MAJOR >= 5
  const gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_APB,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000,  // 1MHz resolution timer
  };
  esp_err_t err = gptimer_new_timer(&timer_config, &timer->gptimer_handle);
  if (err) {
    return NULL;
  }
  const gptimer_event_callbacks_t gptimer_cb = {.on_alarm = isr_handle};
  gptimer_register_event_callbacks(timer->gptimer_handle, &gptimer_cb,
                                   isr_context);
  gptimer_enable(timer->gptimer_handle);
#else
  timer->group = dmx_num / 2;
#ifdef CONFIG_IDF_TARGET_ESP32C3
  timer->idx = 0;  // ESP32C3 uses idx 1 for Watchdog
#else
  timer->idx = dmx_num % 2;
#endif
  const timer_config_t timer_config = {
      .divider = 80,  // (80MHz / 80) == 1MHz resolution timer
      .counter_dir = TIMER_COUNT_UP,
      .counter_en = false,
      .alarm_en = true,
      .auto_reload = true,  // FIXME: set to false?
  };
  esp_err_t err = timer_init(timer->group, timer->idx, &timer_config);
  if (err) {
    return NULL;
  }
  timer_isr_callback_add(timer->group, timer->idx, isr_handle, isr_context,
                         isr_flags);
#endif
  timer->is_running = false;

  return timer;
}

void dmx_timer_deinit(dmx_timer_handle_t timer) {
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_disable(timer->gptimer_handle);
  gptimer_del_timer(timer->gptimer_handle);
#else
  timer_isr_callback_remove(timer->group, timer->idx);
  timer_deinit(timer->group, timer->idx);
#endif
  timer->is_running = false;
}

void DMX_ISR_ATTR dmx_timer_stop(dmx_timer_handle_t timer) {
  if (timer->is_running) {
#if ESP_IDF_VERSION_MAJOR >= 5
    gptimer_stop(timer->gptimer_handle);
    gptimer_set_raw_count(timer->gptimer_handle, 0);
#else
    timer_group_set_counter_enable_in_isr(timer->group, timer->idx, 0);
#endif
    timer->is_running = false;
  }
}

void DMX_ISR_ATTR dmx_timer_set_counter(dmx_timer_handle_t timer,
                                        uint64_t counter) {
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_set_raw_count(timer->gptimer_handle, counter);
#else
  timer_set_counter_value(timer->group, timer->idx, counter);
#endif
}

void DMX_ISR_ATTR dmx_timer_set_alarm(dmx_timer_handle_t timer,
                                      uint64_t alarm) {
#if ESP_IDF_VERSION_MAJOR >= 5
  const gptimer_alarm_config_t alarm_config = {
      .alarm_count = alarm,
      .reload_count = 0,
      .flags.auto_reload_on_alarm = false};
  gptimer_set_alarm_action(timer->gptimer_handle, &alarm_config);
#else
  timer_set_alarm_value(timer->group, timer->idx, alarm);
#endif
}

void DMX_ISR_ATTR dmx_timer_start(dmx_timer_handle_t timer) {
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_start(timer->gptimer_handle);
#else
  timer_start(timer->group, timer->idx);
#endif
  timer->is_running = true;
}

int64_t DMX_ISR_ATTR dmx_timer_get_micros_since_boot() {
  return esp_timer_get_time();
}