#include "timer.h"

#include <stdbool.h>

#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/gptimer.h"
#include "esp_timer.h"
#else
#include "driver/timer.h"
#endif

#ifdef CONFIG_DMX_ISR_IN_IRAM
#define DMX_ISR_ATTR IRAM_ATTR
#else
#define DMX_ISR_ATTR
#endif

static struct dmx_timer_t {
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_handle_t gptimer_handle;
#else
  timer_group_t timer_group;  // The timer group to use for DMX functions.
  timer_idx_t timer_idx;      // The timer index to use for DMX functions.
#endif
  bool is_running;
} dmx_timer_context[DMX_NUM_MAX] = {};

dmx_timer_t dmx_timer_init(dmx_port_t dmx_num, void *isr_handle,
                           void *isr_context, int isr_flags) {
  dmx_timer_t timer = &dmx_timer_context[dmx_num];
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
  timer->timer_group = dmx_num / 2;
#ifdef CONFIG_IDF_TARGET_ESP32C3
  timer->timer_idx = 0;  // ESP32C3 uses timer_idx 1 for Watchdog
#else
  timer->timer_idx = dmx_num % 2;
#endif
  const timer_config_t timer_config = {
      .divider = 80,  // (80MHz / 80) == 1MHz resolution timer
      .counter_dir = TIMER_COUNT_UP,
      .counter_en = false,
      .alarm_en = true,
      .auto_reload = true,
  };
  esp_err_t err =
      timer_init(timer->timer_group, timer->timer_idx, &timer_config);
  if (err) {
    return NULL;
  }
  timer_isr_callback_add(driver->timer_group, driver->timer_idx, isr_handle,
                         isr_context, intr_flags);
#endif
  timer->is_running = false;

  return timer;
}

void dmx_timer_deinit(dmx_timer_t timer) {
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_disable(timer->gptimer_handle);
  gptimer_del_timer(timer->gptimer_handle);
#else
  timer_isr_callback_remove(timer->timer_group, timer->timer_idx);
  timer_deinit(timer->timer_group, timer->timer_idx);
#endif
  timer->is_running = false;
}

void DMX_ISR_ATTR dmx_timer_stop(dmx_timer_t timer) {
  if (timer->is_running) {
#if ESP_IDF_VERSION_MAJOR >= 5
    gptimer_stop(timer->gptimer_handle);
#else
    timer_group_set_counter_enable_in_isr(timer->timer_group, timer->timer_idx,
                                          0);
#endif
    timer->is_running = false;
  }
}

void DMX_ISR_ATTR dmx_timer_set_counter(dmx_timer_t timer, uint64_t counter) {
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_set_raw_count(timer->gptimer_handle, counter);
#else
  timer_set_counter_value(timer->timer_group, timer->timer_idx, counter);
#endif
}

void DMX_ISR_ATTR dmx_timer_set_alarm(dmx_timer_t timer, uint64_t alarm) {
  // if (!timer->is_running) {
#if ESP_IDF_VERSION_MAJOR >= 5
    const gptimer_alarm_config_t alarm_config = {
        .alarm_count = alarm,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true};
    gptimer_set_alarm_action(timer->gptimer_handle, &alarm_config);
    gptimer_start(timer->gptimer_handle);
#else
    timer_set_alarm_value(timer->timer_group, timer->timer_idx, alarm);
    timer_start(timer->timer_group, timer->timer_idx);
#endif
    timer->is_running = true;
  // }
}