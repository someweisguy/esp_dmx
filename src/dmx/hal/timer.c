#include "include/timer.h"

#include <stdbool.h>

#include "dmx/hal/include/uart.h"
#include "dmx/include/service.h"
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

static bool DMX_ISR_ATTR dmx_timer_isr(
#if ESP_IDF_VERSION_MAJOR >= 5
    gptimer_handle_t gptimer_handle,
    const gptimer_alarm_event_data_t *event_data,
#endif
    void *arg) {
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  const dmx_port_t dmx_num = driver->dmx_num;
  int task_awoken = false;

  if (driver->dmx.status == DMX_STATUS_SENDING) {
    if (driver->dmx.progress == DMX_PROGRESS_IN_BREAK) {
      dmx_uart_invert_tx(dmx_num, 0);
      driver->dmx.progress = DMX_PROGRESS_IN_MAB;

      // Reset the alarm for the end of the DMX mark-after-break
      dmx_timer_set_alarm(dmx_num, driver->mab_len, false);
    } else {
      // Write data to the UART
      int write_len = driver->dmx.size;
      dmx_uart_write_txfifo(dmx_num, driver->dmx.data, &write_len);
      driver->dmx.head = write_len;

      // Pause MAB timer alarm
      dmx_timer_stop(dmx_num);  // TODO: is this needed?

      // Enable DMX write interrupts
      dmx_uart_enable_interrupt(dmx_num, DMX_INTR_TX_ALL);
    }
  } else {
    taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    if (driver->task_waiting) {
      xTaskNotifyFromISR(driver->task_waiting, DMX_OK, eSetValueWithOverwrite,
                         &task_awoken);
    }
    taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
    dmx_timer_stop(dmx_num);  // TODO: is this needed?
  }

  return task_awoken;
}

bool dmx_timer_init(dmx_port_t dmx_num, void *isr_context, int isr_flags) {
  struct dmx_timer_t *timer = &dmx_timer_context[dmx_num];

  // Initialize hardware timer
#if ESP_IDF_VERSION_MAJOR >= 5
  const gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000,  // 1MHz resolution timer
  };
  esp_err_t err = gptimer_new_timer(&timer_config, &timer->gptimer_handle);
  if (err) {
    return NULL;
  }
  const gptimer_event_callbacks_t gptimer_cb = {.on_alarm = dmx_timer_isr};
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
      .auto_reload = true,
  };
  esp_err_t err = timer_init(timer->group, timer->idx, &timer_config);
  if (err) {
    return NULL;
  }
  timer_isr_callback_add(timer->group, timer->idx, dmx_timer_isr, isr_context,
                         isr_flags);
#endif
  timer->is_running = false;

  return true;
}

void dmx_timer_deinit(dmx_port_t dmx_num) {
  struct dmx_timer_t *timer = &dmx_timer_context[dmx_num];
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_disable(timer->gptimer_handle);
  gptimer_del_timer(timer->gptimer_handle);
#else
  timer_isr_callback_remove(timer->group, timer->idx);
  timer_deinit(timer->group, timer->idx);
#endif
  timer->is_running = false;
}

void DMX_ISR_ATTR dmx_timer_stop(dmx_port_t dmx_num) {
  struct dmx_timer_t *timer = &dmx_timer_context[dmx_num];
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

void DMX_ISR_ATTR dmx_timer_set_counter(dmx_port_t dmx_num, uint64_t counter) {
  struct dmx_timer_t *timer = &dmx_timer_context[dmx_num];
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_set_raw_count(timer->gptimer_handle, counter);
#else
  timer_set_counter_value(timer->group, timer->idx, counter);
#endif
}

void DMX_ISR_ATTR dmx_timer_set_alarm(dmx_port_t dmx_num, uint64_t alarm,
                                      bool auto_reload) {
  struct dmx_timer_t *timer = &dmx_timer_context[dmx_num];
#if ESP_IDF_VERSION_MAJOR >= 5
  const gptimer_alarm_config_t alarm_config = {
      .alarm_count = alarm,
      .reload_count = 0,
      .flags.auto_reload_on_alarm = auto_reload};
  gptimer_set_alarm_action(timer->gptimer_handle, &alarm_config);
#else
  timer_group_set_alarm_value_in_isr(timer->group, timer->idx, alarm);
#endif
}

void DMX_ISR_ATTR dmx_timer_start(dmx_port_t dmx_num) {
  struct dmx_timer_t *timer = &dmx_timer_context[dmx_num];
  if (!timer->is_running) {
#if ESP_IDF_VERSION_MAJOR >= 5
    gptimer_start(timer->gptimer_handle);
#else
    timer_start(timer->group, timer->idx);
#endif
    timer->is_running = true;
  }
}

int64_t DMX_ISR_ATTR dmx_timer_get_micros_since_boot() {
  return esp_timer_get_time();
}