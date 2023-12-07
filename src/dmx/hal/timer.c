#include "include/timer.h"

#include <stdbool.h>

#include "dmx/include/struct.h"
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
  dmx_uart_handle_t uart = driver->uart;
  dmx_timer_handle_t timer = driver->timer;
  int task_awoken = false;

  if (driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) {
    if (driver->flags & DMX_FLAGS_DRIVER_IS_IN_BREAK) {
      dmx_uart_invert_tx(uart, 0);
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;

      // Reset the alarm for the end of the DMX mark-after-break
      dmx_timer_set_alarm(timer, driver->mab_len, false);
    } else {
      // Write data to the UART
      size_t write_size = driver->tx_size;
      dmx_uart_write_txfifo(uart, driver->data, &write_size);
      driver->head += write_size;

      // Pause MAB timer alarm
      dmx_timer_stop(timer);

      // Enable DMX write interrupts
      dmx_uart_enable_interrupt(uart, DMX_INTR_TX_ALL);
    }
  } else if (driver->task_waiting) {
    // Notify the task
    xTaskNotifyFromISR(driver->task_waiting, DMX_OK, eSetValueWithOverwrite,
                       &task_awoken);  // TODO: return timeout?

    // Pause the receive timer alarm
    dmx_timer_stop(timer);
  }

  return task_awoken;
}

dmx_timer_handle_t dmx_timer_init(dmx_port_t dmx_num, void *isr_context,
                                  int isr_flags) {
  dmx_timer_handle_t timer = &dmx_timer_context[dmx_num];

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

void DMX_ISR_ATTR dmx_timer_set_alarm(dmx_timer_handle_t timer, uint64_t alarm,
                                      bool auto_reload) {
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