#include "sniffer.h"

#include "driver/gpio.h"
#include "dmx/hal.h"
#include "esp_dmx.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_timer.h"
#else
#include "driver/timer.h"
#endif

static const char *TAG = "dmx_sniffer";  // The log tagline for the file.

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

static void DMX_ISR_ATTR dmx_gpio_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  int task_awoken = false;

  if (dmx_uart_get_rx_level(driver->uart)) {
    /* If this ISR is called on a positive edge and the current DMX frame is in
    a break and a negative edge timestamp has been recorded then a break has
    just finished. Therefore the DMX break length is able to be recorded. It can
    also be deduced that the driver is now in a DMX mark-after-break. */

    if ((driver->flags & DMX_FLAGS_DRIVER_IS_IN_BREAK) &&
        driver->sniffer.last_neg_edge_ts > -1) {
      driver->sniffer.data.break_len = now - driver->sniffer.last_neg_edge_ts;
      driver->flags |= DMX_FLAGS_DRIVER_IS_IN_BREAK;
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_MAB;
    }
    driver->sniffer.last_pos_edge_ts = now;
  } else {
    /* If this ISR is called on a negative edge in a DMX mark-after-break then
    the DMX mark-after-break has just finished. It can be recorded. Sniffer data
    is now available to be read by the user. */

    if (driver->flags & DMX_FLAGS_DRIVER_IS_IN_MAB) {
      driver->sniffer.data.mab_len = now - driver->sniffer.last_pos_edge_ts;
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_MAB;

      // Send the sniffer data to the queue
      xQueueOverwriteFromISR(driver->sniffer.queue, &driver->sniffer.data,
                             &task_awoken);
    }
    driver->sniffer.last_neg_edge_ts = now;
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

esp_err_t dmx_sniffer_enable(dmx_port_t dmx_num, int intr_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(intr_pin > 0 && GPIO_IS_VALID_GPIO(intr_pin), ESP_ERR_INVALID_ARG,
            "intr_pin error");
  DMX_CHECK(!dmx_sniffer_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "sniffer is already enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Allocate the sniffer queue
  driver->sniffer.queue = xQueueCreate(1, sizeof(dmx_metadata_t));
  if (driver->sniffer.queue == NULL) {
    ESP_LOGE(TAG, "DMX sniffer queue malloc error");
    return ESP_ERR_NO_MEM;
  }

  // Add the GPIO interrupt handler
  esp_err_t err = gpio_isr_handler_add(intr_pin, dmx_gpio_isr, driver);
  if (err) {
    ESP_LOGE(TAG, "DMX sniffer ISR handler error");
    vQueueDelete(driver->sniffer.queue);
    driver->sniffer.queue = NULL;
    return ESP_FAIL;
  }
  driver->sniffer.intr_pin = intr_pin;

  // Set sniffer default values
  driver->sniffer.last_neg_edge_ts = -1;  // Negative edge hasn't been seen yet
  driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_MAB;

  // Enable the interrupt
  gpio_set_intr_type(intr_pin, GPIO_INTR_ANYEDGE);

  return ESP_OK;
}

esp_err_t dmx_sniffer_disable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "sniffer is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Disable the interrupt and remove the interrupt handler
  gpio_set_intr_type(driver->sniffer.intr_pin, GPIO_INTR_DISABLE);
  esp_err_t err = gpio_isr_handler_remove(driver->sniffer.intr_pin);
  if (err) {
    ESP_LOGE(TAG, "DMX sniffer ISR handler error");
    return ESP_FAIL;
  }

  // Deallocate the sniffer queue
  vQueueDelete(driver->sniffer.queue);
  driver->sniffer.queue = NULL;

  return ESP_OK;
}

bool dmx_sniffer_is_enabled(dmx_port_t dmx_num) {
  return dmx_driver_is_installed(dmx_num) &&
         dmx_driver[dmx_num]->sniffer.queue != NULL;
}

bool dmx_sniffer_get_data(dmx_port_t dmx_num, dmx_metadata_t *metadata,
                          TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(metadata, ESP_ERR_INVALID_ARG, "metadata is null");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "sniffer is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  return xQueueReceive(driver->sniffer.queue, metadata, wait_ticks);
}