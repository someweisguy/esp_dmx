#include "dmx.h"

#include "driver.h"
#include "esp_log.h"
#include "hal/uart_ll.h"

#define DMX_ENTER_CRITICAL_ISR(mux) portENTER_CRITICAL_ISR(mux)
#define DMX_EXIT_CRITICAL_ISR(mux)  portEXIT_CRITICAL_ISR(mux)
#define DMX_ENTER_CRITICAL(mux)     portENTER_CRITICAL(mux)
#define DMXT_EXIT_CRITICAL(mux)     portEXIT_CRITICAL(mux)

static const char *TAG = "dmx";

esp_err_t dmx_driver_install(dmx_port_t dmx_num, int buffer_size,
    int queue_size, QueueHandle_t *dmx_queue, int intr_alloc_flags) {
  // TODO
  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {
  // TODO
  return ESP_OK;
}

bool dmx_is_driver_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && p_dmx_obj[dmx_num] != NULL;
}