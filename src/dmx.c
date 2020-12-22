#include "dmx.h"

#include "driver.h"
#include "driver/periph_ctrl.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "hal/uart_ll.h"

#define DMX_ENTER_CRITICAL_ISR(mux) portENTER_CRITICAL_ISR(mux)
#define DMX_EXIT_CRITICAL_ISR(mux)  portEXIT_CRITICAL_ISR(mux)
#define DMX_ENTER_CRITICAL(mux)     portENTER_CRITICAL(mux)
#define DMX_EXIT_CRITICAL(mux)      portEXIT_CRITICAL(mux)

static const char *TAG = "dmx";
#define DMX_CHECK(a, str, ret_val)                            \
  if (!(a)) {                                                 \
    ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
    return (ret_val);                                         \
  }

esp_err_t dmx_driver_install(dmx_port_t dmx_num, int buffer_size,
    int queue_size, QueueHandle_t *dmx_queue, int intr_alloc_flags) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(buffer_size < 513, "buffer_size error", ESP_ERR_INVALID_ARG);
#if CONFIG_UART_ISR_IN_IRAM  // clang-format off
    if ((intr_alloc_flags & ESP_INTR_FLAG_IRAM) == 0) {
        ESP_LOGI(UART_TAG, "ESP_INTR_FLAG_IRAM flag not set while CONFIG_UART_ISR_IN_IRAM is enabled, flag updated");
        intr_alloc_flags |= ESP_INTR_FLAG_IRAM;
    }
#else
    if ((intr_alloc_flags & ESP_INTR_FLAG_IRAM) != 0) {
        ESP_LOGW(TAG, "ESP_INTR_FLAG_IRAM flag is set while CONFIG_UART_ISR_IN_IRAM is not enabled, flag updated");
        intr_alloc_flags &= ~ESP_INTR_FLAG_IRAM;
    }
#endif  // clang-format on

  if (p_dmx_obj[dmx_num] == NULL) {
    // allocate the dmx driver
    p_dmx_obj[dmx_num] = (dmx_obj_t *)heap_caps_calloc(
        1, sizeof(dmx_obj_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p_dmx_obj[dmx_num] == NULL) {
      ESP_LOGE(TAG, "DMX driver malloc error");
      return ESP_ERR_NO_MEM;
    }

    // initialize the driver to default values
    p_dmx_obj[dmx_num]->dmx_num = dmx_num;
    p_dmx_obj[dmx_num]->buffer_size = buffer_size;
    p_dmx_obj[dmx_num]->buffer = malloc(sizeof(uint8_t) * buffer_size);
    if (p_dmx_obj[dmx_num]->buffer == NULL) {
      ESP_LOGE(TAG, "DMX driver buffer malloc error");
      dmx_driver_delete(dmx_num);
      return ESP_ERR_NO_MEM;
    }
    if (dmx_queue) {
      p_dmx_obj[dmx_num]->queue = xQueueCreate(queue_size, sizeof(dmx_event_t));
      *dmx_queue = p_dmx_obj[dmx_num]->queue;
      ESP_LOGI(TAG, "queue free spaces: %d",
          uxQueueSpacesAvailable(p_dmx_obj[dmx_num]->queue));
    } else {
      p_dmx_obj[dmx_num]->queue = NULL;
    }

  } else {
    ESP_LOGE(TAG, "DMX driver already installed");
    return ESP_FAIL;
  }

  // enable uart peripheral module
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != true) {
    if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
      periph_module_reset(uart_periph_signal[dmx_num].module);
    }
    periph_module_enable(uart_periph_signal[dmx_num].module);
    dmx_context[dmx_num].hw_enabled = true;
  }
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // install interrupt
  uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), UART_INTR_MASK);
  uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_MASK);
  // TODO: replace NULL with interrupt handler
  esp_err_t err = dmx_isr_register(dmx_num, NULL, p_dmx_obj[dmx_num],
      intr_alloc_flags, &p_dmx_obj[dmx_num]->intr_handle);
  if (err) {
    dmx_driver_delete(dmx_num);
    return err;
  }
  const dmx_intr_config_t dmx_intr = {
      .intr_enable_mask = 0,  // interrupts off
      .rxfifo_full_thresh = 120,
      .rx_timeout_thresh = 10,
      .txfifo_empty_intr_thresh = 8,
  };
  err = dmx_intr_config(dmx_num, &dmx_intr);
  if (err) {
    dmx_driver_delete(dmx_num);
    return err;
  }
  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  if (p_dmx_obj[dmx_num] == NULL) {
    ESP_LOGI(TAG, "DMX driver already null");
    return ESP_OK;
  }

  // free driver resources
  if (p_dmx_obj[dmx_num]->buffer) free(p_dmx_obj[dmx_num]->buffer);
  if (p_dmx_obj[dmx_num]->queue) vQueueDelete(p_dmx_obj[dmx_num]->queue);

  // free driver
  heap_caps_free(p_dmx_obj[dmx_num]);
  p_dmx_obj[dmx_num] = NULL;

  // disable uart peripheral module
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != false) {
    if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
      periph_module_disable(uart_periph_signal[dmx_num].module);
    }
    dmx_context[dmx_num].hw_enabled = false;
  }
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

bool dmx_is_driver_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && p_dmx_obj[dmx_num] != NULL;
}

esp_err_t dmx_isr_register(dmx_port_t dmx_num, void (*fn)(void *), void *arg,
    int intr_alloc_flags, dmx_isr_handle_t *handle) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  esp_err_t ret;
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  ret = esp_intr_alloc(
      uart_periph_signal[dmx_num].irq, intr_alloc_flags, fn, arg, handle);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  return ret;
}

esp_err_t dmx_intr_config(
    dmx_port_t dmx_num, const dmx_intr_config_t *intr_conf) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(intr_conf, "intr_conf is null", ESP_ERR_INVALID_ARG);

  // TODO: determine which interrupts are necessary for DMX driver
  uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_MASK);
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (intr_conf->intr_enable_mask & UART_INTR_RXFIFO_TOUT) {
    uart_hal_set_rx_timeout(
        &(dmx_context[dmx_num].hal), intr_conf->rx_timeout_thresh);
  } else {
    // Disable rx_tout intr
    uart_hal_set_rx_timeout(&(dmx_context[dmx_num].hal), 0);
  }
  if (intr_conf->intr_enable_mask & UART_INTR_RXFIFO_FULL) {
    uart_hal_set_rxfifo_full_thr(
        &(dmx_context[dmx_num].hal), intr_conf->rxfifo_full_thresh);
  }
  if (intr_conf->intr_enable_mask & UART_INTR_TXFIFO_EMPTY) {
    uart_hal_set_txfifo_empty_thr(
        &(dmx_context[dmx_num].hal), intr_conf->txfifo_empty_intr_thresh);
  }
  uart_hal_ena_intr_mask(
      &(dmx_context[dmx_num].hal), intr_conf->intr_enable_mask);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  return ESP_OK;
}