#include "driver/dmx.h"

#include <math.h>
#include <string.h>

#include "driver/dmx_ctrl.h"
#include "driver/dmx_default_intr_handler.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "hal/dmx_hal.h"
#include "hal/dmx_types.h"
#include "hal/uart_hal.h"
#include "hal/uart_ll.h"
#include "soc/io_mux_reg.h"
#include "soc/uart_caps.h"

#define DMX_EMPTY_THRESH_DEFAULT  8
#define DMX_FULL_THRESH_DEFAULT   120
#define DMX_TOUT_THRESH_DEFAULT   126
#define DMX_MIN_WAKEUP_THRESH     SOC_UART_MIN_WAKEUP_THRESH

#define DMX_ENTER_CRITICAL(mux)   portENTER_CRITICAL(mux)
#define DMX_EXIT_CRITICAL(mux)    portEXIT_CRITICAL(mux)

static const char *TAG = "dmx";
#define DMX_CHECK(a, str, ret_val)                            \
  if (!(a)) {                                                 \
    ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
    return (ret_val);                                         \
  }

/// Driver Functions  #########################################################
esp_err_t dmx_driver_install(dmx_port_t dmx_num, int buffer_size,
    int queue_size, QueueHandle_t *dmx_queue, int intr_alloc_flags) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(buffer_size > 0 && buffer_size <= DMX_MAX_PACKET_SIZE, "buffer_size error", ESP_ERR_INVALID_ARG);
#if CONFIG_UART_ISR_IN_IRAM
  if ((intr_alloc_flags & ESP_INTR_FLAG_IRAM) == 0) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set while CONFIG_UART_ISR_IN_IRAM is enabled, flag updated");
    intr_alloc_flags |= ESP_INTR_FLAG_IRAM;
  }
#else
  if ((intr_alloc_flags & ESP_INTR_FLAG_IRAM) != 0) {
    ESP_LOGW(TAG, "ESP_INTR_FLAG_IRAM flag is set while CONFIG_UART_ISR_IN_IRAM is not enabled, flag updated");
    intr_alloc_flags &= ~ESP_INTR_FLAG_IRAM;
  }
#endif

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
    if (dmx_queue) {
      p_dmx_obj[dmx_num]->queue = xQueueCreate(queue_size, sizeof(dmx_event_t));
      *dmx_queue = p_dmx_obj[dmx_num]->queue;
      ESP_LOGI(TAG, "queue free spaces: %d", uxQueueSpacesAvailable(p_dmx_obj[dmx_num]->queue));
    } else {
      p_dmx_obj[dmx_num]->queue = NULL;
    }
    p_dmx_obj[dmx_num]->buf_size = buffer_size;
    for (int i = 0; i < 2; ++i) {
      p_dmx_obj[dmx_num]->buffer[i] = calloc(sizeof(uint8_t), buffer_size);
      if (p_dmx_obj[dmx_num]->buffer[i] == NULL) {
        ESP_LOGE(TAG, "DMX driver buffer malloc error");
        dmx_driver_delete(dmx_num);
        return ESP_ERR_NO_MEM;
      }
    }
    p_dmx_obj[dmx_num]->slot_idx = -1;
    p_dmx_obj[dmx_num]->buf_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_RX;

    p_dmx_obj[dmx_num]->rx_last_brk_ts = INT64_MIN;

    p_dmx_obj[dmx_num]->tx_last_brk_ts = INT64_MIN;
    p_dmx_obj[dmx_num]->tx_done_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(p_dmx_obj[dmx_num]->tx_done_sem);

    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    uart_hal_set_rts(&(dmx_context[dmx_num].hal), 1); // set rts low
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  } else {
    ESP_LOGE(TAG, "DMX driver already installed");
    return ESP_ERR_INVALID_STATE;
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
  esp_err_t err = esp_intr_alloc(uart_periph_signal[dmx_num].irq,
      intr_alloc_flags, &dmx_default_intr_handler, p_dmx_obj[dmx_num],
      &p_dmx_obj[dmx_num]->intr_handle);
  if (err) {
    dmx_driver_delete(dmx_num);
    return err;
  }
  const dmx_intr_config_t dmx_intr = {
      .intr_enable_mask = DMX_INTR_RX_ALL, // enable rx
      .rxfifo_full_thresh = DMX_FULL_THRESH_DEFAULT,
      .rx_timeout_thresh = DMX_TOUT_THRESH_DEFAULT,
      .txfifo_empty_intr_thresh = DMX_EMPTY_THRESH_DEFAULT,
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

  // free isr
  esp_err_t err = esp_intr_free(p_dmx_obj[dmx_num]->intr_handle);
  if (err) return err;

  // free driver resources
  for (int i = 0; i < 2; ++i) {
    if (p_dmx_obj[dmx_num]->buffer[i]) free(p_dmx_obj[dmx_num]->buffer[i]);
  }
  if (p_dmx_obj[dmx_num]->queue) vQueueDelete(p_dmx_obj[dmx_num]->queue);
  if (p_dmx_obj[dmx_num]->tx_done_sem) vSemaphoreDelete(p_dmx_obj[dmx_num]->tx_done_sem);

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

esp_err_t dmx_set_mode(dmx_port_t dmx_num, dmx_mode_t dmx_mode) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(dmx_mode < DMX_MODE_MAX, "dmx_mode error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);

  // if the driver is in the requested mode, do nothing
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  const dmx_mode_t current_dmx_mode = p_dmx_obj[dmx_num]->mode;
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (current_dmx_mode == dmx_mode)
    return ESP_OK;

  if (dmx_mode == DMX_MODE_RX) {
    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_TX_ALL);
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_MASK);

    p_dmx_obj[dmx_num]->slot_idx = (uint16_t)-1;
    p_dmx_obj[dmx_num]->buf_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_RX;
    uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));

    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    uart_hal_set_rts(&(dmx_context[dmx_num].hal), 1); // set rts low
    uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_RX_ALL);
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    
  } else { // dmx_mode == DMX_MODE_TX
    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    uart_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_RX_ALL);
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_MASK);

    p_dmx_obj[dmx_num]->slot_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_TX;
    xSemaphoreGive(p_dmx_obj[dmx_num]->tx_done_sem);
    uart_hal_txfifo_rst(&(dmx_context[dmx_num].hal));

    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    uart_hal_set_rts(&(dmx_context[dmx_num].hal), 0); // set rts high
    // tx interrupts are enabled when calling the tx function!!
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  }
  return ESP_OK;
}

/// Hardware Configuration  ###################################################
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_io_num, int rx_io_num,
    int rts_io_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK((tx_io_num < 0 || (GPIO_IS_VALID_OUTPUT_GPIO(tx_io_num))), "tx_io_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK((rx_io_num < 0 || (GPIO_IS_VALID_GPIO(rx_io_num))), "rx_io_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK((rts_io_num < 0 || (GPIO_IS_VALID_OUTPUT_GPIO(rts_io_num))), "rts_io_num error", ESP_ERR_INVALID_ARG);

  // assign hardware pinouts
  if (tx_io_num >= 0) {
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[tx_io_num], PIN_FUNC_GPIO);
    gpio_set_level(tx_io_num, 1);
    gpio_matrix_out(tx_io_num, uart_periph_signal[dmx_num].tx_sig, 0, 0);
  }
  if (rx_io_num >= 0) {
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[rx_io_num], PIN_FUNC_GPIO);
    gpio_set_pull_mode(rx_io_num, GPIO_PULLUP_ONLY);
    gpio_set_direction(rx_io_num, GPIO_MODE_INPUT);
    gpio_matrix_in(rx_io_num, uart_periph_signal[dmx_num].rx_sig, 0);
  }
  if (rts_io_num >= 0) {
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[rts_io_num], PIN_FUNC_GPIO);
    gpio_set_direction(rts_io_num, GPIO_MODE_OUTPUT);
    gpio_matrix_out(rts_io_num, uart_periph_signal[dmx_num].rts_sig, 0, 0);
  }

  return ESP_OK;
}

esp_err_t dmx_param_config(dmx_port_t dmx_num, const dmx_config_t *dmx_config) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(dmx_config, "dmx_config is null", ESP_ERR_INVALID_ARG);
  DMX_CHECK(dmx_config->baudrate >= DMX_MIN_BAUDRATE && dmx_config->baudrate <= DMX_MAX_BAUDRATE, "baudrate error", ESP_ERR_INVALID_ARG);
  const float bit_speed = 1000000.0 / dmx_config->baudrate;
  DMX_CHECK(dmx_config->break_num * bit_speed >= 92 && dmx_config->break_num < 1024, "break_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(dmx_config->idle_num * bit_speed >= 12 && dmx_config->idle_num * bit_speed < 1000000, "idle_num error", ESP_ERR_INVALID_ARG);

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

  // configure the uart hardware
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  uart_hal_init(&(dmx_context[dmx_num].hal), dmx_num);
  uart_hal_set_baudrate(&(dmx_context[dmx_num].hal), dmx_config->source_clk, dmx_config->baudrate);
  uart_hal_set_parity(&(dmx_context[dmx_num].hal), UART_PARITY_DISABLE);
  uart_hal_set_data_bit_num(&(dmx_context[dmx_num].hal), UART_DATA_8_BITS);
  uart_hal_set_stop_bits(&(dmx_context[dmx_num].hal), UART_STOP_BITS_2);
  uart_hal_set_tx_idle_num(&(dmx_context[dmx_num].hal), dmx_config->idle_num);
  uart_hal_set_hw_flow_ctrl(&(dmx_context[dmx_num].hal), UART_HW_FLOWCTRL_DISABLE, 0);
  uart_hal_tx_break(&(dmx_context[dmx_num].hal), dmx_config->break_num);
  uart_hal_set_mode(&(dmx_context[dmx_num].hal), UART_MODE_RS485_COLLISION_DETECT);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  
  // flush both fifos
  uart_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
  uart_hal_txfifo_rst(&(dmx_context[dmx_num].hal));

  return ESP_OK;
}

esp_err_t dmx_set_baudrate(dmx_port_t dmx_num, uint32_t baudrate) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  // TODO: error check
  
  uart_sclk_t source_clk;
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  uart_hal_get_sclk(&(dmx_context[dmx_num].hal), &source_clk);
  uart_hal_set_baudrate(&(dmx_context[dmx_num].hal), source_clk, baudrate);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_baudrate(dmx_port_t dmx_num, uint32_t *baudrate) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  uart_hal_get_baudrate(&(dmx_context[dmx_num].hal), baudrate);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_break_num(dmx_port_t dmx_num, uint8_t break_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  // TODO: error check

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  uart_hal_tx_break(&(dmx_context[dmx_num].hal), break_num);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_break_num(dmx_port_t dmx_num, uint8_t *break_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *break_num = dmx_hal_get_break_num(&(dmx_context[dmx_num].hal));
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_idle_num(dmx_port_t dmx_num, uint16_t idle_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(idle_num < 1024, "idle_num error", ESP_ERR_INVALID_ARG);
  // TODO: error check

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  uart_hal_set_tx_idle_num(&(dmx_context[dmx_num].hal), idle_num);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_idle_num(dmx_port_t dmx_num, uint16_t *idle_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *idle_num = dmx_hal_get_idle_num(&(dmx_context[dmx_num].hal));
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_invert_rts(dmx_port_t dmx_num, bool invert) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_inverse_rts_signal(&(dmx_context[dmx_num].hal), invert);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

/// Interrupt Configuration  ##################################################
esp_err_t dmx_intr_config(dmx_port_t dmx_num, const dmx_intr_config_t *intr_conf) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(intr_conf, "intr_conf is null", ESP_ERR_INVALID_ARG);

  uart_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), UART_INTR_MASK);
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (intr_conf->intr_enable_mask & UART_INTR_RXFIFO_TOUT) {
    uart_hal_set_rx_timeout(&(dmx_context[dmx_num].hal), intr_conf->rx_timeout_thresh);
  } else {
    uart_hal_set_rx_timeout(&(dmx_context[dmx_num].hal), 0); // disable rx tout intr
  }
  if (intr_conf->intr_enable_mask & UART_INTR_RXFIFO_FULL) {
    uart_hal_set_rxfifo_full_thr(&(dmx_context[dmx_num].hal), intr_conf->rxfifo_full_thresh);
  }
  if (intr_conf->intr_enable_mask & UART_INTR_TXFIFO_EMPTY) {
    uart_hal_set_txfifo_empty_thr(&(dmx_context[dmx_num].hal), intr_conf->txfifo_empty_intr_thresh);
  }
  uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), intr_conf->intr_enable_mask);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_rx_full_threshold(dmx_port_t dmx_num, int threshold) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(threshold < UART_RXFIFO_FULL_THRHD_V && threshold > 0, "rx fifo full threshold value error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (uart_hal_get_intr_ena_status(&(dmx_context[dmx_num].hal)) & UART_INTR_RXFIFO_FULL)
    uart_hal_set_rxfifo_full_thr(&(dmx_context[dmx_num].hal), threshold);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}
esp_err_t dmx_set_tx_empty_threshold(dmx_port_t dmx_num, int threshold) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(threshold < UART_TXFIFO_EMPTY_THRHD_V && threshold > 0, "tx fifo empty threshold value error", ESP_ERR_INVALID_ARG);
  
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (uart_hal_get_intr_ena_status(&(dmx_context[dmx_num].hal)) & UART_INTR_TXFIFO_EMPTY)
    uart_hal_set_txfifo_empty_thr(&(dmx_context[dmx_num].hal), threshold);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_rx_timeout(dmx_port_t dmx_num, uint8_t tout_thresh) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(tout_thresh < 127, "tout_thresh max value is 126", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  uart_hal_set_rx_timeout(&(dmx_context[dmx_num].hal), tout_thresh);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

/// Read/Write  ###############################################################
esp_err_t dmx_wait_tx_done(dmx_port_t dmx_num, TickType_t ticks_to_wait) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);

  /* Just try to take the "done" semaphore and give it back immediately. */

  if (xSemaphoreTake(p_dmx_obj[dmx_num]->tx_done_sem, ticks_to_wait) == pdFALSE)
    return ESP_ERR_TIMEOUT;
  xSemaphoreGive(p_dmx_obj[dmx_num]->tx_done_sem);

  return ESP_OK;
}

esp_err_t dmx_tx_frame(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(p_dmx_obj[dmx_num]->mode == DMX_MODE_TX, "not in tx mode", ESP_ERR_INVALID_STATE);

  // only tx when a frame is not being written
  if (xSemaphoreTake(p_dmx_obj[dmx_num]->tx_done_sem, 0) == pdFALSE)
    return ESP_FAIL;
  
  /* The ESP32 uart hardware isn't the ideal hardware to transmit DMX. The DMX 
  protocol states that frames begin with a break, followed by a mark, followed 
  by up to 513 bytes. The ESP32 uart hardware is designed to send a packet, 
  followed by a break, followed by a mark. When using this library "correctly,"
  there shouldn't be any issues because the data stream will be continuous - 
  even though the hardware sends the break and mark after the packet, it will
  LOOK like it is being sent before the packet. However if the byte stream isn't
  continuous, we need to send a break and mark before we send the packet. This 
  is done by inverting the line, busy waiting, un-inverting the line and 
  busy waiting again. The busy waiting isn't very accurate (it's usually 
  accurate within around 10us if the task isn't preempted), but it is the best 
  that can be done short of using the ESP32 hardware timer library. */

  // check if we need to send a new break and mark after break
  const int64_t now = esp_timer_get_time();
  if (now - p_dmx_obj[dmx_num]->tx_last_brk_ts >= 1000000) {
    // get break and mark time in microseconds
    uint32_t baudrate, brk_num, idle_num;
    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    uart_hal_get_baudrate(&(dmx_context[dmx_num].hal), &baudrate);
    const float bit_speed = 1000000.0 / baudrate;
    brk_num = ceil(bit_speed * dmx_hal_get_break_num(&(dmx_context[dmx_num].hal)));
    idle_num = ceil(bit_speed * dmx_hal_get_idle_num(&(dmx_context[dmx_num].hal)));
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

    // invert the tx line and busy wait...
    dmx_hal_inverse_txd_signal(&(dmx_context[dmx_num].hal), UART_SIGNAL_TXD_INV);
    ets_delay_us(brk_num);

    // un-invert the tx line and busy wait...
    dmx_hal_inverse_txd_signal(&(dmx_context[dmx_num].hal), 0);
    ets_delay_us(idle_num);

    p_dmx_obj[dmx_num]->tx_last_brk_ts = now;
  }

  // write data to tx FIFO
  uint32_t bytes_written;
  dmx_obj_t *const p_dmx = p_dmx_obj[dmx_num];
  const uint32_t len = p_dmx->buf_size - p_dmx->slot_idx;
  const uint8_t *offset = p_dmx->buffer[p_dmx->buf_idx] + p_dmx->slot_idx;
  uart_hal_write_txfifo(&(dmx_context[dmx_num].hal), offset, len, 
    &bytes_written);
  p_dmx->slot_idx = bytes_written;

  // enable tx interrupts
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  uart_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_TX_ALL);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_write_frame(dmx_port_t dmx_num, const uint8_t *frame_buffer, uint16_t length) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(frame_buffer, "frame_buffer is null", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(length <= p_dmx_obj[dmx_num]->buf_size, "length error", ESP_ERR_INVALID_ARG);

  /* Writes can only happen in DMX_MODE_TX. Writes are made to buffer 0, whilst
  buffer 1 is used by the driver to write to the tx FIFO. */

  if (p_dmx_obj[dmx_num]->mode != DMX_MODE_TX) {
    ESP_LOGE(TAG, "cannot write if not in tx mode");
    return ESP_ERR_INVALID_STATE;
  }

  memcpy(p_dmx_obj[dmx_num]->buffer[0], frame_buffer, length);

  return ESP_OK;
}

esp_err_t dmx_read_frame(dmx_port_t dmx_num, uint8_t *frame_buffer, uint16_t length) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(frame_buffer, "frame_buffer is null", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(length <= p_dmx_obj[dmx_num]->buf_size, "length error", ESP_ERR_INVALID_ARG);

  /* Reads can happen in either DMX_MODE_RX or DMX_MODE_TX. Reads while in 
  DMX_MODE_RX are made from the inactive buffer while the active buffer is 
  being used to collect data from the rx FIFO. Reads in DMX_MODE_TX are made 
  from buffer 0 whilst buffer 1 is used by the driver to write to the tx 
  FIFO. */

  if (p_dmx_obj[dmx_num]->mode == DMX_MODE_RX) {
    uint8_t active_buffer;
    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    active_buffer = p_dmx_obj[dmx_num]->buf_idx;
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    memcpy(frame_buffer, p_dmx_obj[dmx_num]->buffer[!active_buffer], length);
  } else { // mode == DMX_MODE_TX
    memcpy(frame_buffer, p_dmx_obj[dmx_num]->buffer[0], length);
  }

  return ESP_OK;
}