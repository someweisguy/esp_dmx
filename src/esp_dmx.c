#include "esp_dmx.h"

#include <math.h>
#include <string.h>

#include "dmx_types.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "impl/dmx_hal.h"
#include "impl/driver.h"
#include "impl/intr_handlers.h"
#include "soc/io_mux_reg.h"
#include "soc/uart_periph.h"
#include "soc/uart_reg.h"

// The default value for the RX FIFO full interrupt threshold.
#define DMX_UART_FULL_DEFAULT (1)
// The default value for the TX FIFO empty interrupt threshold.
#define DMX_UART_EMPTY_DEFAULT (8)
// The default value for the UART timeout interrupt.
#define DMX_UART_TIMEOUT_DEFAULT (45)

#define DMX_FUNCTION_NOT_SUPPORTED()                         \
  ESP_LOGE(TAG, "%s() is not supported on %s", __FUNCTION__, \
           CONFIG_IDF_TARGET);                               \
  return ESP_ERR_NOT_SUPPORTED;

static const char *TAG = "dmx";  // The log tagline for the file.

static void dmx_module_enable(dmx_port_t dmx_num) {
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != true) {
    periph_module_enable(uart_periph_signal[dmx_num].module);
    if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
      /* Workaround for ESP32C3: enable core reset before enabling UART module
      clock to prevent UART output garbage value. */
#if SOC_UART_REQUIRE_CORE_RESET
      uart_hal_set_reset_core(&(dmx_context[dmx_num].hal), true);
      periph_module_reset(uart_periph_signal[dmx_num].module);
      uart_hal_set_reset_core(&(dmx_context[dmx_num].hal), false);
#else
      periph_module_reset(uart_periph_signal[dmx_num].module);
#endif
    }
    dmx_context[dmx_num].hw_enabled = true;
  }
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
}

static void dmx_module_disable(dmx_port_t dmx_num) {
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != false) {
    if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
      periph_module_disable(uart_periph_signal[dmx_num].module);
    }
    dmx_context[dmx_num].hw_enabled = false;
  }
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
}

/// Driver Functions  #########################################################
esp_err_t dmx_driver_install(dmx_port_t dmx_num, dmx_config_t *dmx_config,
                             uint32_t queue_size, QueueHandle_t *dmx_queue) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] == NULL, ESP_ERR_INVALID_STATE, TAG,
                      "DMX driver is already installed");
  ESP_RETURN_ON_FALSE(dmx_config != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "dmx_config is null");
  ESP_RETURN_ON_FALSE(dmx_config->buffer_size <= DMX_MAX_PACKET_SIZE,
                      ESP_ERR_INVALID_ARG, TAG, "buffer_size error");
  ESP_RETURN_ON_FALSE(dmx_config->rst_seq_hw == DMX_USE_BUSY_WAIT ||
                          dmx_config->rst_seq_hw < DMX_RESET_SEQUENCE_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "rst_seq_hw error");
  ESP_RETURN_ON_FALSE(dmx_config->timer_idx < TIMER_MAX, ESP_ERR_INVALID_ARG,
                      TAG, "timer_idx error");

  /* Driver ISR is in IRAM so intr_alloc_flags must include the
  ESP_INTR_FLAG_IRAM flag. */
  if ((dmx_config->intr_alloc_flags & ESP_INTR_FLAG_IRAM) == 0) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    dmx_config->intr_alloc_flags |= ESP_INTR_FLAG_IRAM;
  }

  // configure the uart hardware
  dmx_module_enable(dmx_num);
  dmx_context_t *const hardware = &dmx_context[dmx_num];
  portENTER_CRITICAL(&hardware->spinlock);
  dmx_hal_init(&hardware->hal);
  dmx_hal_set_sclk(&hardware->hal, UART_SCLK_APB);
  dmx_hal_set_baudrate(&hardware->hal, DMX_BAUD_RATE);
  portEXIT_CRITICAL(&hardware->spinlock);

  // flush both fifos
  dmx_hal_rxfifo_rst(&hardware->hal);
  dmx_hal_txfifo_rst(&hardware->hal);

  // allocate the dmx driver
  const uint32_t mem_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  dmx_driver[dmx_num] = heap_caps_malloc(sizeof(dmx_driver_t), mem_caps);
  if (dmx_driver[dmx_num] == NULL) {
    ESP_LOGE(TAG, "DMX driver malloc error");
    return ESP_ERR_NO_MEM;
  }
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // allocate driver buffer
  driver->buffer = heap_caps_malloc(dmx_config->buffer_size, mem_caps);
  if (driver->buffer == NULL) {
    ESP_LOGE(TAG, "DMX driver buffer malloc error");
    dmx_driver_delete(dmx_num);
    return ESP_ERR_NO_MEM;
  }
  bzero(driver->buffer, dmx_config->buffer_size);

  // allocate driver rx queue
  if (dmx_queue != NULL) {
    driver->rx.queue = xQueueCreate(queue_size, sizeof(dmx_event_t));
    if (driver->rx.queue == NULL) {
      ESP_LOGE(TAG, "DMX driver queue malloc error");
      dmx_driver_delete(dmx_num);
      return ESP_ERR_NO_MEM;
    }
    *dmx_queue = driver->rx.queue;
  } else {
    driver->rx.queue = NULL;
  }

  // allocate semaphores
  driver->tx.sent_sem = xSemaphoreCreateBinaryStatic(&driver->tx.sent_sem_buf);
  xSemaphoreGive(driver->tx.sent_sem);
  driver->tx.turn_sem = xSemaphoreCreateBinaryStatic(&driver->tx.turn_sem_buf);
  xSemaphoreGive(driver->tx.turn_sem);

  // initialize general driver variables
  driver->dmx_num = dmx_num;
  driver->buf_size = dmx_config->buffer_size;
  driver->mode = DMX_MODE_READ;
  driver->slot_idx = -1;  // driver starts in error state
  driver->rst_seq_hw = dmx_config->rst_seq_hw;
  driver->timer_idx = dmx_config->timer_idx;

  driver->awaiting_response = false;
  driver->awaiting_turnaround = false;

  // initialize driver tx variables
  driver->tx.break_len = DMX_BREAK_LEN_US;
  driver->tx.mab_len = DMX_WRITE_MIN_MAB_LEN_US;

  // initialize driver rx variables
  driver->rx.event_sent = true;  // don't send event until first break rx'd
  driver->rx.intr_io_num = -1;
  driver->rx.break_len = -1;
  driver->rx.mab_len = -1;

  // install uart interrupt
  portENTER_CRITICAL(&hardware->spinlock);
  dmx_hal_disable_intr_mask(&hardware->hal, DMX_ALL_INTR_MASK);
  portEXIT_CRITICAL(&hardware->spinlock);
  dmx_hal_clr_intsts_mask(&hardware->hal, DMX_ALL_INTR_MASK);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, dmx_config->intr_alloc_flags,
                 &dmx_intr_handler, driver, &driver->uart_isr_handle);
  const dmx_intr_config_t dmx_intr_conf = {
      .rxfifo_full_threshold = DMX_UART_FULL_DEFAULT,
      .rx_timeout_threshold = DMX_UART_TIMEOUT_DEFAULT,
      .txfifo_empty_threshold = DMX_UART_EMPTY_DEFAULT,
  };
  dmx_intr_config(dmx_num, &dmx_intr_conf);

  // enable rx interrupt and set rts
  portENTER_CRITICAL(&hardware->spinlock);
  dmx_hal_ena_intr_mask(&hardware->hal, DMX_INTR_RX_ALL);
  dmx_hal_set_rts(&hardware->hal, 1);  // set rts low
  portEXIT_CRITICAL(&hardware->spinlock);

  // install timer interrupt
  if (driver->rst_seq_hw != DMX_USE_BUSY_WAIT) {
    const timer_config_t timer_conf = {
        .divider = 80,  // 80MHz / 80 == 1MHz resolution timer
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = false,
        .alarm_en = true,
        .auto_reload = true,
    };
    timer_init(dmx_config->rst_seq_hw, dmx_config->timer_idx, &timer_conf);
    timer_set_counter_value(dmx_config->rst_seq_hw, dmx_config->timer_idx, 0);
    timer_set_alarm_value(dmx_config->rst_seq_hw, dmx_config->timer_idx, 0);
    timer_isr_callback_add(dmx_config->rst_seq_hw, dmx_config->timer_idx,
                           dmx_timer_intr_handler, driver,
                           dmx_config->intr_alloc_flags);
    timer_enable_intr(dmx_config->rst_seq_hw, dmx_config->timer_idx);
  }

  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "DMX driver not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // free uart interrupt
  esp_err_t err = esp_intr_free(driver->uart_isr_handle);
  if (err) return err;

  // deinit timer and free timer isr
  if (driver->rst_seq_hw != DMX_USE_BUSY_WAIT) {
    timer_deinit(driver->rst_seq_hw, driver->timer_idx);
  }

  // free sniffer isr
  if (driver->rx.intr_io_num != -1) dmx_sniffer_disable(dmx_num);

  // free driver resources
  if (driver->buffer[0]) free(driver->buffer);
  if (driver->rx.queue) vQueueDelete(dmx_driver[dmx_num]->rx.queue);
  if (driver->tx.sent_sem) vSemaphoreDelete(driver->tx.sent_sem);

  // free driver and disable module
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;
  dmx_module_disable(dmx_num);

  return ESP_OK;
}

bool dmx_is_driver_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && dmx_driver[dmx_num] != NULL;
}

esp_err_t dmx_set_mode(dmx_port_t dmx_num, dmx_mode_t dmx_mode) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_mode >= 0 && dmx_mode < DMX_MODE_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_mode error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");

  // if the driver is in the requested mode, do nothing
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  const dmx_mode_t current_dmx_mode = dmx_driver[dmx_num]->mode;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (current_dmx_mode == dmx_mode) return ESP_OK;

  if (dmx_mode == DMX_MODE_READ) {
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_TX_ALL);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), DMX_ALL_INTR_MASK);

    dmx_driver[dmx_num]->slot_idx = -1;
    dmx_driver[dmx_num]->mode = DMX_MODE_READ;
    dmx_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));

    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_set_rts(&(dmx_context[dmx_num].hal), 1);  // set rts low
    dmx_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_RX_ALL);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  } else {  // dmx_mode == DMX_MODE_WRITE
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_RX_ALL);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), DMX_ALL_INTR_MASK);

    // disable sniffer if it is enabled
    if (dmx_driver[dmx_num]->rx.intr_io_num != -1) dmx_sniffer_disable(dmx_num);

    dmx_driver[dmx_num]->mode = DMX_MODE_WRITE;
    dmx_hal_txfifo_rst(&(dmx_context[dmx_num].hal));
    bzero(dmx_driver[dmx_num]->buffer, dmx_driver[dmx_num]->buf_size);

    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_set_rts(&(dmx_context[dmx_num].hal), 0);  // set rts high
    // tx interrupts are enabled when calling the tx function!!
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  }

  return ESP_OK;
}

esp_err_t dmx_get_mode(dmx_port_t dmx_num, dmx_mode_t *dmx_mode) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_mode != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "dmx_mode must not be null");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *dmx_mode = dmx_driver[dmx_num]->mode;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_sniffer_enable(dmx_port_t dmx_num, int intr_io_num) {
#ifdef DMX_GET_RX_LEVEL_NOT_SUPPORTED
  DMX_FUNCTION_NOT_SUPPORTED();
#endif
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(intr_io_num > -1 && GPIO_IS_VALID_GPIO(intr_io_num),
                      ESP_ERR_INVALID_ARG, TAG, "intr_io_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->mode == DMX_MODE_READ,
                      ESP_ERR_INVALID_STATE, TAG, "must be in read mode");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->rx.queue != NULL,
                      ESP_ERR_INVALID_STATE, TAG, "queue is null");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->rx.intr_io_num == -1,
                      ESP_ERR_INVALID_STATE, TAG, "sniffer already enabled");

  // add the isr handler
  esp_err_t err = gpio_isr_handler_add(intr_io_num, dmx_timing_intr_handler,
                                       dmx_driver[dmx_num]);
  if (err) return err;

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_driver[dmx_num]->rx.intr_io_num = intr_io_num;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // set to known values to allow for graceful startup
  dmx_driver[dmx_num]->rx.is_in_brk = false;
  dmx_driver[dmx_num]->rx.last_neg_edge_ts = -1;

  // enable interrupt
  gpio_set_intr_type(intr_io_num, GPIO_INTR_ANYEDGE);

  return ESP_OK;
}

esp_err_t dmx_sniffer_disable(dmx_port_t dmx_num) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->rx.intr_io_num != -1,
                      ESP_ERR_INVALID_STATE, TAG, "sniffer not enabled");

  gpio_num_t intr_io_num;
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  intr_io_num = dmx_driver[dmx_num]->rx.intr_io_num;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // disable the interrupt and remove the isr handler
  gpio_set_intr_type(intr_io_num, GPIO_INTR_DISABLE);
  esp_err_t err = gpio_isr_handler_remove(intr_io_num);
  if (err) return err;

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_driver[dmx_num]->rx.intr_io_num = -1;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

bool dmx_is_sniffer_enabled(dmx_port_t dmx_num) {
  return dmx_is_driver_installed(dmx_num) &&
         dmx_driver[dmx_num]->rx.intr_io_num != -1;
}

/// Hardware Configuration  ###################################################
esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_io_num, int rx_io_num,
                      int rts_io_num) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(tx_io_num < 0 || GPIO_IS_VALID_OUTPUT_GPIO(tx_io_num),
                      ESP_ERR_INVALID_ARG, TAG, "tx_io_num error");
  ESP_RETURN_ON_FALSE(rx_io_num < 0 || GPIO_IS_VALID_GPIO(rx_io_num),
                      ESP_ERR_INVALID_ARG, TAG, "rx_io_num error");
  ESP_RETURN_ON_FALSE(rts_io_num < 0 || GPIO_IS_VALID_OUTPUT_GPIO(rts_io_num),
                      ESP_ERR_INVALID_ARG, TAG, "rts_io_num error");

  return uart_set_pin(dmx_num, tx_io_num, rx_io_num, rts_io_num,
                      DMX_PIN_NO_CHANGE);
}

esp_err_t dmx_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(DMX_BAUD_RATE_IS_VALID(baud_rate), ESP_ERR_INVALID_ARG,
                      TAG, "baud_rate error");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_baudrate(&(dmx_context[dmx_num].hal), baud_rate);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_baud_rate(dmx_port_t dmx_num, uint32_t *baud_rate) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(baud_rate != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "baud_rate is null");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *baud_rate = dmx_hal_get_baudrate(&(dmx_context[dmx_num].hal));
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_break_len(dmx_port_t dmx_num, uint32_t break_len) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  // FIXME
  //ESP_RETURN_ON_FALSE(DMX_TX_BRK_DURATION_IS_VALID(break_len),
  //                    ESP_ERR_INVALID_ARG, TAG, "break_len error");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_driver[dmx_num]->tx.break_len = break_len;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_break_len(dmx_port_t dmx_num, uint32_t *break_len) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(break_len != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "break_len is null");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *break_len = dmx_driver[dmx_num]->tx.break_len;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_mab_len(dmx_port_t dmx_num, uint32_t mab_len) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  // FIXME
  // ESP_RETURN_ON_FALSE(DMX_TX_MAB_DURATION_IS_VALID(mab_len),
                      // ESP_ERR_INVALID_ARG, TAG, "mab_len error");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_driver[dmx_num]->tx.mab_len = mab_len;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_mab_len(dmx_port_t dmx_num, uint32_t *mab_len) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(mab_len != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "mab_len is null");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *mab_len = dmx_driver[dmx_num]->tx.mab_len;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

/// Interrupt Configuration  ##################################################
esp_err_t dmx_intr_config(dmx_port_t dmx_num,
                          const dmx_intr_config_t *intr_conf) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(intr_conf != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "intr_config is null");

  dmx_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), DMX_ALL_INTR_MASK);
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_rx_timeout(&(dmx_context[dmx_num].hal),
                         intr_conf->rx_timeout_threshold);
  dmx_hal_set_rxfifo_full_thr(&(dmx_context[dmx_num].hal),
                              intr_conf->rxfifo_full_threshold);
  dmx_hal_set_txfifo_empty_thr(&(dmx_context[dmx_num].hal),
                               intr_conf->txfifo_empty_threshold);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_rx_full_threshold(dmx_port_t dmx_num, int threshold) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(threshold > 0 && threshold < UART_RXFIFO_FULL_THRHD,
                      ESP_ERR_INVALID_ARG, TAG, "threshold value error");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_hal_get_intr_ena_status(&(dmx_context[dmx_num].hal)) &
      UART_INTR_RXFIFO_FULL) {
    dmx_hal_set_rxfifo_full_thr(&(dmx_context[dmx_num].hal), threshold);
  }
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}
esp_err_t dmx_set_tx_empty_threshold(dmx_port_t dmx_num, int threshold) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(threshold > 0 && threshold < UART_TXFIFO_EMPTY_THRHD,
                      ESP_ERR_INVALID_ARG, TAG,
                      "tx fifo empty threshold value error");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_hal_get_intr_ena_status(&(dmx_context[dmx_num].hal)) &
      UART_INTR_TXFIFO_EMPTY) {
    dmx_hal_set_txfifo_empty_thr(&(dmx_context[dmx_num].hal), threshold);
  }
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_rx_timeout(dmx_port_t dmx_num, uint8_t timeout) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(timeout < 127, ESP_ERR_INVALID_ARG, TAG, "timeout error");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_rx_timeout(&(dmx_context[dmx_num].hal), timeout);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

/// Read/Write  ###############################################################
esp_err_t dmx_read_packet(dmx_port_t dmx_num, void *buffer, uint16_t size) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "buffer is null");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->buf_size >= size,
                      ESP_ERR_INVALID_ARG, TAG, "size error");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  memcpy(buffer, dmx_driver[dmx_num]->buffer, size);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_read_slot(dmx_port_t dmx_num, uint16_t slot_idx, uint8_t *value) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "value is null");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->buf_size < slot_idx,
                      ESP_ERR_INVALID_ARG, TAG, "slot_idx error");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *value = dmx_driver[dmx_num]->buffer[slot_idx];
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_write_packet(dmx_port_t dmx_num, const void *buffer,
                           uint16_t size) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "buffer is null");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->buf_size >= size,
                      ESP_ERR_INVALID_ARG, TAG, "size error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");

  /* Writes can only happen in DMX_MODE_WRITE. Writes are made to buffer 0,
  whilst buffer 1 is used by the driver to write to the tx FIFO. */

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  memcpy(dmx_driver[dmx_num]->buffer, buffer, size);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_write_slot(dmx_port_t dmx_num, uint16_t slot_idx,
                         const uint8_t value) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->buf_size > slot_idx,
                      ESP_ERR_INVALID_ARG, TAG, "slot_idx error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");

  /* Writes can only happen in DMX_MODE_WRITE. Writes are made to buffer 0,
  whilst buffer 1 is used by the driver to write to the tx FIFO. */

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_driver[dmx_num]->buffer[slot_idx] = value;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_send_packet(dmx_port_t dmx_num, uint16_t num_slots) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(
      dmx_driver[dmx_num]->buf_size >= num_slots && num_slots > 0,
      ESP_ERR_INVALID_ARG, TAG, "num_slots error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");

  // only tx when a frame is not being written
  if (!xSemaphoreTake(dmx_driver[dmx_num]->tx.sent_sem, 0)) {
    return ESP_FAIL;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  driver->tx.size = num_slots;

  if (driver->rst_seq_hw != DMX_USE_BUSY_WAIT) {
    // ready and start the hardware timer for a reset sequence
    uint32_t break_len;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    break_len = driver->tx.break_len;
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

    driver->slot_idx = -1;
    timer_set_counter_value(driver->rst_seq_hw, driver->timer_idx, 0);
    timer_set_alarm_value(driver->rst_seq_hw, driver->timer_idx, break_len);
    dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), UART_SIGNAL_TXD_INV);
    timer_start(driver->rst_seq_hw, driver->timer_idx);

  } else {
    // driver is using busy-waits for reset sequence
    uint32_t break_len;
    uint32_t mab_len;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    break_len = driver->tx.break_len;
    mab_len = driver->tx.mab_len;
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

    /*
    This library assumes that all UART signals are un-inverted. This means that
    if the user inverts, for example, the RTS pin, these next two calls to
    dmx_hal_inverse_signal() will un-invert them. If an inverted RTS signal is
    desired, the below code will cause problems.
    */

    // send DMX break and mark-after-break
    dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), UART_SIGNAL_TXD_INV);
    ets_delay_us(break_len);
    dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), 0);
    ets_delay_us(mab_len);

    // write data to tx FIFO
    int16_t wr_len;
    dmx_hal_write_txfifo(&(dmx_context[dmx_num].hal), driver->buffer,
                         driver->tx.size, &wr_len);
    driver->slot_idx = wr_len;

    // enable tx interrupts
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_TX_ALL);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  }

  return ESP_OK;
}

esp_err_t dmx_wait_sent(dmx_port_t dmx_num, TickType_t ticks_to_wait) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");

  if (!xSemaphoreTake(dmx_driver[dmx_num]->tx.sent_sem, ticks_to_wait))
    return ESP_ERR_TIMEOUT;
  xSemaphoreGive(dmx_driver[dmx_num]->tx.sent_sem);

  return ESP_OK;
}

uint64_t dmx_get_uid() {
  uint64_t uid;
  esp_efuse_mac_get_default((uint8_t *)&uid);
  // TODO: get a real manufacturer ID
  uid = ((uint64_t)0xbeef << 32) | (uid & 0xffffffff);
  return uid;
}

uint64_t buf_to_uid(const void *buf) {
    uint64_t uid = 0;
    const uint8_t *b = (uint8_t *)buf;
    for (int bits = 40; bits >= 0; ++b, bits -= 8) uid = uid << 8 | *b;
    return uid;
}

void *uid_to_buf(const uint64_t uid, void *buf) {
    uint8_t *b = (uint8_t *)buf;
    for (int bits = 40; bits >= 0; ++b, bits -= 8) *b = uid >> bits;
    return b;
}

esp_err_t dmx_write_discovery(dmx_port_t dmx_num, uint64_t lower_uid, 
                              uint64_t upper_uid) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");
  ESP_RETURN_ON_FALSE(lower_uid <= RDM_MAX_UID && upper_uid <= RDM_MAX_UID,
                      ESP_ERR_INVALID_ARG, TAG,
                      "uid must be lower than RDM_MAX_UID");
  ESP_RETURN_ON_FALSE(lower_uid <= upper_uid, ESP_ERR_INVALID_ARG, TAG,
                      "lower_uid must be less than or equal to upper_uid");

  // Build the discovery packet
  uint8_t data[38] = {
      RDM_SC, RDM_SUB_SC,                  // RDM start code and sub-start code
      0x24,                                // Message length (excludes checksum)
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // Destination UID (broadcast)
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Source UID
      0x00,                                // Transaction number (TN)
      0x00,                                // Port ID
      0x00,                                // Message count
      0x00, 0x00,                          // Sub-device
      0x10,                                // Command class (CC)
      0x00, 0x01,                          // Parameter ID (PID)
      0x0c,                                // Parameter data length (PDL)
  };
  // TODO: replace with actual device UID based on MAC
  uid_to_buf(0xdeadbeef1234, &data[9]);  // Source UID
  uid_to_buf(lower_uid, &data[24]);      // Lower UID
  uid_to_buf(upper_uid, &data[30]);      // Upper UID

  // Compute the checksum
  uint16_t checksum = 0;
  for (int i = 0; i < sizeof(data) - 2; ++i) {
    checksum += data[i];
  }
  data[36] = checksum >> 8;
  data[37] = checksum;

  return dmx_write_packet(dmx_num, data, sizeof(data));
}

void *memcpyswap(void *dest, const void *src, size_t n) {
  char *chrsrc = (char *)src;
  char *const chrdest = (char *)dest;
  for (; n > 0; --n, chrsrc++) { // FIXME: use pointer addition/subtraction
    chrdest[n - 1] = *chrsrc;
  }
  return dest;
}

esp_err_t dmx_write_mute(dmx_port_t dmx_num, uint64_t mute_uid) {
  // TODO: check args

  // Build the mute packet
  uint8_t data[26] = {
      RDM_SC, RDM_SUB_SC,                  // RDM start code and sub-start code
      0x18,                                // Message length (excludes checksum)
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //! Destination UID
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Source UID
      0x00,                                // Transaction number (TN)
      0x00,                                //! Port ID
      0x00,                                //! Message count
      0x00, 0x00,                          // Sub-device
      0x10,                                //! Command class (CC)
      0x00, 0x02,                          //! Parameter ID (PID)
      0x00,                                //! Parameter data length (PDL)
  };  
  const uint64_t my_uid = 0xdeadbeef1234;
  memcpyswap(&data[3], &mute_uid, 6);
  memcpyswap(&data[9], &my_uid,  6);

  // Compute the checksum
  uint16_t checksum = 0;
  for (int i = 0; i < sizeof(data) - 2; ++i) {
    checksum += data[i];
  }
  data[24] = checksum >> 8;
  data[25] = checksum;

  return dmx_write_packet(dmx_num, data, sizeof(data));
}

esp_err_t dmx_wait_turnaround(dmx_port_t dmx_num, TickType_t ticks_to_wait) {
  // TODO: check args

  // if (xSemaphoreTake(dmx_driver[dmx_num]->tx.turn_sem, 0)) {
  //   return ESP_OK;
  // }

  // FIXME: enter critical section
  vTaskSuspendAll();
  const int64_t now = esp_timer_get_time();
  const int64_t elapsed = now - dmx_driver[dmx_num]->tx.last_data_ts;

  dmx_driver[dmx_num]->awaiting_turnaround = true;
  if (dmx_driver[dmx_num]->mode == DMX_MODE_WRITE) {
    if (dmx_driver[dmx_num]->tx.last_cc == RDM_DISCOVERY_COMMAND) {
      if (elapsed >= RDM_DISCOVERY_TIMEOUT) return ESP_OK;
      timer_set_counter_value(dmx_driver[dmx_num]->rst_seq_hw,
                            dmx_driver[dmx_num]->timer_idx,
                            0);
      timer_set_alarm_value(dmx_driver[dmx_num]->rst_seq_hw,
                            dmx_driver[dmx_num]->timer_idx,
                            RDM_DISCOVERY_TIMEOUT - elapsed);
      
    } else {
      // TODO
    }
  } else {
    // TODO
  }
  timer_start(dmx_driver[dmx_num]->rst_seq_hw, dmx_driver[dmx_num]->timer_idx);
  xTaskResumeAll();
  // FIXME: exit critical section

  if (!xSemaphoreTake(dmx_driver[dmx_num]->tx.turn_sem, ticks_to_wait)) {
    return ESP_ERR_TIMEOUT;
  }
  int64_t end = esp_timer_get_time();
  // ESP_LOGI(TAG, "start: %lli, end: %lli, waited: %lli, last slot: %lli, time since last slot: %lli", now, 
  //    end, end - now, dmx_driver[dmx_num]->tx.last_data_ts, 
  //    (int64_t)(end - dmx_driver[dmx_num]->tx.last_data_ts));

  return ESP_OK;
}