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
#include "soc/rtc_cntl_reg.h"
#include "soc/uart_periph.h"

#if SOC_UART_SUPPORT_RTC_CLK
#define RTC_ENABLED(uart_num) (BIT(uart_num))
#endif

// The default value for the RX FIFO full interrupt threshold.
#define DMX_UART_FULL_DEFAULT (120)
// The default value for the TX FIFO empty interrupt threshold.
#define DMX_UART_EMPTY_DEFAULT (8)
// The default value for the UART timeout interrupt.
#define DMX_UART_TIMEOUT_DEFAULT (126)
// The max value for the RX FIFO full interrupt threshold.
#define DMX_RXFIFO_FULL_THRESHOLD_MAX (0x7F)
// The max value for the TX FIFO empty interrupt threshold.
#define DMX_TXFIFO_EMPTY_THRESHOLD_MAX (0x7F)

#define DMX_ALL_INTR_MASK (-1)

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
  ESP_RETURN_ON_FALSE(dmx_config->rst_seq_hw == DMX_USE_UART || 
                      dmx_config->rst_seq_hw < DMX_RESET_SEQUENCE_MAX, 
                      ESP_ERR_INVALID_ARG, TAG, "rst_seq_hw error");
  ESP_RETURN_ON_FALSE(dmx_config->timer_idx < TIMER_MAX, 
                      ESP_ERR_INVALID_ARG, TAG, "timer_idx error");

  /* Driver ISR is in IRAM so intr_alloc_flags must include the
  ESP_INTR_FLAG_IRAM flag. */
  if ((dmx_config->intr_alloc_flags & ESP_INTR_FLAG_IRAM) == 0) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    dmx_config->intr_alloc_flags |= ESP_INTR_FLAG_IRAM;
  }
    
  // configure the uart hardware
  dmx_module_enable(dmx_num);
  uint8_t brk_num = 0;
  uint16_t idle_num = 0;
  if (dmx_config->rst_seq_hw == DMX_USE_UART) {
    // user is not using a hardware timer for the reset sequence
    brk_num = 44;
    idle_num = 3;
  }
  dmx_context_t hardware_ctx = dmx_context[dmx_num];
  portENTER_CRITICAL(&(hardware_ctx.spinlock));
  dmx_hal_init(&(hardware_ctx.hal));
  dmx_hal_set_sclk(&(hardware_ctx.hal), UART_SCLK_APB);
  dmx_hal_set_baudrate(&(hardware_ctx.hal), DMX_TYP_BAUD_RATE);
  dmx_hal_set_tx_break_num(&(hardware_ctx.hal), brk_num);
  dmx_hal_set_tx_idle_num(&(hardware_ctx.hal), idle_num);
  portEXIT_CRITICAL(&(hardware_ctx.spinlock));

  // flush both fifos
  dmx_hal_rxfifo_rst(&(hardware_ctx.hal));
  dmx_hal_txfifo_rst(&(hardware_ctx.hal));

  // allocate the dmx driver
  const uint32_t mem_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  dmx_driver[dmx_num] = heap_caps_malloc(sizeof(dmx_driver_t), mem_caps);
  if (dmx_driver[dmx_num] == NULL) {
    ESP_LOGE(TAG, "DMX driver malloc error");
    return ESP_ERR_NO_MEM;
  }
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // allocate driver double-buffer
  int alloc_size = dmx_config->buffer_size;
#ifdef ARDUINO
  /* Arduino only allocates 4-byte aligned memory on the heap. If the size of
  the double-buffer isn't divisible by 4, when it is later freed by calling
  dmx_driver_delete() the heap will appear corrupted and the ESP32 will
  crash. As a workaround for Arduino, we allocate extra bytes until the total
  allocation is divisible by 4. These extra bytes are left unused. */
  alloc_size += (dmx_config->buffer_size % 4);
#endif
  // TODO: try to heaps_cap_malloc the buffer to 8BIT to see if arduino still crashes
  driver->buffer = malloc(sizeof(uint8_t) * alloc_size);
  if (driver->buffer == NULL) {
    ESP_LOGE(TAG, "DMX driver buffer malloc error");
    dmx_driver_delete(dmx_num);
    return ESP_ERR_NO_MEM;
  }

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
  driver->tx.done_sem = xSemaphoreCreateBinary();
  if (driver->tx.done_sem == NULL) {
    ESP_LOGE(TAG, "DMX driver semaphore malloc error");
    dmx_driver_delete(dmx_num);
    return ESP_ERR_NO_MEM;
  }
  xSemaphoreGive(driver->tx.done_sem);

  // initialize general driver variables
  driver->dmx_num = dmx_num;
  driver->buf_size = dmx_config->buffer_size;
  driver->slot_idx = (uint16_t)-1;
  driver->mode = DMX_MODE_READ;
  driver->rst_seq_hw = dmx_config->rst_seq_hw;

  // initialize driver tx variables
  driver->tx.break_len = DMX_TX_TYP_SPACE_FOR_BRK_US;
  driver->tx.mab_len = DMX_TX_MIN_MRK_AFTER_BRK_US;
  if (driver->rst_seq_hw == DMX_USE_UART) {
    driver->tx.last_break_ts = -DMX_TX_MAX_BRK_TO_BRK_US;
  } else {
    driver->tx.timer_idx = dmx_config->timer_idx;
  }

  // initialize driver rx variables
  driver->rx.last_break_ts = -DMX_RX_MAX_BRK_TO_BRK_US;
  driver->rx.intr_io_num = -1;
  driver->rx.break_len = -1;
  driver->rx.mab_len = -1;

  // install uart interrupt
  portENTER_CRITICAL(&(hardware_ctx.spinlock));
  dmx_hal_disable_intr_mask(&(hardware_ctx.hal), DMX_ALL_INTR_MASK);
  portEXIT_CRITICAL(&(hardware_ctx.spinlock));
  dmx_hal_clr_intsts_mask(&(hardware_ctx.hal), DMX_ALL_INTR_MASK);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, dmx_config->intr_alloc_flags, 
                 &dmx_intr_handler, driver, &driver->uart_isr_handle);
  const dmx_intr_config_t dmx_intr_conf = {
      .rxfifo_full_thresh = DMX_UART_FULL_DEFAULT,
      .rx_timeout_thresh = DMX_UART_TIMEOUT_DEFAULT,
      .txfifo_empty_intr_thresh = DMX_UART_EMPTY_DEFAULT,
  };
  dmx_intr_config(dmx_num, &dmx_intr_conf);

  // enable rx interrupt and set rts
  portENTER_CRITICAL(&(hardware_ctx.spinlock));
  dmx_hal_ena_intr_mask(&(hardware_ctx.hal), DMX_INTR_RX_ALL);
  dmx_hal_set_rts(&(hardware_ctx.hal), 1);  // set rts low
  portEXIT_CRITICAL(&(hardware_ctx.spinlock));

  // install timer interrupt
  if (dmx_driver[dmx_num]->rst_seq_hw != DMX_USE_UART) {
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
  if (driver->rst_seq_hw != DMX_USE_UART) {
    timer_deinit(driver->rst_seq_hw, driver->tx.timer_idx);
  }
  
  // free sniffer isr
  if (driver->rx.intr_io_num != -1) dmx_sniffer_disable(dmx_num);

  // free driver resources
  if (driver->buffer[0]) free(driver->buffer);
  if (driver->rx.queue) vQueueDelete(dmx_driver[dmx_num]->rx.queue);
  if (driver->tx.done_sem) vSemaphoreDelete(driver->tx.done_sem);

  // free driver
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;

  // disable rtc clock (if using it) and uart peripheral module
#if SOC_UART_SUPPORT_RTC_CLK
  uart_sclk_t sclk = 0;
  dmx_hal_get_sclk(&(dmx_context[dmx_num].hal), &sclk);
  if (sclk == UART_SCLK_RTC) {
    rtc_clk_disable(dmx_num);
  }
#endif
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

    dmx_driver[dmx_num]->slot_idx = (uint16_t)-1;
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

    dmx_driver[dmx_num]->slot_idx = 0;
    dmx_driver[dmx_num]->mode = DMX_MODE_WRITE;
    xSemaphoreGive(dmx_driver[dmx_num]->tx.done_sem);
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

  // check that the new baud_rate is within DMX specification
  if (!DMX_BAUD_RATE_IS_VALID(baud_rate)) {
    ESP_LOGE(TAG, "baud_rate must be between %i and %i", DMX_MIN_BAUD_RATE,
             DMX_MAX_BAUD_RATE);
    return ESP_ERR_INVALID_ARG;
  }

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
  ESP_RETURN_ON_FALSE(DMX_TX_BRK_DURATION_IS_VALID(break_len), 
                      ESP_ERR_INVALID_ARG, TAG, "break_len error");

  if (dmx_driver[dmx_num]->rst_seq_hw != DMX_USE_UART) {
    // driver is using hardware timers for reset sequence
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_driver[dmx_num]->tx.break_len = break_len;
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  } else {
    // driver is using uart hardware for reset sequence
    uint32_t baud_rate;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_get_baud_rate(dmx_num, &baud_rate);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    const uint32_t break_num = ceil(break_len / (1000000.0 / baud_rate));
    ESP_RETURN_ON_FALSE(break_num < 256, ESP_ERR_INVALID_STATE, TAG, 
                        "break_len error (reset-sequence-last mode)");

    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_set_tx_break_num(&(dmx_context[dmx_num].hal), break_num);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  }

  return ESP_OK;
}

esp_err_t dmx_get_break_len(dmx_port_t dmx_num, uint32_t *break_len) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(break_len != NULL, ESP_ERR_INVALID_ARG, TAG, 
                      "break_len is null");

  if (dmx_driver[dmx_num]->rst_seq_hw != DMX_USE_UART) {
    // driver is using hardware timers for reset sequence
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    *break_len = dmx_driver[dmx_num]->tx.break_len;
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  } else {
    // driver is using uart hardware for reset sequence
#ifdef DMX_GET_BREAK_NUM_NOT_IMPLEMENTED
    DMX_FUNCTION_NOT_SUPPORTED();
#endif

    uint32_t baud_rate;
    uint8_t break_num;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_get_baud_rate(dmx_num, &baud_rate);
    break_num = dmx_hal_get_break_num(&(dmx_context[dmx_num].hal));
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    *break_len = (1000000.0 / baud_rate) * break_num;
  }

  return ESP_OK;
}

esp_err_t dmx_set_mab_len(dmx_port_t dmx_num, uint32_t mab_len) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(DMX_TX_MAB_DURATION_IS_VALID(mab_len), 
                      ESP_ERR_INVALID_ARG, TAG, "mab_len error");
  
  if (dmx_driver[dmx_num]->rst_seq_hw != DMX_USE_UART) {
    // driver is using hardware timers for reset sequence
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_driver[dmx_num]->tx.mab_len = mab_len;
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  } else {
    // driver is using uart hardware for reset sequence
    uint32_t baud_rate;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_get_baud_rate(dmx_num, &baud_rate);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    const uint32_t idle_num = ceil(mab_len / (1000000.0 / baud_rate));
    ESP_RETURN_ON_FALSE(idle_num < 1024, ESP_ERR_INVALID_STATE, TAG, 
                        "mab_len error (reset-sequence-last mode)");

    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_set_tx_idle_num(&(dmx_context[dmx_num].hal), idle_num);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  }

  return ESP_OK;
}

esp_err_t dmx_get_mab_len(dmx_port_t dmx_num, uint32_t *mab_len) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(mab_len != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "mab_len is null");

  if (dmx_driver[dmx_num]->rst_seq_hw != DMX_USE_UART) {
    // driver is using hardware timers for reset sequence
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    *mab_len = dmx_driver[dmx_num]->tx.mab_len;
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  } else {
    // driver is using uart hardware for reset sequence
#ifdef DMX_GET_IDLE_NUM_NOT_IMPLEMENTED
    DMX_FUNCTION_NOT_SUPPORTED();
#endif

    uint32_t baud_rate;
    uint8_t idle_num;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_get_baud_rate(dmx_num, &baud_rate);
    idle_num = dmx_hal_get_break_num(&(dmx_context[dmx_num].hal));
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    *mab_len = (1000000.0 / baud_rate) * idle_num;
  }

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
                         intr_conf->rx_timeout_thresh);
  dmx_hal_set_rxfifo_full_thr(&(dmx_context[dmx_num].hal),
                              intr_conf->rxfifo_full_thresh);
  dmx_hal_set_txfifo_empty_thr(&(dmx_context[dmx_num].hal),
                               intr_conf->txfifo_empty_intr_thresh);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_rx_full_threshold(dmx_port_t dmx_num, int threshold) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(threshold > 0 && 
                      threshold < DMX_RXFIFO_FULL_THRESHOLD_MAX,
                      ESP_ERR_INVALID_ARG, TAG, 
                      "rx fifo full threshold value error");

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
  ESP_RETURN_ON_FALSE(threshold > 0 &&
                      threshold < DMX_TXFIFO_EMPTY_THRESHOLD_MAX,
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
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->buf_size >= size, ESP_ERR_INVALID_ARG,
                      TAG, "size error");

  if (size == 0) return ESP_OK; // TODO: can we remove this line? 

  memcpy(buffer, dmx_driver[dmx_num]->buffer, size);

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

  *value = dmx_driver[dmx_num]->buffer[slot_idx];

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
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->buf_size >= size, ESP_ERR_INVALID_ARG,
                      TAG, "size error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");

  /* Writes can only happen in DMX_MODE_WRITE. Writes are made to buffer 0,
  whilst buffer 1 is used by the driver to write to the tx FIFO. */

  if (size == 0) return ESP_OK;

  memcpy(dmx_driver[dmx_num]->buffer, buffer, size);

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

  dmx_driver[dmx_num]->buffer[slot_idx] = value;

  return ESP_OK;
}

esp_err_t dmx_send_packet(dmx_port_t dmx_num, uint16_t num_slots) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->buf_size >= num_slots &&
                      num_slots > 0, ESP_ERR_INVALID_ARG, TAG,
                      "num_slots error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");

  // only tx when a frame is not being written
  if (xSemaphoreTake(dmx_driver[dmx_num]->tx.done_sem, 0) == pdFALSE)
    return ESP_FAIL;

  /* There are two modes in which this library can transmit DMX: reset-sequence
  first mode or reset-sequence last mode. The default is reset-sequence first
  mode. This mode uses one of the ESP32's hardware timers to precisely send a 
  break and mark-after-break to DMX listener devices before sending the DMX data
  packet. Reset-sequence last mode takes advantage of the ESP32 UART hardware to 
  send a break and mark-after-break after the data packet is sent. When using 
  this library "correctly," there shouldn't be any issues because the data 
  stream will be continuous - even though the hardware sends the break and 
  mark-after-break after the packet, it will LOOK like it is being sent before 
  the packet. However if the byte stream isn't continuous, we need to send a 
  break and mark-after-break before we send the packet. This is done by 
  inverting the line, busy waiting, un-inverting the line and busy waiting 
  again. The busy waiting isn't very accurate (it's usually accurate within
  around 30us if the task isn't preempted), but it can be used when the user 
  is unable to use one of the available hardware timers on the ESP32. */

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  if (driver->rst_seq_hw != DMX_USE_UART) {
    // driver is using hardware timers for reset sequence

    // ready the dmx driver to send a reset sequence
    driver->tx.size = num_slots;
    driver->tx.step = 0;

    // ready and start the hardware timer for a reset sequence
    timer_set_alarm_value(driver->rst_seq_hw, driver->tx.timer_idx, 0);
    timer_set_counter_value(driver->rst_seq_hw, driver->tx.timer_idx, 0);
    timer_start(driver->rst_seq_hw, driver->tx.timer_idx);
  } else {
    // driver is using uart hardware for reset sequence
    const int64_t now = esp_timer_get_time();

    // check if a simulated reset sequence must be sent
    if (now - driver->tx.last_break_ts >= DMX_TX_MAX_BRK_TO_BRK_US) {
      // get break and mark time in microseconds
      uint32_t baud_rate, break_num, idle_num;
      portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
      baud_rate = dmx_hal_get_baudrate(&(dmx_context[dmx_num].hal));
      break_num = dmx_hal_get_break_num(&(dmx_context[dmx_num].hal));
      idle_num = dmx_hal_get_idle_num(&(dmx_context[dmx_num].hal));
      portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
      const int brk_us = (int)ceil(break_num * (1000000.0 / baud_rate));
      const int mab_us = (int)ceil(idle_num * (1000000.0 / baud_rate));

      /* This library assumes that all UART signals are un-inverted. This means
      that if the user inverts, for example, the RTS pin, these next two calls
      to dmx_hal_inverse_signal() will un-invert them. If an inverted RTS signal
      is desired, the below code will cause problems. */

      // invert the tx line and busy wait...
      dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), UART_SIGNAL_TXD_INV);
      ets_delay_us(brk_us);

      // un-invert the tx line and busy wait...
      dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), 0);
      ets_delay_us(mab_us);

      driver->tx.last_break_ts = now;
    }

    // write data to tx FIFO
    uint32_t bytes_written;
    dmx_hal_write_txfifo(&(dmx_context[dmx_num].hal), driver->buffer, 
                         driver->tx.size, &bytes_written);
    driver->slot_idx = bytes_written;

    // enable tx interrupts
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_TX_ALL);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  }

  return ESP_OK;
}

esp_err_t dmx_wait_send_done(dmx_port_t dmx_num, TickType_t ticks_to_wait) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_driver[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");

  /* Just try to take the "done" semaphore and give it back immediately. */

  if (xSemaphoreTake(dmx_driver[dmx_num]->tx.done_sem, ticks_to_wait) == pdFALSE)
    return ESP_ERR_TIMEOUT;
  xSemaphoreGive(dmx_driver[dmx_num]->tx.done_sem);

  return ESP_OK;
}
