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

static inline int get_brk_us(int baud_rate, int break_num) {
  // get break in microseconds
  return (int)ceil(break_num * (1000000.0 / baud_rate));
}

static inline int get_mab_us(int baud_rate, int idle_num) {
  // get mark-after-break in microseconds
  return (int)ceil(idle_num * (1000000.0 / baud_rate));
}

#if SOC_UART_SUPPORT_RTC_CLK
static uint8_t rtc_enabled = 0;
static portMUX_TYPE rtc_num_spinlock = portMUX_INITIALIZER_UNLOCKED;

static void rtc_clk_enable(dmx_port_t dmx_num) {
  portENTER_CRITICAL(&rtc_num_spinlock);
  if (!(rtc_enabled & RTC_ENABLED(dmx_num))) {
    rtc_enabled |= RTC_ENABLED(dmx_num);
  }
  SET_PERI_REG_MASK(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_CLK8M_EN_M);
  portEXIT_CRITICAL(&rtc_num_spinlock);
}

static void rtc_clk_disable(dmx_port_t dmx_num) {
  assert(rtc_enabled & RTC_ENABLED(dmx_num));

  portENTER_CRITICAL(&rtc_num_spinlock);
  rtc_enabled &= ~RTC_ENABLED(dmx_num);
  if (rtc_enabled == 0) {
    CLEAR_PERI_REG_MASK(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_CLK8M_EN_M);
  }
  portEXIT_CRITICAL(&rtc_num_spinlock);
}
#endif

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
esp_err_t dmx_driver_install(dmx_port_t dmx_num, uint16_t buffer_size,
                             uint32_t queue_size, QueueHandle_t *dmx_queue,
                             int intr_alloc_flags) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(buffer_size > 0 && buffer_size <= DMX_MAX_PACKET_SIZE,
                      ESP_ERR_INVALID_ARG, TAG, "buffer_size error");

  /* Driver ISR is in IRAM so intr_alloc_flags must include the
  ESP_INTR_FLAG_IRAM flag. */
  if ((intr_alloc_flags & ESP_INTR_FLAG_IRAM) == 0) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    intr_alloc_flags |= ESP_INTR_FLAG_IRAM;
  }

  if (p_dmx_obj[dmx_num] == NULL) {
    // allocate the dmx driver
    p_dmx_obj[dmx_num] = (dmx_obj_t *)heap_caps_calloc(1, sizeof(dmx_obj_t), 
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p_dmx_obj[dmx_num] == NULL) {
      ESP_LOGE(TAG, "DMX driver malloc error");
      return ESP_ERR_NO_MEM;
    }

    // initialize the driver to default values
    p_dmx_obj[dmx_num]->dmx_num = dmx_num;
    if (dmx_queue) {
      p_dmx_obj[dmx_num]->queue = xQueueCreate(queue_size, sizeof(dmx_event_t));
      *dmx_queue = p_dmx_obj[dmx_num]->queue;
      ESP_LOGI(TAG, "queue free spaces: %d",
               uxQueueSpacesAvailable(p_dmx_obj[dmx_num]->queue));
    } else {
      p_dmx_obj[dmx_num]->queue = NULL;
    }
    p_dmx_obj[dmx_num]->buf_size = buffer_size;
#ifdef ARDUINO
    /* Arduino only allocates 4-byte aligned memory on the heap. If the size of
    the double-buffer isn't divisible by 4, when it is later freed by calling
    dmx_driver_delete() the heap will appear corrupted and the ESP32 will
    crash. As a workaround for Arduino, we allocate extra bytes until the total
    allocation is divisible by 4. These extra bytes are left unused. */
    const int alloc_size = 2 * buffer_size + (2 * buffer_size % 4);
#else
    const int alloc_size = buffer_size * 2;
#endif
    p_dmx_obj[dmx_num]->buffer[0] = heap_caps_malloc(alloc_size, 
      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p_dmx_obj[dmx_num]->buffer[0] == NULL) {
      ESP_LOGE(TAG, "DMX driver buffer malloc error");
      dmx_driver_delete(dmx_num);
      return ESP_ERR_NO_MEM;
    }
    p_dmx_obj[dmx_num]->buffer[1] = p_dmx_obj[dmx_num]->buffer[0] + buffer_size;

    p_dmx_obj[dmx_num]->slot_idx = (uint16_t)-1;
    p_dmx_obj[dmx_num]->buf_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_READ;

    p_dmx_obj[dmx_num]->rx_last_brk_ts = -DMX_RX_MAX_BRK_TO_BRK_US;
    p_dmx_obj[dmx_num]->intr_io_num = -1;
    p_dmx_obj[dmx_num]->rx_brk_len = -1;
    p_dmx_obj[dmx_num]->rx_mab_len = -1;

    p_dmx_obj[dmx_num]->tx_last_brk_ts = -DMX_TX_MAX_BRK_TO_BRK_US;
    p_dmx_obj[dmx_num]->tx_done_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(p_dmx_obj[dmx_num]->tx_done_sem);

  } else {
    ESP_LOGE(TAG, "DMX driver already installed");
    return ESP_ERR_INVALID_STATE;
  }

  // install interrupt
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), DMX_ALL_INTR_MASK);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), DMX_ALL_INTR_MASK);
  esp_err_t err = esp_intr_alloc(uart_periph_signal[dmx_num].irq, 
                                 intr_alloc_flags, &dmx_intr_handler,
                                 p_dmx_obj[dmx_num],
                                 &p_dmx_obj[dmx_num]->intr_handle);
  if (err) {
    dmx_driver_delete(dmx_num);
    return err;
  }
  const dmx_intr_config_t dmx_intr = {
      .rxfifo_full_thresh = DMX_UART_FULL_DEFAULT,
      .rx_timeout_thresh = DMX_UART_TIMEOUT_DEFAULT,
      .txfifo_empty_intr_thresh = DMX_UART_EMPTY_DEFAULT,
  };
  err = dmx_intr_config(dmx_num, &dmx_intr);
  if (err) {
    dmx_driver_delete(dmx_num);
    return err;
  }

  // enable rx interrupts and set rts
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_RX_ALL);
  dmx_hal_set_rts(&(dmx_context[dmx_num].hal), 1);  // set rts low
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");

  if (p_dmx_obj[dmx_num] == NULL) {
    ESP_LOGI(TAG, "DMX driver already null");
    return ESP_OK;
  }

  // free isr
  esp_err_t err = esp_intr_free(p_dmx_obj[dmx_num]->intr_handle);
  if (err) return err;

  // free sniffer isr
  if (p_dmx_obj[dmx_num]->intr_io_num != -1) dmx_sniffer_disable(dmx_num);

  // free driver resources
  if (p_dmx_obj[dmx_num]->buffer[0]) free(p_dmx_obj[dmx_num]->buffer[0]);
  if (p_dmx_obj[dmx_num]->queue) vQueueDelete(p_dmx_obj[dmx_num]->queue);
  if (p_dmx_obj[dmx_num]->tx_done_sem)
    vSemaphoreDelete(p_dmx_obj[dmx_num]->tx_done_sem);

  // free driver
  heap_caps_free(p_dmx_obj[dmx_num]);
  p_dmx_obj[dmx_num] = NULL;

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
  return dmx_num < DMX_NUM_MAX && p_dmx_obj[dmx_num] != NULL;
}

esp_err_t dmx_set_mode(dmx_port_t dmx_num, dmx_mode_t dmx_mode) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_mode >= 0 && dmx_mode < DMX_MODE_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_mode error");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");

  // if the driver is in the requested mode, do nothing
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  const dmx_mode_t current_dmx_mode = p_dmx_obj[dmx_num]->mode;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (current_dmx_mode == dmx_mode) return ESP_OK;

  if (dmx_mode == DMX_MODE_READ) {
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_disable_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_TX_ALL);
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_clr_intsts_mask(&(dmx_context[dmx_num].hal), DMX_ALL_INTR_MASK);

    p_dmx_obj[dmx_num]->slot_idx = (uint16_t)-1;
    p_dmx_obj[dmx_num]->buf_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_READ;
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
    if (p_dmx_obj[dmx_num]->intr_io_num != -1) dmx_sniffer_disable(dmx_num);

    p_dmx_obj[dmx_num]->slot_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_WRITE;
    xSemaphoreGive(p_dmx_obj[dmx_num]->tx_done_sem);
    dmx_hal_txfifo_rst(&(dmx_context[dmx_num].hal));
    bzero(p_dmx_obj[dmx_num]->buffer[0], p_dmx_obj[dmx_num]->buf_size);

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
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *dmx_mode = p_dmx_obj[dmx_num]->mode;
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
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->mode == DMX_MODE_READ,
                      ESP_ERR_INVALID_STATE, TAG, "must be in read mode");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->queue != NULL, ESP_ERR_INVALID_STATE,
                      TAG, "queue is null");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->intr_io_num == -1,
                      ESP_ERR_INVALID_STATE, TAG, "sniffer already enabled");

  // add the isr handler
  esp_err_t err = gpio_isr_handler_add(intr_io_num, dmx_timing_intr_handler,
                                       p_dmx_obj[dmx_num]);
  if (err) return err;

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  p_dmx_obj[dmx_num]->intr_io_num = intr_io_num;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // set to known values to allow for graceful startup
  p_dmx_obj[dmx_num]->rx_is_in_brk = false;
  p_dmx_obj[dmx_num]->rx_last_neg_edge_ts = -1;

  // enable interrupt
  gpio_set_intr_type(intr_io_num, GPIO_INTR_ANYEDGE);

  return ESP_OK;
}

esp_err_t dmx_sniffer_disable(dmx_port_t dmx_num) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->intr_io_num != -1,
                      ESP_ERR_INVALID_STATE, TAG, "sniffer not enabled");

  gpio_num_t intr_io_num;
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  intr_io_num = p_dmx_obj[dmx_num]->intr_io_num;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // disable the interrupt and remove the isr handler
  gpio_set_intr_type(intr_io_num, GPIO_INTR_DISABLE);
  esp_err_t err = gpio_isr_handler_remove(intr_io_num);
  if (err) return err;

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  p_dmx_obj[dmx_num]->intr_io_num = -1;
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

bool dmx_is_sniffer_enabled(dmx_port_t dmx_num) {
  return dmx_is_driver_installed(dmx_num) &&
         p_dmx_obj[dmx_num]->intr_io_num != -1;
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

esp_err_t dmx_param_config(dmx_port_t dmx_num, const dmx_config_t *dmx_config) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(dmx_config != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "dmx_config is null");
  ESP_RETURN_ON_FALSE(dmx_config->idle_num <= 0x3ff, ESP_ERR_INVALID_ARG, TAG,
                      "idle_num error");

  // check that the configuration is within DMX specification
  if (!DMX_BAUD_RATE_IS_VALID(dmx_config->baud_rate)) {
    ESP_LOGE(TAG, "baud_rate must be between %i and %i", DMX_MIN_BAUD_RATE,
             DMX_MAX_BAUD_RATE);
    return ESP_ERR_INVALID_ARG;
  }
  const int brk_us = get_brk_us(dmx_config->baud_rate, dmx_config->break_num);
  if (brk_us < DMX_TX_MIN_SPACE_FOR_BRK_US) {
    ESP_LOGE(TAG, "break must be at least %ius (was set to %ius)",
             DMX_TX_MIN_SPACE_FOR_BRK_US, brk_us);
    return ESP_ERR_INVALID_ARG;
  }
  const int mab_us = get_mab_us(dmx_config->baud_rate, dmx_config->idle_num);
  if (!DMX_TX_MAB_DURATION_IS_VALID(mab_us)) {
    ESP_LOGE(TAG, "mark-after-break must be between %ius and %ius (was set to "
             "%ius)", DMX_TX_MIN_MRK_AFTER_BRK_US, DMX_TX_MAX_MRK_AFTER_BRK_US,
             mab_us);
    return ESP_ERR_INVALID_ARG;
  }

  // enable uart module and rtc clock, if using it
  dmx_module_enable(dmx_num);
#if SOC_UART_SUPPORT_RTC_CLK
  if (dmx_config->source_clk == UART_SCLK_RTC) {
    rtc_clk_enable(dmx_num);
  }
#endif

  // configure the uart hardware
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_init(&(dmx_context[dmx_num].hal));
  dmx_hal_set_sclk(&(dmx_context[dmx_num].hal), dmx_config->source_clk);
  dmx_hal_set_baudrate(&(dmx_context[dmx_num].hal), dmx_config->baud_rate);
  dmx_hal_set_tx_idle_num(&(dmx_context[dmx_num].hal), dmx_config->idle_num);
  dmx_hal_set_tx_break_num(&(dmx_context[dmx_num].hal), dmx_config->break_num);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // flush both fifos
  dmx_hal_rxfifo_rst(&(dmx_context[dmx_num].hal));
  dmx_hal_txfifo_rst(&(dmx_context[dmx_num].hal));

  return ESP_OK;
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

esp_err_t dmx_set_break_num(dmx_port_t dmx_num, uint8_t break_num) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");

  // ensure the new break is within DMX specification
  uint32_t baud_rate;
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  baud_rate = dmx_hal_get_baudrate(&(dmx_context[dmx_num].hal));
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  const int brk_us = get_brk_us(baud_rate, break_num);
  if (brk_us < DMX_TX_MIN_SPACE_FOR_BRK_US) {
    ESP_LOGE(TAG, "break must be at least %ius (was set to %ius)",
             DMX_TX_MIN_SPACE_FOR_BRK_US, brk_us);
    return ESP_ERR_INVALID_ARG;
  }

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_tx_break_num(&(dmx_context[dmx_num].hal), break_num);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_break_num(dmx_port_t dmx_num, uint8_t *break_num) {
#ifdef DMX_GET_BREAK_NUM_NOT_SUPPORTED
  DMX_FUNCTION_NOT_SUPPORTED();
#endif
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(break_num != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "break_num is null");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *break_num = dmx_hal_get_break_num(&(dmx_context[dmx_num].hal));
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_idle_num(dmx_port_t dmx_num, uint16_t idle_num) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(idle_num <= 0x3ff, ESP_ERR_INVALID_ARG, TAG,
                      "idle_num error");

  // ensure the new mark-after-break is within DMX specification
  uint32_t baud_rate;
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  baud_rate = dmx_hal_get_baudrate(&(dmx_context[dmx_num].hal));
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  const int mab_us = get_mab_us(baud_rate, idle_num);
  if (!DMX_TX_MAB_DURATION_IS_VALID(mab_us)) {
    ESP_LOGE(TAG, "mark-after-break must be between %ius and %ius (was set to "
             "%ius)", DMX_TX_MIN_MRK_AFTER_BRK_US, DMX_TX_MAX_MRK_AFTER_BRK_US,
             mab_us);
    return ESP_ERR_INVALID_ARG;
  }

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_tx_idle_num(&(dmx_context[dmx_num].hal), idle_num);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_idle_num(dmx_port_t dmx_num, uint16_t *idle_num) {
#ifdef DMX_GET_IDLE_NUM_NOT_IMPLEMENTED
  DMX_FUNCTION_NOT_SUPPORTED();
#endif
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(idle_num != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "idle_num is null");

  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *idle_num = dmx_hal_get_idle_num(&(dmx_context[dmx_num].hal));
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
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->buf_size >= size, ESP_ERR_INVALID_ARG,
                      TAG, "size error");

  /* Reads can happen in either DMX_MODE_READ or DMX_MODE_WRITE. Reads while in
  DMX_MODE_READ are made from the inactive buffer while the active buffer is
  being used to collect data from the rx FIFO. Reads in DMX_MODE_WRITE are made
  from buffer 0 whilst buffer 1 is used by the driver to write to the tx
  FIFO. */

  if (size == 0) return ESP_OK;

  if (p_dmx_obj[dmx_num]->mode == DMX_MODE_READ) {
    uint8_t active_buffer;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    active_buffer = p_dmx_obj[dmx_num]->buf_idx;
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    memcpy(buffer, p_dmx_obj[dmx_num]->buffer[!active_buffer], size);
  } else {  // mode == DMX_MODE_WRITE
    memcpy(buffer, p_dmx_obj[dmx_num]->buffer[0], size);
  }

  return ESP_OK;
}

esp_err_t dmx_read_slot(dmx_port_t dmx_num, uint16_t slot_idx, uint8_t *value) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "value is null");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->buf_size < slot_idx,
                      ESP_ERR_INVALID_ARG, TAG, "slot_idx error");

  /* Reads can happen in either DMX_MODE_READ or DMX_MODE_WRITE. Reads while in
  DMX_MODE_READ are made from the inactive buffer while the active buffer is
  being used to collect data from the rx FIFO. Reads in DMX_MODE_WRITE are made
  from buffer 0 whilst buffer 1 is used by the driver to write to the tx
  FIFO. */

  if (p_dmx_obj[dmx_num]->mode == DMX_MODE_READ) {
    uint8_t active_buffer;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    active_buffer = p_dmx_obj[dmx_num]->buf_idx;
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    *value = p_dmx_obj[dmx_num]->buffer[!active_buffer][slot_idx];
  } else {  // mode == DMX_MODE_WRITE
    *value = p_dmx_obj[dmx_num]->buffer[0][slot_idx];
  }

  return ESP_OK;
}

esp_err_t dmx_write_packet(dmx_port_t dmx_num, const void *buffer,
                           uint16_t size) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(buffer != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "buffer is null");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->buf_size >= size, ESP_ERR_INVALID_ARG,
                      TAG, "size error");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");

  /* Writes can only happen in DMX_MODE_WRITE. Writes are made to buffer 0,
  whilst buffer 1 is used by the driver to write to the tx FIFO. */

  if (size == 0) return ESP_OK;

  memcpy(p_dmx_obj[dmx_num]->buffer[0], buffer, size);

  return ESP_OK;
}

esp_err_t dmx_write_slot(dmx_port_t dmx_num, uint16_t slot_idx,
                         const uint8_t value) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->buf_size > slot_idx,
                      ESP_ERR_INVALID_ARG, TAG, "slot_idx error");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");

  /* Writes can only happen in DMX_MODE_WRITE. Writes are made to buffer 0,
  whilst buffer 1 is used by the driver to write to the tx FIFO. */

  p_dmx_obj[dmx_num]->buffer[0][slot_idx] = value;

  return ESP_OK;
}

esp_err_t dmx_send_packet(dmx_port_t dmx_num, uint16_t num_slots) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->buf_size >= num_slots &&
                      num_slots > 0, ESP_ERR_INVALID_ARG, TAG,
                      "num_slots error");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num]->mode == DMX_MODE_WRITE,
                      ESP_ERR_INVALID_STATE, TAG, "not in write mode");

  // only tx when a frame is not being written
  if (xSemaphoreTake(p_dmx_obj[dmx_num]->tx_done_sem, 0) == pdFALSE)
    return ESP_FAIL;

  /* The DMX protocol states that frames begin with a break, followed by a
  mark, followed by up to 513 bytes. The ESP32 uart hardware is designed to
  send a packet, followed by a break, followed by a mark. When using this
  library "correctly," there shouldn't be any issues because the data stream
  will be continuous - even though the hardware sends the break and mark after
  the packet, it will LOOK like it is being sent before the packet. However if
  the byte stream isn't continuous, we need to send a break and mark before we
  send the packet. This is done by inverting the line, busy waiting,
  un-inverting the line and busy waiting again. The busy waiting isn't very
  accurate (it's usually accurate within around 30us if the task isn't
  preempted), but it is the best that can be done short of using the ESP32
  hardware timer library. */

  // check if we need to send a new break and mark after break
  const int64_t now = esp_timer_get_time();
  if (now - p_dmx_obj[dmx_num]->tx_last_brk_ts >= DMX_TX_MAX_BRK_TO_BRK_US) {
    // get break and mark time in microseconds
    uint32_t baud_rate, break_num, idle_num;
    portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    baud_rate = dmx_hal_get_baudrate(&(dmx_context[dmx_num].hal));
    break_num = dmx_hal_get_break_num(&(dmx_context[dmx_num].hal));
    idle_num = dmx_hal_get_idle_num(&(dmx_context[dmx_num].hal));
    portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    const int brk_us = get_brk_us(baud_rate, break_num);
    const int mab_us = get_mab_us(baud_rate, idle_num);

    /* This library assumes that all UART signals are un-inverted. This means
    that if the user inverts, say, the RTS pin, these next two calls to
    dmx_hal_inverse_signal() will un-invert them. If an inverted RTS signal is
    desired, the below code will cause problems. */

    // invert the tx line and busy wait...
    dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), UART_SIGNAL_TXD_INV);
    ets_delay_us(brk_us);

    // un-invert the tx line and busy wait...
    dmx_hal_inverse_signal(&(dmx_context[dmx_num].hal), 0);
    ets_delay_us(mab_us);

    p_dmx_obj[dmx_num]->tx_last_brk_ts = now;
  }

  // write data to tx FIFO
  uint32_t bytes_written;
  p_dmx_obj[dmx_num]->send_size = num_slots;
  const uint32_t buf_idx = p_dmx_obj[dmx_num]->buf_idx;
  const uint8_t *zeroeth_slot = p_dmx_obj[dmx_num]->buffer[buf_idx];
  dmx_hal_write_txfifo(&(dmx_context[dmx_num].hal), zeroeth_slot, num_slots,
                       &bytes_written);
  p_dmx_obj[dmx_num]->slot_idx = bytes_written;

  // enable tx interrupts
  portENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_ena_intr_mask(&(dmx_context[dmx_num].hal), DMX_INTR_TX_ALL);
  portEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_wait_send_done(dmx_port_t dmx_num, TickType_t ticks_to_wait) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(p_dmx_obj[dmx_num] != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "driver not installed");

  /* Just try to take the "done" semaphore and give it back immediately. */

  if (xSemaphoreTake(p_dmx_obj[dmx_num]->tx_done_sem, ticks_to_wait) == pdFALSE)
    return ESP_ERR_TIMEOUT;
  xSemaphoreGive(p_dmx_obj[dmx_num]->tx_done_sem);

  return ESP_OK;
}
