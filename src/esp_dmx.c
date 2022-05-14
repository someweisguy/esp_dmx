#include "esp_dmx.h"

#include <math.h>
#include <string.h>

#include "dmx_types.h"
#include "dmx/driver.h"
#include "dmx/hal.h"
#include "dmx/intr_handlers.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/io_mux_reg.h"
#include "soc/uart_periph.h"

/* Define the UART number for the console. Depends on which platform is in use. */
#ifdef CONFIG_ESP_CONSOLE_UART_NUM
// UART number of the console is defined in sdkconfig (using ESP-IDF).
#define CONSOLE_UART_NUM  CONFIG_ESP_CONSOLE_UART_NUM
#elif defined(ARDUINO_USB_CDC_ON_BOOT)
// UART number of the console is defined in HardwareSerial.h (using Arduino).
#define CONSOLE_UART_NUM  ARDUINO_USB_CDC_ON_BOOT
#else
// UART number of the console is not defined.
#define CONSOLE_UART_NUM  -1
#endif

#define DMX_EMPTY_THRESH_DEFAULT  8
#define DMX_FULL_THRESH_DEFAULT   120
#define DMX_TOUT_THRESH_DEFAULT   126

#define DMX_RXFIFO_FULL_THRHD_V   0x7F
#define DMX_TXFIFO_EMPTY_THRHD_V  0x7F

#define DMX_ALL_INTR_MASK         -1

#define DMX_ENTER_CRITICAL(mux)   portENTER_CRITICAL(mux)
#define DMX_EXIT_CRITICAL(mux)    portEXIT_CRITICAL(mux)

static const char *TAG = "dmx";
#define DMX_CHECK(a, str, ret_val)                            \
  if (!(a)) {                                                 \
    ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
    return (ret_val);                                         \
  }

static inline int get_brk_us(int baud_rate, int break_num) {
    // get break in microseconds
    return (int) ceil(break_num * (1000000.0 / baud_rate));
}

static inline int get_mab_us(int baud_rate, int idle_num) {
    // get mark-after-break in microseconds
    return (int) ceil(idle_num * (1000000.0 / baud_rate));
}

/// Driver Functions  #########################################################

esp_err_t dmx_driver_install(dmx_port_t dmx_num, int buffer_size,
    int queue_size, QueueHandle_t *dmx_queue, int intr_alloc_flags) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(buffer_size > 0 && buffer_size <= DMX_MAX_PACKET_SIZE, "buffer_size error", ESP_ERR_INVALID_ARG);

  // driver ISR is in IRAM so intr_alloc_flags must include the 
  //  ESP_INTR_FLAG_IRAM flag
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
      ESP_LOGI(TAG, "queue free spaces: %d", uxQueueSpacesAvailable(p_dmx_obj[dmx_num]->queue));
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
    p_dmx_obj[dmx_num]->buffer[0] = malloc(sizeof(uint8_t) * alloc_size);
    if (p_dmx_obj[dmx_num]->buffer[0] == NULL) {
      ESP_LOGE(TAG, "DMX driver buffer malloc error");
        dmx_driver_delete(dmx_num);
        return ESP_ERR_NO_MEM;
    }
    p_dmx_obj[dmx_num]->buffer[1] = p_dmx_obj[dmx_num]->buffer[0] + buffer_size;

    p_dmx_obj[dmx_num]->slot_idx = (uint16_t)-1;
    p_dmx_obj[dmx_num]->buf_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_RX;

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

  // enable uart peripheral module
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != true) {
    if (dmx_num != CONSOLE_UART_NUM) {
      periph_module_reset(uart_periph_signal[dmx_num].module);
    }
    periph_module_enable(uart_periph_signal[dmx_num].module);
    dmx_context[dmx_num].hw_enabled = true;
  }
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // install interrupt
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_disable_intr_mask(dmx_context[dmx_num].dev, DMX_ALL_INTR_MASK);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_clr_intsts_mask(dmx_context[dmx_num].dev, DMX_ALL_INTR_MASK);
  esp_err_t err = esp_intr_alloc(uart_periph_signal[dmx_num].irq,
      intr_alloc_flags, &dmx_intr_handler, p_dmx_obj[dmx_num],
      &p_dmx_obj[dmx_num]->intr_handle);
  if (err) {
    dmx_driver_delete(dmx_num);
    return err;
  }
  const dmx_intr_config_t dmx_intr = {
      .rxfifo_full_thresh = DMX_FULL_THRESH_DEFAULT,
      .rx_timeout_thresh = DMX_TOUT_THRESH_DEFAULT,
      .txfifo_empty_intr_thresh = DMX_EMPTY_THRESH_DEFAULT,
  };
  err = dmx_intr_config(dmx_num, &dmx_intr);
  if (err) {
    dmx_driver_delete(dmx_num);
    return err;
  }

  // enable rx interrupts and set rts
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_ena_intr_mask(dmx_context[dmx_num].dev, DMX_INTR_RX_ALL);
  dmx_hal_set_rts(dmx_context[dmx_num].dev, 1); // set rts low
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

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

  // free rx analyzer isr
  if (p_dmx_obj[dmx_num]->intr_io_num != -1) 
    dmx_rx_timing_disable(dmx_num);

  // free driver resources
  if (p_dmx_obj[dmx_num]->buffer[0])
    free(p_dmx_obj[dmx_num]->buffer[0]);
  if (p_dmx_obj[dmx_num]->queue)
    vQueueDelete(p_dmx_obj[dmx_num]->queue);
  if (p_dmx_obj[dmx_num]->tx_done_sem) 
    vSemaphoreDelete(p_dmx_obj[dmx_num]->tx_done_sem);

  // free driver
  heap_caps_free(p_dmx_obj[dmx_num]);
  p_dmx_obj[dmx_num] = NULL;

  // disable uart peripheral module
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != false) {
    if (dmx_num != CONSOLE_UART_NUM) {
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
    dmx_hal_disable_intr_mask(dmx_context[dmx_num].dev, DMX_INTR_TX_ALL);
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_clr_intsts_mask(dmx_context[dmx_num].dev, DMX_ALL_INTR_MASK);

    p_dmx_obj[dmx_num]->slot_idx = (uint16_t)-1;
    p_dmx_obj[dmx_num]->buf_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_RX;
    dmx_hal_rxfifo_rst(dmx_context[dmx_num].dev);

    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_set_rts(dmx_context[dmx_num].dev, 1); // set rts low
    dmx_hal_ena_intr_mask(dmx_context[dmx_num].dev, DMX_INTR_RX_ALL);
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    
  } else { // dmx_mode == DMX_MODE_TX
    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_disable_intr_mask(dmx_context[dmx_num].dev, DMX_INTR_RX_ALL);
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_clr_intsts_mask(dmx_context[dmx_num].dev, DMX_ALL_INTR_MASK);

    // disable rx timing if it is enabled
    if (p_dmx_obj[dmx_num]->intr_io_num != -1)
      dmx_rx_timing_disable(dmx_num);

    p_dmx_obj[dmx_num]->slot_idx = 0;
    p_dmx_obj[dmx_num]->mode = DMX_MODE_TX;
    xSemaphoreGive(p_dmx_obj[dmx_num]->tx_done_sem);
    dmx_hal_txfifo_rst(dmx_context[dmx_num].dev);
    bzero(p_dmx_obj[dmx_num]->buffer[0], p_dmx_obj[dmx_num]->buf_size);

    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    dmx_hal_set_rts(dmx_context[dmx_num].dev, 0); // set rts high
    // tx interrupts are enabled when calling the tx function!!
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  }

  return ESP_OK;
}

esp_err_t dmx_get_mode(dmx_port_t dmx_num, dmx_mode_t *dmx_mode) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *dmx_mode = p_dmx_obj[dmx_num]->mode;  
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_rx_timing_enable(dmx_port_t dmx_num, int intr_io_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(GPIO_IS_VALID_GPIO(intr_io_num), "intr_io_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(p_dmx_obj[dmx_num]->mode == DMX_MODE_RX, "must be in rx mode", ESP_ERR_INVALID_STATE);
  DMX_CHECK(p_dmx_obj[dmx_num]->queue, "queue is null", ESP_ERR_INVALID_STATE);
  DMX_CHECK(p_dmx_obj[dmx_num]->intr_io_num == -1, "rx analyze already enabled", ESP_ERR_INVALID_STATE);

  // add the isr handler
  esp_err_t err = gpio_isr_handler_add(intr_io_num, dmx_timing_intr_handler, 
    p_dmx_obj[dmx_num]);
  if (err) return err;

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  p_dmx_obj[dmx_num]->intr_io_num = intr_io_num;
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // set to known values to allow for graceful startup
  p_dmx_obj[dmx_num]->rx_is_in_brk = false;
  p_dmx_obj[dmx_num]->rx_last_neg_edge_ts = -1;  

  // enable interrupt
  gpio_set_intr_type(intr_io_num, GPIO_INTR_ANYEDGE);

  return ESP_OK;
}

esp_err_t dmx_rx_timing_disable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(p_dmx_obj[dmx_num]->intr_io_num != -1, "rx analyze not enabled", ESP_ERR_INVALID_STATE);

  gpio_num_t intr_io_num;
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  intr_io_num = p_dmx_obj[dmx_num]->intr_io_num;
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // disable the interrupt and remove the isr handler
  gpio_set_intr_type(intr_io_num, GPIO_INTR_DISABLE);
  esp_err_t err = gpio_isr_handler_remove(intr_io_num);
  if (err) return err;

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  p_dmx_obj[dmx_num]->intr_io_num = -1;
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

bool dmx_is_rx_timing_enabled(dmx_port_t dmx_num) {
  return dmx_is_driver_installed(dmx_num) && p_dmx_obj[dmx_num]->intr_io_num != -1;
}

/// Hardware Configuration  ###################################################

esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_io_num, int rx_io_num,
    int rts_io_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK((tx_io_num < 0 || (GPIO_IS_VALID_OUTPUT_GPIO(tx_io_num))), "tx_io_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK((rx_io_num < 0 || (GPIO_IS_VALID_GPIO(rx_io_num))), "rx_io_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK((rts_io_num < 0 || (GPIO_IS_VALID_OUTPUT_GPIO(rts_io_num))), "rts_io_num error", ESP_ERR_INVALID_ARG);
  
  return uart_set_pin(dmx_num, tx_io_num, rx_io_num, rts_io_num, -1);
}

esp_err_t dmx_param_config(dmx_port_t dmx_num, const dmx_config_t *dmx_config) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(dmx_config, "dmx_config is null", ESP_ERR_INVALID_ARG);
  DMX_CHECK(dmx_config->idle_num <= 0x3ff, "idle_num error", ESP_ERR_INVALID_ARG);

  // check that the configuration is within DMX specification
  if (dmx_config->baud_rate < DMX_MIN_BAUD_RATE || dmx_config->baud_rate > DMX_MAX_BAUD_RATE) {
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
  if (mab_us < DMX_TX_MIN_MRK_AFTER_BRK_US || mab_us > DMX_TX_MAX_MRK_AFTER_BRK_US) {
    ESP_LOGE(TAG, "mark-after-break must be between %ius and %ius (was set to %ius)",
      DMX_TX_MIN_MRK_AFTER_BRK_US, DMX_TX_MAX_MRK_AFTER_BRK_US, mab_us);
    return ESP_ERR_INVALID_ARG;
  }

  // enable uart peripheral module
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != true) {
    if (dmx_num != CONSOLE_UART_NUM) {
      periph_module_reset(uart_periph_signal[dmx_num].module);
    }
    periph_module_enable(uart_periph_signal[dmx_num].module);
    dmx_context[dmx_num].hw_enabled = true;
  }
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  // configure the uart hardware
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_init(dmx_context[dmx_num].dev, dmx_num);
  dmx_hal_set_baudrate(dmx_context[dmx_num].dev, dmx_config->source_clk, 
    dmx_config->baud_rate);
  dmx_hal_set_tx_idle_num(dmx_context[dmx_num].dev, dmx_config->idle_num);
  dmx_hal_set_tx_break_num(dmx_context[dmx_num].dev, dmx_config->break_num);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  
  // flush both fifos
  dmx_hal_rxfifo_rst(dmx_context[dmx_num].dev);
  dmx_hal_txfifo_rst(dmx_context[dmx_num].dev);

  return ESP_OK;
}

esp_err_t dmx_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  // check that the new baud_rate is within DMX specification
  if (baud_rate < DMX_MIN_BAUD_RATE || baud_rate > DMX_MAX_BAUD_RATE) {
    ESP_LOGE(TAG, "baud_rate must be between %i and %i", DMX_MIN_BAUD_RATE,
      DMX_MAX_BAUD_RATE);
    return ESP_ERR_INVALID_ARG;
  }
  
  uart_sclk_t source_clk;
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  source_clk = dmx_hal_get_sclk(dmx_context[dmx_num].dev);
  dmx_hal_set_baudrate(dmx_context[dmx_num].dev, source_clk, baud_rate);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_baud_rate(dmx_port_t dmx_num, uint32_t *baud_rate) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *baud_rate = dmx_hal_get_baudrate(dmx_context[dmx_num].dev);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_break_num(dmx_port_t dmx_num, uint8_t break_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  // ensure the new break is within DMX specification
  uint32_t baud_rate;
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  baud_rate = dmx_hal_get_baudrate(dmx_context[dmx_num].dev);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  const int brk_us = get_brk_us(baud_rate, break_num);
  if (brk_us < DMX_TX_MIN_SPACE_FOR_BRK_US) {
    ESP_LOGE(TAG, "break must be at least %ius (was set to %ius)", 
      DMX_TX_MIN_SPACE_FOR_BRK_US, brk_us);
    return ESP_ERR_INVALID_ARG;
  }

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_tx_break_num(dmx_context[dmx_num].dev, break_num);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_break_num(dmx_port_t dmx_num, uint8_t *break_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *break_num = dmx_hal_get_break_num(dmx_context[dmx_num].dev);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_idle_num(dmx_port_t dmx_num, uint16_t idle_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(idle_num <= 0x3ff, "idle_num error", ESP_ERR_INVALID_ARG);
  
  // ensure the new mark-after-break is within DMX specification
  uint32_t baud_rate;
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  baud_rate = dmx_hal_get_baudrate(dmx_context[dmx_num].dev);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
  const int mab_us = get_mab_us(baud_rate, idle_num);
  if (mab_us < DMX_TX_MIN_MRK_AFTER_BRK_US || mab_us > DMX_TX_MAX_MRK_AFTER_BRK_US) {
    ESP_LOGE(TAG, "mark-after-break must be between %ius and %ius (was set to %ius)",
      DMX_TX_MIN_MRK_AFTER_BRK_US, DMX_TX_MAX_MRK_AFTER_BRK_US, mab_us);
    return ESP_ERR_INVALID_ARG;
  }

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_tx_idle_num(dmx_context[dmx_num].dev, idle_num);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_get_idle_num(dmx_port_t dmx_num, uint16_t *idle_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  *idle_num = dmx_hal_get_idle_num(dmx_context[dmx_num].dev);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_invert_rts(dmx_port_t dmx_num, bool invert) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_inverse_rts_signal(dmx_context[dmx_num].dev, invert);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

/// Interrupt Configuration  ##################################################

esp_err_t dmx_intr_config(dmx_port_t dmx_num, const dmx_intr_config_t *intr_conf) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(intr_conf, "intr_conf is null", ESP_ERR_INVALID_ARG);

  dmx_hal_clr_intsts_mask(dmx_context[dmx_num].dev, DMX_ALL_INTR_MASK);
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_rx_timeout(dmx_context[dmx_num].dev, intr_conf->rx_timeout_thresh);
  dmx_hal_set_rxfifo_full_thr(dmx_context[dmx_num].dev, intr_conf->rxfifo_full_thresh);
  dmx_hal_set_txfifo_empty_thr(dmx_context[dmx_num].dev, intr_conf->txfifo_empty_intr_thresh);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_rx_full_threshold(dmx_port_t dmx_num, int threshold) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(threshold < DMX_RXFIFO_FULL_THRHD_V && threshold > 0, "rx fifo full threshold value error", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_hal_get_intr_ena_status(dmx_context[dmx_num].dev) & UART_INTR_RXFIFO_FULL) {
    dmx_hal_set_rxfifo_full_thr(dmx_context[dmx_num].dev, threshold);
  }
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}
esp_err_t dmx_set_tx_empty_threshold(dmx_port_t dmx_num, int threshold) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(threshold < DMX_TXFIFO_EMPTY_THRHD_V && threshold > 0, "tx fifo empty threshold value error", ESP_ERR_INVALID_ARG);
  
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_hal_get_intr_ena_status(dmx_context[dmx_num].dev) & UART_INTR_TXFIFO_EMPTY) {
    dmx_hal_set_txfifo_empty_thr(dmx_context[dmx_num].dev, threshold);
  }
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_set_rx_timeout(dmx_port_t dmx_num, uint8_t tout_thresh) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(tout_thresh < 127, "tout_thresh max value is 126", ESP_ERR_INVALID_ARG);

  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_set_rx_timeout(dmx_context[dmx_num].dev, tout_thresh);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

/// Read/Write  ###############################################################

esp_err_t dmx_read_packet(dmx_port_t dmx_num, uint8_t *buffer, uint16_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(buffer, "buffer is null", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(size <= p_dmx_obj[dmx_num]->buf_size, "size error", ESP_ERR_INVALID_ARG);

  /* Reads can happen in either DMX_MODE_RX or DMX_MODE_TX. Reads while in 
  DMX_MODE_RX are made from the inactive buffer while the active buffer is 
  being used to collect data from the rx FIFO. Reads in DMX_MODE_TX are made 
  from buffer 0 whilst buffer 1 is used by the driver to write to the tx 
  FIFO. */

  if (size == 0) return ESP_OK;

  if (p_dmx_obj[dmx_num]->mode == DMX_MODE_RX) {
    uint8_t active_buffer;
    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    active_buffer = p_dmx_obj[dmx_num]->buf_idx;
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    memcpy(buffer, p_dmx_obj[dmx_num]->buffer[!active_buffer], size);
  } else { // mode == DMX_MODE_TX
    memcpy(buffer, p_dmx_obj[dmx_num]->buffer[0], size);
  }

  return ESP_OK;
}

esp_err_t dmx_read_slot(dmx_port_t dmx_num, int slot_idx, uint8_t *value) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(slot_idx >= 0 && slot_idx < p_dmx_obj[dmx_num]->buf_size, "slot_idx error", ESP_ERR_INVALID_ARG);

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
    *value = p_dmx_obj[dmx_num]->buffer[!active_buffer][slot_idx];
  } else { // mode == DMX_MODE_TX
    *value = p_dmx_obj[dmx_num]->buffer[0][slot_idx];
  }

  return ESP_OK;
}

esp_err_t dmx_write_packet(dmx_port_t dmx_num, const uint8_t *buffer, uint16_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(buffer, "buffer is null", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(size <= p_dmx_obj[dmx_num]->buf_size, "size error", ESP_ERR_INVALID_ARG);

  /* Writes can only happen in DMX_MODE_TX. Writes are made to buffer 0, whilst
  buffer 1 is used by the driver to write to the tx FIFO. */

  if (size == 0) return ESP_OK;

  if (p_dmx_obj[dmx_num]->mode != DMX_MODE_TX) {
    ESP_LOGE(TAG, "cannot write if not in tx mode");
    return ESP_ERR_INVALID_STATE;
  }

  memcpy(p_dmx_obj[dmx_num]->buffer[0], buffer, size);

  return ESP_OK;
}

esp_err_t dmx_write_slot(dmx_port_t dmx_num, int slot_idx, uint8_t value) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(slot_idx >= 0 && slot_idx < p_dmx_obj[dmx_num]->buf_size, "slot_idx error", ESP_ERR_INVALID_ARG);

  /* Writes can only happen in DMX_MODE_TX. Writes are made to buffer 0, whilst
  buffer 1 is used by the driver to write to the tx FIFO. */

  if (p_dmx_obj[dmx_num]->mode != DMX_MODE_TX) {
    ESP_LOGE(TAG, "cannot write if not in tx mode");
    return ESP_ERR_INVALID_STATE;
  }

  p_dmx_obj[dmx_num]->buffer[0][slot_idx] = value;

  return ESP_OK;
}

esp_err_t dmx_tx_packet(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);
  DMX_CHECK(p_dmx_obj[dmx_num]->mode == DMX_MODE_TX, "not in tx mode", ESP_ERR_INVALID_STATE);

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
    DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
    baud_rate = dmx_hal_get_baudrate(dmx_context[dmx_num].dev);
    break_num = dmx_hal_get_break_num(dmx_context[dmx_num].dev);
    idle_num = dmx_hal_get_idle_num(dmx_context[dmx_num].dev);
    DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
    const int brk_us = get_brk_us(baud_rate, break_num);
    const int mab_us = get_mab_us(baud_rate, idle_num);

    // invert the tx line and busy wait...
    dmx_hal_inverse_txd_signal(dmx_context[dmx_num].dev, 1);
    ets_delay_us(brk_us);

    // un-invert the tx line and busy wait...
    dmx_hal_inverse_txd_signal(dmx_context[dmx_num].dev, 0);
    ets_delay_us(mab_us);

    p_dmx_obj[dmx_num]->tx_last_brk_ts = now;
  }

  // write data to tx FIFO
  uint32_t bytes_written;
  dmx_obj_t *const p_dmx = p_dmx_obj[dmx_num];
  const uint32_t len = p_dmx->buf_size - p_dmx->slot_idx;
  const uint8_t *offset = p_dmx->buffer[p_dmx->buf_idx] + p_dmx->slot_idx;
  dmx_hal_write_txfifo(dmx_context[dmx_num].dev, offset, len, &bytes_written);
  p_dmx->slot_idx = bytes_written;

  // enable tx interrupts
  DMX_ENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  dmx_hal_ena_intr_mask(dmx_context[dmx_num].dev, DMX_INTR_TX_ALL);
  DMX_EXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));

  return ESP_OK;
}

esp_err_t dmx_wait_tx_done(dmx_port_t dmx_num, TickType_t ticks_to_wait) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, "dmx_num error", ESP_ERR_INVALID_ARG);
  DMX_CHECK(p_dmx_obj[dmx_num], "driver not installed", ESP_ERR_INVALID_STATE);

  /* Just try to take the "done" semaphore and give it back immediately. */

  if (xSemaphoreTake(p_dmx_obj[dmx_num]->tx_done_sem, ticks_to_wait) == pdFALSE)
    return ESP_ERR_TIMEOUT;
  xSemaphoreGive(p_dmx_obj[dmx_num]->tx_done_sem);

  return ESP_OK;
}
