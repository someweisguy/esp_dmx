#include "esp_dmx.h"

#include <string.h>

#include "dmx_types.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "endian.h"
#include "esp_check.h"
#include "esp_log.h"
#include "impl/dmx_hal.h"
#include "impl/driver.h"
#include "impl/intr_handlers.h"

enum {
  DMX_UART_FULL_DEFAULT = 1,   // The default value for the RX FIFO full interrupt threshold.
  DMX_UART_EMPTY_DEFAULT = 8,  // The default value for the TX FIFO empty interrupt threshold.

  DMX_MEMORY_CAPABILITIES = (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
};

#define DMX_FUNCTION_NOT_SUPPORTED()                         \
  ESP_LOGE(TAG, "%s() is not supported on %s", __FUNCTION__, \
           CONFIG_IDF_TARGET);                               \
  return ESP_ERR_NOT_SUPPORTED;

static const char *TAG = "dmx";  // The log tagline for the file.

static void dmx_module_enable(dmx_port_t dmx_num) {
  taskENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != true) {
    periph_module_enable(uart_periph_signal[dmx_num].module);
    if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
#if SOC_UART_REQUIRE_CORE_RESET
      /* Workaround for ESP32C3: enable core reset before enabling UART module
      clock to prevent UART output garbage value. */
      uart_hal_set_reset_core(&(dmx_context[dmx_num].hal), true);
      periph_module_reset(uart_periph_signal[dmx_num].module);
      uart_hal_set_reset_core(&(dmx_context[dmx_num].hal), false);
#else
      periph_module_reset(uart_periph_signal[dmx_num].module);
#endif
    }
    dmx_context[dmx_num].hw_enabled = true;
  }
  taskEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
}

static void dmx_module_disable(dmx_port_t dmx_num) {
  taskENTER_CRITICAL(&(dmx_context[dmx_num].spinlock));
  if (dmx_context[dmx_num].hw_enabled != false) {
    if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
      periph_module_disable(uart_periph_signal[dmx_num].module);
    }
    dmx_context[dmx_num].hw_enabled = false;
  }
  taskEXIT_CRITICAL(&(dmx_context[dmx_num].spinlock));
}

/// Driver Functions  #########################################################
esp_err_t dmx_driver_install(dmx_port_t dmx_num, dmx_config_t *dmx_config) {

  dmx_context_t *const hardware = &dmx_context[dmx_num];
  dmx_driver_t *driver;

  // Initialize the DMX UID
  if (dmx_uid == 0) {
    // TODO: initialize the UID based on the device MAC address
    dmx_uid = 0x1234567890ab;
  }

  // Configure the UART hardware
  dmx_module_enable(dmx_num);
  dmx_hal_init(&hardware->hal);

  // Flush the hardware FIFOs
  dmx_hal_rxfifo_rst(&hardware->hal);
  dmx_hal_txfifo_rst(&hardware->hal);

  // Allocate the DMX driver dynamically
  // TODO: can we use a default malloc here? 
  driver = heap_caps_malloc(sizeof(dmx_driver_t), DMX_MEMORY_CAPABILITIES);
  if (driver == NULL) {
    ESP_LOGE(TAG, "DMX driver malloc error");
    return ESP_ERR_NO_MEM;
  }
  dmx_driver[dmx_num] = driver;

  // Allocate semaphore dynamically
  driver->mux = xSemaphoreCreateRecursiveMutex();
  if (driver->mux == NULL) {
    ESP_LOGE(TAG, "DMX driver mutex malloc error");
    return ESP_ERR_NO_MEM;
  }
  xSemaphoreGiveRecursive(driver->mux);

  // Initialize the driver buffer
  bzero(driver->data.buffer, DMX_MAX_PACKET_SIZE);
  driver->data.size = DMX_MAX_PACKET_SIZE;
  driver->data.head = 0;
  driver->data.previous_type = DMX_UNKNOWN_PACKET;
  driver->data.previous_ts = 0;
  driver->data.sent_previous = false;

  driver->mode = DMX_MODE_READ;
  driver->is_in_break = false;
  driver->is_receiving = false;
  driver->is_sending = false;

  // Initialize driver state
  driver->dmx_num = dmx_num;
  driver->rst_seq_hw = dmx_config->rst_seq_hw;
  driver->timer_idx = dmx_config->timer_idx;
  driver->task_waiting = NULL;

  // TODO: reorganize these inits
  // driver->tx.rdm_tn = 0;

  // Initialize TX settings
  driver->break_len = DMX_BREAK_LEN_US;
  driver->mab_len = DMX_WRITE_MIN_MAB_LEN_US;

  // Initialize sniffer in the disabled state
  driver->sniffer.intr_io_num = -1;

  // Driver ISR is in IRAM so interrupt flags must include IRAM flag
  if (!(dmx_config->intr_alloc_flags & ESP_INTR_FLAG_IRAM)) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    dmx_config->intr_alloc_flags |= ESP_INTR_FLAG_IRAM;
  }

  // Install UART interrupt
  dmx_hal_disable_interrupt(&hardware->hal, DMX_ALL_INTR_MASK);
  dmx_hal_clear_interrupt(&hardware->hal, DMX_ALL_INTR_MASK);
  dmx_hal_set_txfifo_empty_threshold(&hardware->hal, DMX_UART_EMPTY_DEFAULT);
  dmx_hal_set_rxfifo_full_threshold(&hardware->hal, DMX_UART_FULL_DEFAULT);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, dmx_config->intr_alloc_flags,
                 &dmx_uart_isr, driver, &driver->uart_isr_handle);

  // Install hardware timer interrupt if specified by the user
  if (driver->rst_seq_hw != -1) {
    const timer_config_t timer_config = {
        .divider = 80,  // (80MHz / 80) == 1MHz resolution timer
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = false,
        .alarm_en = true,
        .auto_reload = true,
    };
    timer_init(dmx_config->rst_seq_hw, dmx_config->timer_idx, &timer_config);
    timer_isr_callback_add(dmx_config->rst_seq_hw, dmx_config->timer_idx,
                           dmx_timer_isr, driver, dmx_config->intr_alloc_flags);
    timer_enable_intr(dmx_config->rst_seq_hw, dmx_config->timer_idx);
  }

  // Enable UART read interrupt and set RTS low
  taskENTER_CRITICAL(&hardware->spinlock);
  dmx_hal_enable_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
  dmx_hal_set_rts(&hardware->hal, DMX_MODE_READ);
  taskEXIT_CRITICAL(&hardware->spinlock);

  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {

  return ESP_OK;
}

bool dmx_is_driver_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && dmx_driver[dmx_num] != NULL;
}

esp_err_t dmx_sniffer_enable(dmx_port_t dmx_num, int intr_io_num) {

  return ESP_OK;
}

esp_err_t dmx_sniffer_disable(dmx_port_t dmx_num) {

  return ESP_OK;
}

bool dmx_is_sniffer_enabled(dmx_port_t dmx_num) {
  return dmx_is_driver_installed(dmx_num) &&
         dmx_driver[dmx_num]->sniffer.intr_io_num != -1;
}

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

/// Transmit Configuration  ###################################################
esp_err_t dmx_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  // TODO: check args
  // TODO: check that the new baud_rate is within DMX specification

  dmx_context_t *const hardware = &dmx_context[dmx_num];
  taskENTER_CRITICAL(&hardware->spinlock);
  dmx_hal_set_baud_rate(&hardware->hal, baud_rate);
  taskEXIT_CRITICAL(&hardware->spinlock);

  return ESP_OK;
}

esp_err_t dmx_get_baud_rate(dmx_port_t dmx_num, uint32_t *baud_rate) {
  // TODO: Check args

  dmx_context_t *const hardware = &dmx_context[dmx_num];
  taskENTER_CRITICAL(&hardware->spinlock);
  *baud_rate = dmx_hal_get_baud_rate(&hardware->hal);
  taskEXIT_CRITICAL(&hardware->spinlock);

  return ESP_OK;
}

esp_err_t dmx_set_break_len(dmx_port_t dmx_num, uint32_t break_len) {
  // TODO: check args
  // TODO: ensure break_len is within DMX spec

  dmx_context_t *const hardware = &dmx_context[dmx_num];
  taskENTER_CRITICAL(&hardware->spinlock);
  dmx_driver[dmx_num]->break_len = break_len;
  taskEXIT_CRITICAL(&hardware->spinlock);

  return ESP_OK;
}

esp_err_t dmx_get_break_len(dmx_port_t dmx_num, uint32_t *break_len) {
  // TODO: check args

  dmx_context_t *const hardware = &dmx_context[dmx_num];
  taskENTER_CRITICAL(&hardware->spinlock);
  *break_len = dmx_driver[dmx_num]->break_len;
  taskEXIT_CRITICAL(&hardware->spinlock);

  return ESP_OK;
}

esp_err_t dmx_set_mab_len(dmx_port_t dmx_num, uint32_t mab_len) {
  // TODO: check args
  // TODO: ensure mab_len is within DMX spec

  dmx_context_t *const hardware = &dmx_context[dmx_num];
  taskENTER_CRITICAL(&hardware->spinlock);
  dmx_driver[dmx_num]->mab_len = mab_len;
  taskEXIT_CRITICAL(&hardware->spinlock);

  return ESP_OK;
}

esp_err_t dmx_get_mab_len(dmx_port_t dmx_num, uint32_t *mab_len) {
  // TODO: check args

  dmx_context_t *const hardware = &dmx_context[dmx_num];
  taskENTER_CRITICAL(&hardware->spinlock);
  *mab_len = dmx_driver[dmx_num]->mab_len;
  taskEXIT_CRITICAL(&hardware->spinlock);

  return ESP_OK;
}

/// Read/Write  ###############################################################
esp_err_t dmx_read(dmx_port_t dmx_num, void *buffer, size_t size) {
  // TODO: Check arguments

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Copy data from the driver buffer to the destination asynchronously
  memcpy(buffer, driver->data.buffer, size);

  return ESP_OK;
}

esp_err_t dmx_write(dmx_port_t dmx_num, const void *buffer, size_t size) {
  // TODO: check args

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // TODO: if packet being sent is RDM, don't let a write happen asynchronously
  // Copy data from the source to the driver buffer asynchronously
  memcpy(driver->data.buffer, buffer, size);

  return ESP_OK;
}

esp_err_t dmx_read_slot(dmx_port_t dmx_num, size_t index, uint8_t *value) {

  return ESP_OK;
}

esp_err_t dmx_write_slot(dmx_port_t dmx_num, size_t index,
                         const uint8_t value) {
  return ESP_OK;
}

esp_err_t dmx_send(dmx_port_t dmx_num, size_t size, TickType_t ticks_to_wait) {
  // TODO: Check arguments
  
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const hardware = &dmx_context[dmx_num];

  // Ensure the driver is not sending
  taskENTER_CRITICAL(&hardware->spinlock);
  if (driver->is_sending) {
    taskEXIT_CRITICAL(&hardware->spinlock);
    return ESP_FAIL;
  }
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Block until the mutex can be taken and decrement block time accordingly
  const TickType_t start_tick = xTaskGetTickCount();
  if (!xSemaphoreTakeRecursive(driver->mux, ticks_to_wait)) {
    return ESP_ERR_TIMEOUT;
  }
  ticks_to_wait -= xTaskGetTickCount() - start_tick;

  // Determine if an alarm needs to be set to wait until driver is ready
  uint32_t timeout = 0;
  taskENTER_CRITICAL(&hardware->spinlock);
  if (driver->data.sent_previous) {
    if (driver->data.previous_type == RDM_DISCOVERY_COMMAND) {
      timeout = RDM_DISCOVERY_NO_RESPONSE_PACKET_SPACING;
    } else if (driver->data.previous_uid != RDM_BROADCAST_UID) {
      timeout = RDM_REQUEST_NO_RESPONSE_PACKET_SPACING;
    } else {
      timeout = RDM_BROADCAST_PACKET_SPACING;
    }
  } else {
    timeout = RDM_RESPOND_TO_REQUEST_PACKET_SPACING;
  }
  const int64_t elapsed = esp_timer_get_time() - driver->data.previous_ts;
  if (elapsed < timeout) {
    timer_set_counter_value(driver->rst_seq_hw, driver->timer_idx, elapsed);
    timer_set_alarm_value(driver->rst_seq_hw, driver->timer_idx, timeout);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
    timer_start(driver->rst_seq_hw, driver->timer_idx);
  }

  // Turn the DMX bus around
  if (driver->mode == DMX_MODE_READ) {
    dmx_hal_disable_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
    dmx_hal_set_rts(&hardware->hal, DMX_MODE_WRITE);
    driver->mode = DMX_MODE_WRITE;
  }
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Block if an alarm was set
  if (elapsed < timeout) {
    bool notified = xTaskNotifyWait(0, ULONG_MAX, NULL, ticks_to_wait);
    if (!notified) {
      timer_pause(driver->rst_seq_hw, driver->timer_idx);
      xTaskNotifyStateClear(driver->task_waiting);
    }
    driver->task_waiting = NULL;
    if (!notified) {
      xSemaphoreGiveRecursive(driver->mux);
      return ESP_ERR_TIMEOUT;
    }
  }

  // Record the outgoing packet type
  const uint8_t sc = driver->data.buffer[0];  // DMX start code.
  if (sc == DMX_SC) {
    driver->data.previous_type = DMX_DIMMER_PACKET;
    driver->data.previous_uid = -1;  // No destination UID
  } else if (sc == RDM_SC) {
    const rdm_packet_t *rdm = (rdm_packet_t *)driver->data.buffer;
    driver->data.previous_type = rdm->cc;
    driver->data.previous_uid = uidcpy(rdm->destination_uid);
  } else if (sc == RDM_PREAMBLE || sc == RDM_DELIMITER) {
    driver->data.previous_type = RDM_DISCOVERY_COMMAND_RESPONSE;
    driver->data.previous_uid = RDM_BROADCAST_UID;
  } else {
    driver->data.previous_type = DMX_UNKNOWN_PACKET;
    driver->data.previous_uid = -1;  // No destination UID
  }
  driver->data.sent_previous = true;

  // Begin sending the packet
  taskENTER_CRITICAL(&hardware->spinlock);
  const uint32_t break_len = driver->break_len;
  taskEXIT_CRITICAL(&hardware->spinlock);
  timer_set_counter_value(driver->rst_seq_hw, driver->timer_idx, 0);
  timer_set_alarm_value(driver->rst_seq_hw, driver->timer_idx, break_len);
  taskENTER_CRITICAL(&hardware->spinlock);
  driver->is_sending = true;
  driver->is_in_break = true;
  driver->data.size = size;
  driver->data.head = 0;
  dmx_hal_invert_signal(&hardware->hal, UART_SIGNAL_TXD_INV);
  timer_start(driver->rst_seq_hw, driver->timer_idx);
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return ESP_OK;
}

esp_err_t dmx_wait_sent(dmx_port_t dmx_num, TickType_t ticks_to_wait) {
  // TODO: Check arguments
  
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const hardware = &dmx_context[dmx_num];

  // Block until the mutex can be taken and decrement timeout accordingly
  const TickType_t start_tick = xTaskGetTickCount();
  if (!xSemaphoreTakeRecursive(driver->mux, ticks_to_wait)) {
    return ESP_ERR_TIMEOUT;
  }
  ticks_to_wait -= xTaskGetTickCount() - start_tick;

  // Determine if the task needs to block
  taskENTER_CRITICAL(&hardware->spinlock);
  if (driver->is_sending) {
    driver->task_waiting = xTaskGetCurrentTaskHandle();
  }
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Wait for a notification that the driver is done sending
  bool result = true;
  if (driver->task_waiting) {
    result = xTaskNotifyWait(0, ULONG_MAX, NULL, ticks_to_wait);
    driver->task_waiting = NULL;
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return result ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t dmx_receive(dmx_port_t dmx_num, dmx_event_t *event,
                      TickType_t ticks_to_wait) {
  // TODO: Check arguments

  // Ensure the driver isn't sending and decrement timeout accordingly
  TickType_t start_tick = xTaskGetTickCount();
  if (dmx_wait_sent(dmx_num, ticks_to_wait) != ESP_OK) {
    return ESP_ERR_TIMEOUT;
  }
  ticks_to_wait -= xTaskGetTickCount() - start_tick;

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const hardware = &dmx_context[dmx_num];

  // Block until the mutex can be taken and decrement timeout accordingly
  start_tick = xTaskGetTickCount();
  if (!xSemaphoreTakeRecursive(driver->mux, ticks_to_wait)) {
    return ESP_ERR_TIMEOUT;
  }
  ticks_to_wait -= xTaskGetTickCount() - start_tick;

  // Turn the DMX bus around
  taskENTER_CRITICAL(&hardware->spinlock);
  if (driver->mode == DMX_MODE_WRITE) {
    dmx_hal_disable_interrupt(&hardware->hal, DMX_INTR_TX_ALL);
    dmx_hal_set_rts(&hardware->hal, DMX_MODE_READ);
    dmx_hal_enable_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
    driver->mode = DMX_MODE_READ;
  }

  // Set an RDM receive timeout to allow driver to fail quickly
  uint32_t timeout = 0;
  if (driver->data.sent_previous &&
      driver->data.previous_type != DMX_DIMMER_PACKET &&
      driver->data.previous_type != DMX_UNKNOWN_PACKET) {
    timeout = RDM_RESPONSE_LOST_TIMEOUT;
  }
  const int64_t elapsed = esp_timer_get_time() - driver->data.previous_ts;
  if (elapsed < timeout) {
    timer_set_counter_value(driver->rst_seq_hw, driver->timer_idx, elapsed);
    timer_set_alarm_value(driver->rst_seq_hw, driver->timer_idx, timeout);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
    timer_start(driver->rst_seq_hw, driver->timer_idx);
  } else {
    // TODO: Handle case where timeout is elapsed?
  }
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Block until a packet is received
  uint32_t packet_size;
  bool notified = xTaskNotifyWait(0, ULONG_MAX, &packet_size, ticks_to_wait);
  if (elapsed < timeout && notified && packet_size > 0) {
    timer_pause(driver->rst_seq_hw, driver->timer_idx);
    xTaskNotifyStateClear(driver->task_waiting);
  }
  driver->task_waiting = NULL;

  // Process DMX packet data
  if (notified && packet_size > 0 && event != NULL) {
    event->size = packet_size;
    // TODO
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return notified && packet_size > 0 ? ESP_OK : ESP_ERR_TIMEOUT;
}

void *memcpyswap(void *dest, const void *src, size_t n) {
  char *chrdest = (char *)dest;
  char *chrsrc = (char *)src + n - 1;
  for (; (void *)chrsrc >= src; --chrsrc, chrdest++) {
    *chrdest = *chrsrc;
  }
  return chrdest;
}
