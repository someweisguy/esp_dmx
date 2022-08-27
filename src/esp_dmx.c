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
#include "rdm_tools.h"

#define DMX_UART_FULL_DEFAULT  (1)  // The default value for the RX FIFO full interrupt threshold.
#define DMX_UART_EMPTY_DEFAULT (8)  // The default value for the TX FIFO empty interrupt threshold.

#define DMX_FUNCTION_NOT_SUPPORTED()                         \
  ESP_LOGE(TAG, "%s() is not supported on %s", __FUNCTION__, \
           CONFIG_IDF_TARGET);                               \
  return ESP_ERR_NOT_SUPPORTED;

static const char *TAG = "dmx";  // The log tagline for the file.

enum rdm_packet_spacing {
  RDM_DISCOVERY_NO_RESPONSE_PACKET_SPACING = 5800,  // The amount of time that must pass between the RDM controller sending a discovery packet and then sending any other packet.
  RDM_REQUEST_NO_RESPONSE_PACKET_SPACING = 3000,    // The amount of time that must pass between the RDM controller sending a request, considering the response lost, and sending any other packet.
  RDM_BROADCAST_PACKET_SPACING = 176,               // The amount of time that must pass between the RDM controller sending a broadcast packet and then sending any other packet.
  RDM_RESPOND_TO_REQUEST_PACKET_SPACING = 176,      // The amount of time that must pass between the RDM controller sending a request and the RDM responder sending a response packet.

  RDM_RESPONSE_LOST_TIMEOUT = 2800,         // The amount of time that must pass before the RDM controller considers a discovery response packet lost. 
};

#define DMX_CONTEXT_INIT(uart_num)                                 \
  {                                                                \
    .hal.dev = UART_LL_GET_HW(uart_num),                           \
    .spinlock = portMUX_INITIALIZER_UNLOCKED, .hw_enabled = false, \
  }

DRAM_ATTR dmx_context_t dmx_context[DMX_NUM_MAX] = {
    DMX_CONTEXT_INIT(DMX_NUM_0),
    DMX_CONTEXT_INIT(DMX_NUM_1),
#if DMX_NUM_MAX > 2
    DMX_CONTEXT_INIT(DMX_NUM_2),
#endif
};

DRAM_ATTR dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {0};

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

esp_err_t dmx_driver_install(dmx_port_t dmx_num, dmx_config_t *dmx_config) {

  dmx_context_t *const hardware = &dmx_context[dmx_num];
  dmx_driver_t *driver;

  // Configure the UART hardware
  dmx_module_enable(dmx_num);
  dmx_hal_init(&hardware->hal);

  // Flush the hardware FIFOs
  dmx_hal_rxfifo_rst(&hardware->hal);
  dmx_hal_txfifo_rst(&hardware->hal);

  // Allocate the DMX driver dynamically
  driver = heap_caps_malloc(sizeof(dmx_driver_t), MALLOC_CAP_32BIT);
  if (driver == NULL) {
    ESP_LOGE(TAG, "DMX driver malloc error");
    return ESP_ERR_NO_MEM;
  }
  dmx_driver[dmx_num] = driver;

  // Buffer must be allocated in unaligned memory
  driver->data.buffer = heap_caps_malloc(DMX_PACKET_SIZE, MALLOC_CAP_8BIT);
  if (driver->data.buffer == NULL) {
    ESP_LOGE(TAG, "DMX driver buffer malloc error");
    dmx_driver_delete(dmx_num);
    return ESP_ERR_NO_MEM;
  }

  // Allocate semaphore
  driver->mux = xSemaphoreCreateRecursiveMutex();
  if (driver->mux == NULL) {
    ESP_LOGE(TAG, "DMX driver mutex malloc error");
    dmx_driver_delete(dmx_num);
    return ESP_ERR_NO_MEM;
  }

  // Initialize driver state
  driver->dmx_num = dmx_num;
  driver->timer_group = dmx_config->timer_group;
  driver->timer_num = dmx_config->timer_num;
  driver->task_waiting = NULL;

  // Initialize driver flags
  driver->is_in_break = false;
  driver->received_packet = false;
  driver->is_sending = false;
  driver->timer_running = false;

  // Initialize the driver buffer
  bzero(driver->data.buffer, DMX_MAX_PACKET_SIZE);
  driver->data.previous_type = RDM_NON_RDM_PACKET;
  driver->data.rx_size = DMX_MAX_PACKET_SIZE;
  driver->data.sent_previous = false;
  driver->data.previous_uid = 0;
  driver->data.previous_ts = 0;
  driver->data.head = DMX_MAX_PACKET_SIZE;  // Don't read before a DMX break

  // Initialize DMX transmit settings
  driver->break_len = RDM_BREAK_LEN_US;
  driver->mab_len = RDM_MAB_LEN_US;

  // Initialize sniffer in the disabled state
  driver->sniffer.queue = NULL;

  // Driver ISR is in IRAM so interrupt flags must include IRAM flag
  if (!(dmx_config->intr_alloc_flags & ESP_INTR_FLAG_IRAM)) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    dmx_config->intr_alloc_flags |= ESP_INTR_FLAG_IRAM;
  }

  // Install UART interrupt
  dmx_hal_disable_interrupt(&hardware->hal, DMX_ALL_INTR_MASK);
  dmx_hal_clear_interrupt(&hardware->hal, DMX_ALL_INTR_MASK);
  dmx_hal_set_txfifo_empty(&hardware->hal, DMX_UART_EMPTY_DEFAULT);
  dmx_hal_set_rxfifo_full(&hardware->hal, DMX_UART_FULL_DEFAULT);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, dmx_config->intr_alloc_flags,
                 &dmx_uart_isr, driver, &driver->uart_isr_handle);

  // Install hardware timer interrupt if specified by the user
  if (driver->timer_group != -1) {
    const timer_config_t timer_config = {
        .divider = 80,  // (80MHz / 80) == 1MHz resolution timer
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = false,
        .alarm_en = true,
        .auto_reload = true,
    };
    timer_init(dmx_config->timer_group, dmx_config->timer_num, &timer_config);
    timer_isr_callback_add(dmx_config->timer_group, dmx_config->timer_num,
                           dmx_timer_isr, driver, dmx_config->intr_alloc_flags);
    timer_enable_intr(dmx_config->timer_group, dmx_config->timer_num);
  }

  // Enable UART read interrupt and set RTS low
  taskENTER_CRITICAL(&hardware->spinlock);
  dmx_hal_enable_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
  dmx_hal_set_rts(&hardware->hal, 1);
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Give the mutex and return
  xSemaphoreGiveRecursive(driver->mux);
  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {
  // TODO: check args

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Free driver mutex
  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return ESP_FAIL;
  }
  xSemaphoreGiveRecursive(driver->mux);
  vSemaphoreDelete(driver->mux);

  // Uninstall UART ISR
  if (driver->uart_isr_handle != NULL) {
    esp_intr_free(driver->uart_isr_handle);
  }

  // Uninstall sniffer ISR
  if (dmx_is_sniffer_enabled(dmx_num)) {
    dmx_sniffer_disable(dmx_num);
  }

  // Free driver data buffer
  if (driver->data.buffer != NULL) {
    heap_caps_free(driver->data.buffer);
  }

  // Free hardware timer ISR
  if (driver->timer_group != -1) {
    timer_deinit(driver->timer_group, driver->timer_num);
  }

  // Free driver
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;

  // Disable UART module
  dmx_module_disable(dmx_num);

  return ESP_OK;
}

bool dmx_driver_is_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && dmx_driver[dmx_num] != NULL;
}

esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_num, int rx_num, int rts_num) {
  ESP_RETURN_ON_FALSE(dmx_num >= 0 && dmx_num < DMX_NUM_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "dmx_num error");
  ESP_RETURN_ON_FALSE(tx_num < 0 || GPIO_IS_VALID_OUTPUT_GPIO(tx_num),
                      ESP_ERR_INVALID_ARG, TAG, "tx_num error");
  ESP_RETURN_ON_FALSE(rx_num < 0 || GPIO_IS_VALID_GPIO(rx_num),
                      ESP_ERR_INVALID_ARG, TAG, "rx_num error");
  ESP_RETURN_ON_FALSE(rts_num < 0 || GPIO_IS_VALID_OUTPUT_GPIO(rts_num),
                      ESP_ERR_INVALID_ARG, TAG, "rts_num error");

  return uart_set_pin(dmx_num, tx_num, rx_num, rts_num, DMX_PIN_NO_CHANGE);
}

esp_err_t dmx_sniffer_enable(dmx_port_t dmx_num, int intr_num) {
  // TODO: Check args

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  
  // Add the GPIO interrupt handler
  esp_err_t err = gpio_isr_handler_add(intr_num, dmx_gpio_isr, driver);
  if (err) {
    return err;
  }
  driver->sniffer.intr_io_num = intr_num;

  // TODO: Initialize the sniffer queue

  // Indicate that a negative edge has not yet been seen by the sniffer
  driver->sniffer.last_neg_edge_ts = -1;

  // Enable the interrupt
  gpio_set_intr_type(intr_num, GPIO_INTR_ANYEDGE);

  return ESP_OK;
}

esp_err_t dmx_sniffer_disable(dmx_port_t dmx_num) {
  // TODO: check args

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const hardware = &dmx_context[dmx_num];
  
  // Disable the interrupt and remove the interrupt handler
  taskENTER_CRITICAL(&hardware->spinlock);
  const int intr_io_num = driver->sniffer.intr_io_num;
  taskEXIT_CRITICAL(&hardware->spinlock);
  gpio_set_intr_type(intr_io_num, GPIO_INTR_DISABLE);
  esp_err_t err = gpio_isr_handler_remove(intr_io_num);
  if (err) {
    return err;
  }
  driver->sniffer.intr_io_num = -1;

  // TODO: Uninitialize the sniffer queue

  return ESP_OK;
}

bool dmx_sniffer_is_enabled(dmx_port_t dmx_num) {
  return dmx_driver_is_installed(dmx_num) &&
         dmx_driver[dmx_num]->sniffer.queue != NULL;
}

size_t dmx_set_baud_rate(dmx_port_t dmx_num, size_t baud_rate) {
  // TODO: check args
  
  // Clamp the baud rate to within DMX specification
  if (baud_rate < DMX_MIN_BAUD_RATE) {
    baud_rate = DMX_MIN_BAUD_RATE;
  } else if (baud_rate > DMX_MAX_BAUD_RATE) {
    baud_rate = DMX_MAX_BAUD_RATE;
  }

  dmx_context_t *const hardware = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&hardware->spinlock);
  dmx_hal_set_baud_rate(&hardware->hal, baud_rate);
  taskEXIT_CRITICAL(&hardware->spinlock);

  return baud_rate;
}

size_t dmx_get_baud_rate(dmx_port_t dmx_num) {
  // TODO: Check args

  dmx_context_t *const hardware = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&hardware->spinlock);
  const size_t baud_rate = dmx_hal_get_baud_rate(&hardware->hal);
  taskEXIT_CRITICAL(&hardware->spinlock);

  return baud_rate;
}

size_t dmx_set_break_len(dmx_port_t dmx_num, size_t break_len) {
  // TODO: check args
  
  // Clamp the break length to within DMX specification
  if (break_len < DMX_MIN_BREAK_LEN_US) {
    break_len = DMX_MIN_BREAK_LEN_US;
  } else if (break_len > DMX_MAX_BREAK_LEN_US) {
    break_len = DMX_MAX_BREAK_LEN_US;
  }

  dmx_context_t *const hardware = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&hardware->spinlock);
  dmx_driver[dmx_num]->break_len = break_len;
  taskEXIT_CRITICAL(&hardware->spinlock);

  return break_len;
}

size_t dmx_get_break_len(dmx_port_t dmx_num) {
  // TODO: check args

  dmx_context_t *const hardware = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&hardware->spinlock);
  const size_t break_len = dmx_driver[dmx_num]->break_len;
  taskEXIT_CRITICAL(&hardware->spinlock);

  return break_len;
}

size_t dmx_set_mab_len(dmx_port_t dmx_num, size_t mab_len) {
  // TODO: check args
  
  // Clamp the mark-after-break length to within DMX specification
  if (mab_len < DMX_MIN_MAB_LEN_US) {
    mab_len = DMX_MIN_MAB_LEN_US;
  } else if (mab_len > DMX_MAX_MAB_LEN_US) {
    mab_len = DMX_MAX_MAB_LEN_US;
  }

  dmx_context_t *const hardware = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&hardware->spinlock);
  dmx_driver[dmx_num]->mab_len = mab_len;
  taskEXIT_CRITICAL(&hardware->spinlock);

  return mab_len;
}

size_t dmx_get_mab_len(dmx_port_t dmx_num) {
  // TODO: check args

  dmx_context_t *const hardware = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&hardware->spinlock);
  const size_t mab_len = dmx_driver[dmx_num]->mab_len;
  taskEXIT_CRITICAL(&hardware->spinlock);

  return mab_len;
}

size_t dmx_read(dmx_port_t dmx_num, void *destination, size_t size) {
  // TODO: Check arguments
  
  // Clamp size to the maximum DMX packet size
  if (size > DMX_MAX_PACKET_SIZE) {
    size = DMX_MAX_PACKET_SIZE;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Copy data from the driver buffer to the destination asynchronously
  memcpy(destination, driver->data.buffer, size);

  return size;
}

size_t dmx_write(dmx_port_t dmx_num, const void *source, size_t size) {
  // TODO: check args
  
  // Clamp size to the maximum DMX packet size or fail quickly on invalid size
  if (size > DMX_MAX_PACKET_SIZE) {
    size = DMX_MAX_PACKET_SIZE;
  } else if (size == 0) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const hardware = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&hardware->spinlock);
  if (driver->is_sending && driver->data.previous_type != RDM_NON_RDM_PACKET) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(&hardware->spinlock);
    return 0;
  } else if (dmx_hal_get_rts(&hardware->hal) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_hal_disable_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
    dmx_hal_set_rts(&hardware->hal, 0);
  }
  driver->data.tx_size = size;  // Update driver transmit size
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Copy data from the source to the driver buffer asynchronously
  memcpy(driver->data.buffer, source, size);

  return size;
}

size_t dmx_receive(dmx_port_t dmx_num, dmx_event_t *event, TickType_t timeout) {
  // TODO: Check arguments

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const hardware = &dmx_context[dmx_num];

  // Block until the mutex can be taken
  if (!xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY)) {
    return 0;
  }

  // Block until the driver is done sending
  if (!dmx_wait_sent(dmx_num, portMAX_DELAY)) {
    return 0;
  }

  // Set the RTS pin to read from the DMX bus
  taskENTER_CRITICAL(&hardware->spinlock);
  if (dmx_hal_get_rts(&hardware->hal) == 0) {
    dmx_hal_disable_interrupt(&hardware->hal, DMX_INTR_TX_ALL);
    dmx_hal_set_rts(&hardware->hal, 1);
    dmx_hal_enable_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
  }
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Receive the latest data packet or check if the driver must wait
  dmx_err_t err = DMX_OK;
  uint32_t packet_size = 0;
  taskENTER_CRITICAL(&hardware->spinlock);
  if (!driver->received_packet) {
    driver->task_waiting = xTaskGetCurrentTaskHandle();
  } else {
    // A packet has already been received
    packet_size = driver->data.head;
    err = driver->data.err;
  }
  const bool received_packet = driver->received_packet;
  const bool sent_previous = driver->data.sent_previous;
  const int previous_type = driver->data.previous_type;
  const uint64_t previous_uid = driver->data.previous_uid;
  const int64_t previous_ts = driver->data.previous_ts;
  taskEXIT_CRITICAL(&hardware->spinlock);

  // If a packet hasn't been received, the driver must wait
  if (!received_packet) {
    // Determine if a fail-quick timeout must be set
    uint32_t timeout = 0;
    if (sent_previous && previous_uid != RDM_BROADCAST_UID &&
        (previous_type == RDM_GET_COMMAND || previous_type == RDM_SET_COMMAND ||
         previous_type == RDM_DISCOVERY_COMMAND)) {
      timeout = RDM_RESPONSE_LOST_TIMEOUT;
    }

    // Set the timeout alarm if the timeout hasn't elapsed yet
    taskENTER_CRITICAL(&hardware->spinlock);
    const int64_t elapsed = esp_timer_get_time() - previous_ts;
    if (elapsed < timeout) {
      timer_set_counter_value(driver->timer_group, driver->timer_num, elapsed);
      timer_set_alarm_value(driver->timer_group, driver->timer_num, timeout);
      timer_start(driver->timer_group, driver->timer_num);
      driver->timer_running = true;
    }
    taskEXIT_CRITICAL(&hardware->spinlock);

    // Fail immediately if the timeout has elapsed and a response was expected
    if (timeout > 0 && elapsed >= timeout) {
      taskENTER_CRITICAL(&hardware->spinlock);
      driver->task_waiting = NULL;
      taskEXIT_CRITICAL(&hardware->spinlock);
      xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
      xSemaphoreGiveRecursive(driver->mux);
      return 0;
    }

    // Wait for a task notification
    xTaskNotifyWait(0, ULONG_MAX, &packet_size, timeout);
    driver->task_waiting = NULL;
    err = driver->data.err;
  }

  // Process DMX packet data
  if (packet_size > 0 && err == DMX_OK && event != NULL) {
    taskENTER_CRITICAL(&hardware->spinlock);
    bool is_rdm = rdm_parse(driver->data.buffer, packet_size, &event->rdm);
    event->sc = driver->data.buffer[0];
    taskEXIT_CRITICAL(&hardware->spinlock);
    event->size = packet_size;
    event->is_rdm = is_rdm;
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return packet_size;
}

size_t dmx_send(dmx_port_t dmx_num) {
  // TODO: Check arguments
  
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const hardware = &dmx_context[dmx_num];

  // Block until the mutex can be taken
  if (!xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY)) {
    return 0;
  }

  // Block until the driver is done sending
  if (!dmx_wait_sent(dmx_num, portMAX_DELAY)) {
    return 0;
  }

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
    timer_set_counter_value(driver->timer_group, driver->timer_num, elapsed);
    timer_set_alarm_value(driver->timer_group, driver->timer_num, timeout);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
    timer_start(driver->timer_group, driver->timer_num);
    driver->timer_running = true;
  }
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Block if an alarm was set
  if (elapsed < timeout) {
    bool notified = xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    if (!notified) {
      timer_pause(driver->timer_group, driver->timer_num);
      driver->timer_running = false;
      xTaskNotifyStateClear(driver->task_waiting);
    }
    driver->task_waiting = NULL;
    if (!notified) {
      xSemaphoreGiveRecursive(driver->mux);
      return 0;
    }
  }

  // Turn the DMX bus around and get the send size
  taskENTER_CRITICAL(&hardware->spinlock);
  if (dmx_hal_get_rts(&hardware->hal) == 1) {
    dmx_hal_disable_interrupt(&hardware->hal, DMX_INTR_RX_ALL);
    dmx_hal_set_rts(&hardware->hal, 0);
  }
  const size_t size = driver->data.tx_size;
  taskEXIT_CRITICAL(&hardware->spinlock);

  // Record the outgoing packet type
  const uint8_t sc = driver->data.buffer[0];  // DMX start code.
  if (sc == RDM_SC && driver->data.buffer[1] == RDM_SUB_SC) {
    const rdm_data_t *rdm = (rdm_data_t *)driver->data.buffer;
    driver->data.previous_type = rdm->cc;
    driver->data.previous_uid = buf_to_uid(rdm->destination_uid);
  } else if (sc == RDM_PREAMBLE || sc == RDM_DELIMITER) {
    driver->data.previous_type = RDM_DISCOVERY_COMMAND_RESPONSE;
    driver->data.previous_uid = RDM_BROADCAST_UID;
  } else {
    driver->data.previous_type = RDM_NON_RDM_PACKET;
    driver->data.previous_uid = 0;
  }
  driver->data.sent_previous = true;
  
  // Determine if a DMX break is required and send the packet
  if (driver->data.previous_type == RDM_DISCOVERY_COMMAND_RESPONSE) {
    // RDM discovery responses do not send a DMX break - write immediately
    taskENTER_CRITICAL(&hardware->spinlock);
    driver->is_sending = true;

    size_t write_size = driver->data.tx_size;
    dmx_hal_write_txfifo(&hardware->hal, driver->data.buffer, &write_size);
    driver->data.head = write_size;
    
    // Enable DMX write interrupts
    dmx_hal_enable_interrupt(&hardware->hal, DMX_INTR_TX_ALL);
    taskEXIT_CRITICAL(&hardware->spinlock);
  } else {
    // Use the hardware timer to send a DMX break and mark-after-break
    taskENTER_CRITICAL(&hardware->spinlock);
    const uint32_t break_len = driver->break_len;
    taskEXIT_CRITICAL(&hardware->spinlock);
    timer_set_counter_value(driver->timer_group, driver->timer_num, 0);
    timer_set_alarm_value(driver->timer_group, driver->timer_num, break_len);
    
    // Send the packet by starting the DMX break
    taskENTER_CRITICAL(&hardware->spinlock);
    driver->is_sending = true;
    driver->is_in_break = true;
    driver->data.head = 0;
    dmx_hal_invert_tx(&hardware->hal, 1);
    timer_start(driver->timer_group, driver->timer_num);
    driver->timer_running = true;
    taskEXIT_CRITICAL(&hardware->spinlock);
  }

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return size;
}

bool dmx_wait_sent(dmx_port_t dmx_num, TickType_t timeout) {
  // TODO: Check arguments
  
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const hardware = &dmx_context[dmx_num];

  // Block until the mutex can be taken
  if (!xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY)) {
    return false;
  }

  // Determine if the task needs to block
  bool result = true;
  if (timeout > 0) {
    bool task_waiting = false;
    taskENTER_CRITICAL(&hardware->spinlock);
    if (driver->is_sending) {
      driver->task_waiting = xTaskGetCurrentTaskHandle();
      task_waiting = true;
    }
    taskEXIT_CRITICAL(&hardware->spinlock);

    // Wait for a notification that the driver is done sending
    if (task_waiting) {
      result = xTaskNotifyWait(0, ULONG_MAX, NULL, timeout);
      driver->task_waiting = NULL;
    }
  } else {
    taskENTER_CRITICAL(&hardware->spinlock);
    if (driver->is_sending) {
      result = false;
    }
    taskEXIT_CRITICAL(&hardware->spinlock);
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return result;
}
