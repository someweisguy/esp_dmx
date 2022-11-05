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
#include "esp_rdm.h"
#include "private/dmx_context.h"
#include "private/driver.h"
#include "private/rdm_encode_types.h"
#include "rdm_constants.h"
#include "rdm_types.h"

// Used for argument checking at the beginning of each function.
#define DMX_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

// Initializes the DMX context.
#define DMX_CONTEXT_INIT(uart_num)                                             \
  {                                                                            \
    .dev = UART_LL_GET_HW(uart_num), .spinlock = portMUX_INITIALIZER_UNLOCKED, \
    .hw_enabled = false,                                                       \
  }

#ifdef CONFIG_DMX_ISR_IN_IRAM
#define DMX_ISR_ATTR IRAM_ATTR
#else
#define DMX_ISR_ATTR
#endif

DRAM_ATTR dmx_context_t dmx_context[DMX_NUM_MAX] = {
    DMX_CONTEXT_INIT(DMX_NUM_0),
    DMX_CONTEXT_INIT(DMX_NUM_1),
#if DMX_NUM_MAX > 2
    DMX_CONTEXT_INIT(DMX_NUM_2),
#endif
};

DRAM_ATTR dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {0};

enum dmx_default_interrupt_values {
  DMX_UART_FULL_DEFAULT = 1,   // RX FIFO full default interrupt threshold.
  DMX_UART_EMPTY_DEFAULT = 8,  // TX FIFO empty default interrupt threshold.
};

enum dmx_interrupt_mask {
  DMX_INTR_RX_FIFO_OVERFLOW = UART_INTR_RXFIFO_OVF,
  DMX_INTR_RX_FRAMING_ERR = UART_INTR_PARITY_ERR | UART_INTR_RS485_PARITY_ERR |
                            UART_INTR_FRAM_ERR | UART_INTR_RS485_FRM_ERR,
  DMX_INTR_RX_ERR = DMX_INTR_RX_FIFO_OVERFLOW | DMX_INTR_RX_FRAMING_ERR,

  DMX_INTR_RX_BREAK = UART_INTR_BRK_DET,
  DMX_INTR_RX_DATA = UART_INTR_RXFIFO_FULL,
  DMX_INTR_RX_ALL = DMX_INTR_RX_DATA | DMX_INTR_RX_BREAK | DMX_INTR_RX_ERR,

  DMX_INTR_TX_DATA = UART_INTR_TXFIFO_EMPTY,
  DMX_INTR_TX_DONE = UART_INTR_TX_DONE,
  DMX_INTR_TX_ALL = DMX_INTR_TX_DATA | DMX_INTR_TX_DONE,

  DMX_ALL_INTR_MASK = -1
};

enum rdm_packet_timing {
  RDM_DISCOVERY_NO_RESPONSE_PACKET_SPACING = 5800,
  RDM_REQUEST_NO_RESPONSE_PACKET_SPACING = 3000,
  RDM_BROADCAST_PACKET_SPACING = 176,
  RDM_RESPOND_TO_REQUEST_PACKET_SPACING = 176,

  RDM_CONTROLLER_RESPONSE_LOST_TIMEOUT = 2800,
  RDM_RESPONDER_RESPONSE_LOST_TIMEOUT = 2000
};

static void DMX_ISR_ATTR dmx_uart_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const context = &dmx_context[driver->dmx_num];

  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_uart_get_interrupt_status(context);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_ERR) {
      if (!driver->received_a_packet) {
        // Read data from the FIFO into the driver buffer if possible
        if (driver->data.head >= 0 && driver->data.head < DMX_MAX_PACKET_SIZE) {
          // Data can be read into driver buffer
          int read_len = DMX_MAX_PACKET_SIZE - driver->data.head;
          uint8_t *data_ptr = &driver->data.buffer[driver->data.head];
          dmx_uart_read_rxfifo(context, data_ptr, &read_len);
          driver->data.head += read_len;
        } else {
          // Data cannot be read into driver buffer
          if (driver->data.head > 0) {
            // Only increment head if data has already been read into the buffer
            driver->data.head += dmx_uart_get_rxfifo_len(context);
          }
          dmx_uart_rxfifo_reset(context);
        }

        taskENTER_CRITICAL_ISR(&context->spinlock);
        // Set driver flags
        driver->is_in_break = false;
        driver->data.timestamp = now;
        driver->received_a_packet = true;
        driver->data.err = intr_flags & DMX_INTR_RX_FRAMING_ERR
                               ? ESP_ERR_INVALID_RESPONSE
                               : ESP_FAIL;

        // Notify the task if there is one waiting
        if (driver->task_waiting) {
          xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                             eSetValueWithOverwrite, &task_awoken);
        }
        taskEXIT_CRITICAL_ISR(&context->spinlock);
      } else {
        // Packet has already been received - don't process errors
        dmx_uart_rxfifo_reset(context);
      }
      dmx_uart_clear_interrupt(context, DMX_INTR_RX_ERR);
    }

    else if (intr_flags & DMX_INTR_RX_BREAK) {
      // Reset the FIFO and clear the interrupt
      dmx_uart_rxfifo_reset(context);
      dmx_uart_clear_interrupt(context, DMX_INTR_RX_BREAK);

      // Stop the receive timeout if it is running
      dmx_timer_pause(context);

      if (!driver->received_a_packet) {
        // When a DMX break is received before the driver thinks a packet is
        // finished, the data.rx_size must be updated.
        driver->data.rx_size = driver->data.head;
      }

      taskENTER_CRITICAL_ISR(&context->spinlock);
      // Set driver flags
      driver->is_in_break = true;
      driver->received_a_packet = false;
      driver->packet_was_handled = false;
      driver->data.head = 0;  // Driver buffer is ready for data
      taskEXIT_CRITICAL_ISR(&context->spinlock);
    }

    else if (intr_flags & DMX_INTR_RX_DATA) {
      // Read data from the FIFO into the driver buffer if possible
      if (driver->data.head >= 0 && driver->data.head < DMX_MAX_PACKET_SIZE) {
        // Data can be read into driver buffer
        int read_len = DMX_MAX_PACKET_SIZE - driver->data.head;
        uint8_t *data_ptr = &driver->data.buffer[driver->data.head];
        dmx_uart_read_rxfifo(context, data_ptr, &read_len);
        driver->data.head += read_len;
      } else {
        // Data cannot be read into driver buffer
        if (driver->data.head > 0) {
          // Only increment head if data has already been read into the buffer
          driver->data.head += dmx_uart_get_rxfifo_len(context);
        }
        dmx_uart_rxfifo_reset(context);
      }
      dmx_uart_clear_interrupt(context, DMX_INTR_RX_DATA);

      // Stop the receive timeout if it is running
      dmx_timer_pause(context);

      // Set driver flags
      taskENTER_CRITICAL_ISR(&context->spinlock);
      driver->is_in_break = false;
      driver->data.timestamp = now;
      taskEXIT_CRITICAL_ISR(&context->spinlock);

      // Determine if a complete packet has been received
      bool packet_is_complete = false;
      if (!driver->received_a_packet) {
        const rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
        if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC) {
          // The packet is a standard RDM packet
          if (driver->data.head >= RDM_BASE_PACKET_SIZE &&
              driver->data.head >= rdm->message_len + 2) {
            if (rdm->cc == RDM_CC_DISC_COMMAND &&
                rdm->pid == bswap16(RDM_PID_DISC_UNIQUE_BRANCH)) {
              driver->data.type = RDM_PACKET_TYPE_DISCOVERY;
            } else if (RDM_UID_IS_BROADCAST(buf_to_uid(rdm->destination_uid))) {
              driver->data.type = RDM_PACKET_TYPE_BROADCAST;
            } else if (rdm->cc == RDM_CC_GET_COMMAND ||
                       rdm->cc == RDM_CC_SET_COMMAND ||
                       rdm->cc == RDM_CC_DISC_COMMAND) {
              driver->data.type = RDM_PACKET_TYPE_REQUEST;
            } else {
              driver->data.type = RDM_PACKET_TYPE_RESPONSE;
            }
            packet_is_complete = true;
          }
        } else if (rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER) {
          // The packet is a DISC_UNIQUE_BRANCH response
          if (driver->data.head >= 17) {
            // Find the length of the preamble (0 to 7 bytes)
            size_t preamble_len = 0;
            for (; preamble_len < 7; ++preamble_len) {
              if (driver->data.buffer[preamble_len] == RDM_DELIMITER) {
                break;
              }
            }

            if (driver->data.head >= preamble_len + 17) {
              // DISC_UNIQUE_BRANCH responses should be 17 bytes after preamble
              taskENTER_CRITICAL_ISR(&context->spinlock);
              driver->data.type = RDM_PACKET_TYPE_DISCOVERY_RESPONSE;
              taskEXIT_CRITICAL_ISR(&context->spinlock);
              packet_is_complete = true;
            }
          }
        } else {
          // The packet is a DMX packet
          if (driver->data.head >= driver->data.rx_size) {
            taskENTER_CRITICAL_ISR(&context->spinlock);
            driver->data.type = RDM_PACKET_TYPE_NON_RDM;
            taskEXIT_CRITICAL_ISR(&context->spinlock);
            packet_is_complete = true;
          }
        }
      }

      // Notify tasks that the packet is complete
      if (packet_is_complete) {
        taskENTER_CRITICAL_ISR(&context->spinlock);
        driver->data.err = ESP_OK;
        driver->received_a_packet = true;
        driver->data.sent_last = false;
        if (driver->task_waiting) {
          xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                             eSetValueWithOverwrite, &task_awoken);
        }
        taskEXIT_CRITICAL_ISR(&context->spinlock);
      }
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // Write data to the UART and clear the interrupt
      size_t write_size = driver->data.tx_size - driver->data.head;
      const uint8_t *src = &driver->data.buffer[driver->data.head];
      dmx_uart_write_txfifo(context, src, &write_size);
      driver->data.head += write_size;
      dmx_uart_clear_interrupt(context, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->data.head == driver->data.tx_size) {
        dmx_uart_disable_interrupt(context, DMX_INTR_TX_DATA);
      }
    }

    else if (intr_flags & DMX_INTR_TX_DONE) {
      // Disable write interrupts and clear the interrupt
      dmx_uart_disable_interrupt(context, DMX_INTR_TX_ALL);
      dmx_uart_clear_interrupt(context, DMX_INTR_TX_DONE);

      // Record timestamp, unset sending flag, and notify task
      taskENTER_CRITICAL_ISR(&context->spinlock);
      driver->is_sending = false;
      driver->data.timestamp = now;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, 0, eNoAction, &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(&context->spinlock);

      // Turn DMX bus around quickly if expecting an RDM response
      bool expecting_response = false;
      if (driver->data.type == RDM_PACKET_TYPE_DISCOVERY) {
        expecting_response = true;
        driver->data.head = 0;  // Not expecting a DMX break
      } else if (driver->data.type == RDM_PACKET_TYPE_REQUEST) {
        expecting_response = true;
        driver->data.head = -1;  // Expecting a DMX break
      }
      taskENTER_CRITICAL_ISR(&context->spinlock);
      if (expecting_response) {
        driver->received_a_packet = false;
        driver->packet_was_handled = false;
        dmx_uart_rxfifo_reset(context);
        dmx_uart_set_rts(context, 1);
        dmx_uart_clear_interrupt(context, DMX_INTR_RX_ALL);
        dmx_uart_enable_interrupt(context, DMX_INTR_RX_ALL);
      }
      taskEXIT_CRITICAL_ISR(&context->spinlock);
    }
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

static void DMX_ISR_ATTR dmx_gpio_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const context = &dmx_context[driver->dmx_num];
  int task_awoken = false;

  if (dmx_uart_get_rx_level(context)) {
    /* If this ISR is called on a positive edge and the current DMX frame is in
    a break and a negative edge timestamp has been recorded then a break has
    just finished. Therefore the DMX break length is able to be recorded. It can
    also be deduced that the driver is now in a DMX mark-after-break. */

    if (driver->is_in_break && driver->sniffer.last_neg_edge_ts > -1) {
      driver->sniffer.data.break_len = now - driver->sniffer.last_neg_edge_ts;
      driver->sniffer.is_in_mab = true;
      driver->is_in_break = false;
    }
    driver->sniffer.last_pos_edge_ts = now;
  } else {
    /* If this ISR is called on a negative edge in a DMX mark-after-break then
    the DMX mark-after-break has just finished. It can be recorded. Sniffer data
    is now available to be read by the user. */

    if (driver->sniffer.is_in_mab) {
      driver->sniffer.data.mab_len = now - driver->sniffer.last_pos_edge_ts;
      driver->sniffer.is_in_mab = false;

      // Send the sniffer data to the queue
      xQueueOverwriteFromISR(driver->sniffer.queue, &driver->sniffer.data,
                             &task_awoken);
    }
    driver->sniffer.last_neg_edge_ts = now;
  }
}

static bool DMX_ISR_ATTR dmx_timer_isr(void *arg) {
  const dmx_port_t dmx_num = *(dmx_port_t *)arg;
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const context = &dmx_context[dmx_num];
  int task_awoken = false;

  if (!driver->is_sending && driver->task_waiting) {
    // Notify the task and pause the timer
    dmx_timer_pause(context);
    context->timer_running = false;
    xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                       eSetValueWithOverwrite, &task_awoken);
  } else if (driver->is_in_break) {
    // End the DMX break
    dmx_uart_invert_tx(context, 0);
    driver->is_in_break = false;

    // Get the configured length of the DMX mark-after-break
    taskENTER_CRITICAL_ISR(&context->spinlock);
    const uint32_t mab_len = driver->mab_len;
    taskEXIT_CRITICAL_ISR(&context->spinlock);

    // Reset the alarm for the end of the DMX mark-after-break
    dmx_timer_set_alarm(context, mab_len);
  } else {
    // Write data to the UART and pause the timer
    size_t write_size = driver->data.tx_size;
    dmx_uart_write_txfifo(context, driver->data.buffer, &write_size);
    driver->data.head += write_size;
    dmx_timer_pause(context);

    // Enable DMX write interrupts
    dmx_uart_enable_interrupt(context, DMX_INTR_TX_ALL);
  }

  return task_awoken;
}

static const char *TAG = "dmx";  // The log tagline for the file.

esp_err_t dmx_driver_install(dmx_port_t dmx_num, int intr_flags) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(!dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is already installed");

#ifdef CONFIG_DMX_ISR_IN_IRAM
  // Driver ISR is in IRAM so interrupt flags must include IRAM flag
  if (!(intr_flags & ESP_INTR_FLAG_IRAM)) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    intr_flags |= ESP_INTR_FLAG_IRAM;
  }
#endif

  dmx_context_t *const context = &dmx_context[dmx_num];
  dmx_driver_t *driver;

  // Initialize and flush the UART
  dmx_uart_init(dmx_num, context);
  dmx_uart_rxfifo_reset(context);
  dmx_uart_txfifo_reset(context);

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
  driver->task_waiting = NULL;

  // Initialize driver flags
  driver->is_in_break = false;
  driver->received_a_packet = false;
  driver->packet_was_handled = false;
  driver->is_sending = false;
  driver->rdm_is_muted = false;
  driver->rdm_tn = 0;

  // Initialize the driver buffer
  bzero(driver->data.buffer, DMX_MAX_PACKET_SIZE);
  driver->data.sent_last = false;
  driver->data.type = RDM_PACKET_TYPE_NON_RDM;
  driver->data.timestamp = 0;
  driver->data.head = -1;  // Wait for DMX break before reading data
  driver->data.rx_size = DMX_MAX_PACKET_SIZE;

  // Initialize DMX transmit settings
  driver->break_len = RDM_BREAK_LEN_US;
  driver->mab_len = RDM_MAB_LEN_US;

  // Initialize sniffer in the disabled state
  driver->sniffer.queue = NULL;

  // Install UART interrupt
  dmx_uart_disable_interrupt(context, DMX_ALL_INTR_MASK);
  dmx_uart_clear_interrupt(context, DMX_ALL_INTR_MASK);
  dmx_uart_set_txfifo_empty(context, DMX_UART_EMPTY_DEFAULT);
  dmx_uart_set_rxfifo_full(context, DMX_UART_FULL_DEFAULT);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, intr_flags, &dmx_uart_isr,
                 driver, &driver->uart_isr_handle);

  // TODO: Allow busy-waiting instead of hardware timers
  // Initialize hardware timer
  dmx_timer_init(dmx_num, context);
  dmx_timer_add_callback(context, dmx_timer_isr, driver, intr_flags);

  // Enable UART read interrupt and set RTS low
  taskENTER_CRITICAL(&context->spinlock);
  dmx_uart_enable_interrupt(context, DMX_INTR_RX_ALL);
  dmx_uart_set_rts(context, 1);
  taskEXIT_CRITICAL(&context->spinlock);

  // Give the mutex and return
  xSemaphoreGiveRecursive(driver->mux);
  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is not installed");

  dmx_context_t *const context = &dmx_context[dmx_num];
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
  if (dmx_sniffer_is_enabled(dmx_num)) {
    dmx_sniffer_disable(dmx_num);
  }

  // Free driver data buffer
  if (driver->data.buffer != NULL) {
    heap_caps_free(driver->data.buffer);
  }

  // Free hardware timer ISR
  // TODO: check if using busy-waits
  // TODO: unregister ISR callback
  dmx_timer_deinit(context);
  
  // Free driver
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;

  // Disable UART module
  // TODO: make this a context function
  taskENTER_CRITICAL(&context->spinlock);
  if (context->hw_enabled) {
    if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
      periph_module_disable(uart_periph_signal[dmx_num].module);
    }
    context->hw_enabled = false;
  }
  taskEXIT_CRITICAL(&context->spinlock);

  return ESP_OK;
}

bool dmx_driver_is_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && dmx_driver[dmx_num] != NULL;
}

esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_pin, int rx_pin, int rts_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(tx_pin < 0 || GPIO_IS_VALID_OUTPUT_GPIO(tx_pin),
            ESP_ERR_INVALID_ARG, "tx_pin error");
  DMX_CHECK(rx_pin < 0 || GPIO_IS_VALID_GPIO(rx_pin), ESP_ERR_INVALID_ARG,
            "rx_pin error");
  DMX_CHECK(rts_pin < 0 || GPIO_IS_VALID_OUTPUT_GPIO(rts_pin),
            ESP_ERR_INVALID_ARG, "rts_pin error");

  return uart_set_pin(dmx_num, tx_pin, rx_pin, rts_pin, DMX_PIN_NO_CHANGE);
}

esp_err_t dmx_sniffer_enable(dmx_port_t dmx_num, int intr_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(intr_pin > 0 && GPIO_IS_VALID_GPIO(intr_pin), ESP_ERR_INVALID_ARG,
            "intr_pin error");
  DMX_CHECK(!dmx_sniffer_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "sniffer is already enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Allocate the sniffer queue
  driver->sniffer.queue = xQueueCreate(1, sizeof(dmx_sniffer_data_t));
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
  driver->sniffer.is_in_mab = false;

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

bool dmx_sniffer_get_data(dmx_port_t dmx_num, dmx_sniffer_data_t *sniffer_data,
                          TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(sniffer_data, ESP_ERR_INVALID_ARG, "sniffer_data is null");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "sniffer is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  return xQueueReceive(driver->sniffer.queue, sniffer_data, wait_ticks);
}

uint32_t dmx_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");

  // Clamp the baud rate to within DMX specification
  if (baud_rate < DMX_MIN_BAUD_RATE) {
    baud_rate = DMX_MIN_BAUD_RATE;
  } else if (baud_rate > DMX_MAX_BAUD_RATE) {
    baud_rate = DMX_MAX_BAUD_RATE;
  }

  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  dmx_uart_set_baud_rate(context, baud_rate);
  taskEXIT_CRITICAL(&context->spinlock);

  return baud_rate;
}

uint32_t dmx_get_baud_rate(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");

  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  const uint32_t baud_rate = dmx_uart_get_baud_rate(context);
  taskEXIT_CRITICAL(&context->spinlock);

  return baud_rate;
}

uint32_t dmx_set_break_len(dmx_port_t dmx_num, uint32_t break_len) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp the break length to within DMX specification
  if (break_len < DMX_MIN_BREAK_LEN_US) {
    break_len = DMX_MIN_BREAK_LEN_US;
  } else if (break_len > DMX_MAX_BREAK_LEN_US) {
    break_len = DMX_MAX_BREAK_LEN_US;
  }

  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  dmx_driver[dmx_num]->break_len = break_len;
  taskEXIT_CRITICAL(&context->spinlock);

  return break_len;
}

uint32_t dmx_get_break_len(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  const uint32_t break_len = dmx_driver[dmx_num]->break_len;
  taskEXIT_CRITICAL(&context->spinlock);

  return break_len;
}

uint32_t dmx_set_mab_len(dmx_port_t dmx_num, uint32_t mab_len) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp the mark-after-break length to within DMX specification
  if (mab_len < DMX_MIN_MAB_LEN_US) {
    mab_len = DMX_MIN_MAB_LEN_US;
  } else if (mab_len > DMX_MAX_MAB_LEN_US) {
    mab_len = DMX_MAX_MAB_LEN_US;
  }

  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  dmx_driver[dmx_num]->mab_len = mab_len;
  taskEXIT_CRITICAL(&context->spinlock);

  return mab_len;
}

uint32_t dmx_get_mab_len(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  const uint32_t mab_len = dmx_driver[dmx_num]->mab_len;
  taskEXIT_CRITICAL(&context->spinlock);

  return mab_len;
}

size_t dmx_read(dmx_port_t dmx_num, void *destination, size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(destination, 0, "destination is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp size to the maximum DMX packet size
  if (size > DMX_MAX_PACKET_SIZE) {
    size = DMX_MAX_PACKET_SIZE;
  } else if (size == 0) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Copy data from the driver buffer to the destination asynchronously
  memcpy(destination, driver->data.buffer, size);

  return size;
}

size_t dmx_read_offset(dmx_port_t dmx_num, size_t offset, void *destination,
                       size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(offset < DMX_MAX_PACKET_SIZE, 0, "offset error");
  DMX_CHECK(destination, 0, "destination is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp size to the maximum DMX packet size
  if (size + offset > DMX_MAX_PACKET_SIZE) {
    size = DMX_MAX_PACKET_SIZE - offset;
  } else if (size == 0) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Copy data from the driver buffer to the destination asynchronously
  memcpy(destination, driver->data.buffer + offset, size);

  return size;
}

int dmx_read_slot(dmx_port_t dmx_num, size_t slot_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, -1, "dmx_num error");
  DMX_CHECK(slot_num < DMX_MAX_PACKET_SIZE, -1, "slot_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), -1, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Return data from the driver buffer asynchronously
  return driver->data.buffer[slot_num];
}

size_t dmx_write(dmx_port_t dmx_num, const void *source, size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(source, 0, "source is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp size to the maximum DMX packet size or fail quickly on invalid size
  if (size > DMX_MAX_PACKET_SIZE) {
    size = DMX_MAX_PACKET_SIZE;
  } else if (size == 0) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  if (driver->is_sending && driver->data.type != RDM_PACKET_TYPE_NON_RDM) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(&context->spinlock);
    return 0;
  } else if (dmx_uart_get_rts(context) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_uart_disable_interrupt(context, DMX_INTR_RX_ALL);
    dmx_uart_set_rts(context, 0);
  }
  driver->data.tx_size = size;  // Update driver transmit size
  taskEXIT_CRITICAL(&context->spinlock);

  // Copy data from the source to the driver buffer asynchronously
  memcpy(driver->data.buffer, source, size);

  return size;
}

size_t dmx_write_offset(dmx_port_t dmx_num, size_t offset, const void *source,
                        size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(offset < DMX_MAX_PACKET_SIZE, 0, "offset error");
  DMX_CHECK(source, 0, "source is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp size to the maximum DMX packet size
  if (size + offset > DMX_MAX_PACKET_SIZE) {
    size = DMX_MAX_PACKET_SIZE - offset;
  } else if (size == 0) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  if (driver->is_sending && driver->data.type != RDM_PACKET_TYPE_NON_RDM) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(&context->spinlock);
    return 0;
  } else if (dmx_uart_get_rts(context) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_uart_disable_interrupt(context, DMX_INTR_RX_ALL);
    dmx_uart_set_rts(context, 0);
  }
  driver->data.tx_size = offset + size;  // Update driver transmit size
  taskEXIT_CRITICAL(&context->spinlock);

  // Copy data from the source to the driver buffer asynchronously
  memcpy(driver->data.buffer + offset, source, size);

  return size;
}

int dmx_write_slot(dmx_port_t dmx_num, size_t slot_num, uint8_t value) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, -1, "dmx_num error");
  DMX_CHECK(slot_num < DMX_MAX_PACKET_SIZE, -1, "slot_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), -1, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  if (driver->is_sending && driver->data.type != RDM_PACKET_TYPE_NON_RDM) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(&context->spinlock);
    return 0;
  } else if (dmx_uart_get_rts(context) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_uart_disable_interrupt(context, DMX_INTR_RX_ALL);
    dmx_uart_set_rts(context, 0);
  }

  // Ensure that the next packet to be sent includes this slot
  if (driver->data.tx_size < slot_num) {
    driver->data.tx_size = slot_num;
  }
  taskEXIT_CRITICAL(&context->spinlock);

  // Set the driver buffer slot to the assigned value asynchronously
  driver->data.buffer[slot_num] = value;

  return value;
}

size_t dmx_receive(dmx_port_t dmx_num, dmx_event_t *event,
                   TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const context = &dmx_context[dmx_num];

  // Block until the mutex can be taken
  TimeOut_t timeout;
  vTaskSetTimeOutState(&timeout);
  if (!xSemaphoreTakeRecursive(driver->mux, wait_ticks) ||
      xTaskCheckForTimeOut(&timeout, &wait_ticks)) {
    return 0;
  }

  // Block until the driver is done sending
  if (!dmx_wait_sent(dmx_num, wait_ticks) ||
      xTaskCheckForTimeOut(&timeout, &wait_ticks)) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Set the RTS pin to read from the DMX bus
  taskENTER_CRITICAL(&context->spinlock);
  if (dmx_uart_get_rts(context) == 0) {
    dmx_uart_disable_interrupt(context, DMX_INTR_TX_ALL);
    dmx_uart_set_rts(context, 1);
    driver->data.head = -1;  // Wait for DMX break before reading data
    dmx_uart_enable_interrupt(context, DMX_INTR_RX_ALL);
  }
  taskEXIT_CRITICAL(&context->spinlock);

  // Receive the latest data packet or check if the driver must wait
  esp_err_t err = ESP_OK;
  uint32_t packet_size = 0;
  taskENTER_CRITICAL(&context->spinlock);
  const bool driver_received_a_packet = driver->received_a_packet;
  const bool packet_was_handled = driver->packet_was_handled;
  driver->packet_was_handled = true;
  taskEXIT_CRITICAL(&context->spinlock);
  if (!driver_received_a_packet || packet_was_handled) {
    driver->task_waiting = xTaskGetCurrentTaskHandle();
  } else {
    taskENTER_CRITICAL(&context->spinlock);
    packet_size = driver->data.head;
    err = driver->data.err;
    if (packet_size == -1) {
      packet_size = 0;
    }
    taskEXIT_CRITICAL(&context->spinlock);
  }

  if (!driver_received_a_packet) {
    // Get additional driver flags
    taskENTER_CRITICAL(&context->spinlock);
    const bool driver_sent_last = driver->data.sent_last;
    const bool expecting_response =
        driver->data.type == RDM_PACKET_TYPE_DISCOVERY ||
        driver->data.type == RDM_PACKET_TYPE_REQUEST;
    taskEXIT_CRITICAL(&context->spinlock);

    if (driver_sent_last && expecting_response) {
      // An RDM response is expected in <10ms so a hardware timer may be needed
      taskENTER_CRITICAL(&context->spinlock);
      const int64_t elapsed = esp_timer_get_time() - driver->data.timestamp;
      if (elapsed < RDM_CONTROLLER_RESPONSE_LOST_TIMEOUT) {
        dmx_timer_set_counter(context, elapsed);
        dmx_timer_set_alarm(context, RDM_CONTROLLER_RESPONSE_LOST_TIMEOUT);
        dmx_timer_start(context);
      }
      taskEXIT_CRITICAL(&context->spinlock);

      // Check if the response has already timed out
      if (elapsed >= RDM_CONTROLLER_RESPONSE_LOST_TIMEOUT) {
        driver->task_waiting = NULL;
        xTaskNotifyStateClear(xTaskGetCurrentTaskHandle()); // TODO: needed?
        xSemaphoreGiveRecursive(driver->mux);
        return packet_size;
      }
    }

    // Wait for a task notification
    if (xTaskNotifyWait(0, ULONG_MAX, &packet_size, wait_ticks)) {
      err = driver->data.err;
      if (packet_size == -1) {
        packet_size = 0;
      }
    }
    driver->task_waiting = NULL;
  }

  // Report packet data in the DMX event
  if (event != NULL) {
    event->err = err;
    event->size = packet_size;
    if (packet_size > 0) {
      bool is_rdm = false;
      if (!err) {
        // Quickly check if the packet is an RDM packet
        const rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
        taskENTER_CRITICAL(&context->spinlock);
        is_rdm = (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC) ||
                 rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER;
        taskEXIT_CRITICAL(&context->spinlock);
      }
      event->sc = driver->data.buffer[0];
      event->is_rdm = is_rdm;
    } else {
      event->sc = -1;
      event->is_rdm = false;
    }
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return packet_size;
}

size_t dmx_send(dmx_port_t dmx_num, size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const context = &dmx_context[dmx_num];

  // Block until the mutex can be taken
  if (!xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY)) {
    return 0;
  }

  // Block until the driver is done sending
  if (!dmx_wait_sent(dmx_num, portMAX_DELAY)) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Determine if it is too late to send a response packet
  int64_t elapsed = 0;
  taskENTER_CRITICAL(&context->spinlock);
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC &&
      (rdm->cc == RDM_CC_DISC_COMMAND_RESPONSE ||
       rdm->cc == RDM_CC_GET_COMMAND_RESPONSE ||
       rdm->cc == RDM_CC_SET_COMMAND_RESPONSE)) {
    elapsed = esp_timer_get_time() - driver->data.timestamp;
  }
  taskEXIT_CRITICAL(&context->spinlock);
  if (elapsed >= RDM_RESPONDER_RESPONSE_LOST_TIMEOUT) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Determine if an alarm needs to be set to wait until driver is ready
  uint32_t timeout = 0;
  taskENTER_CRITICAL(&context->spinlock);
  if (driver->data.sent_last) {
    if (driver->data.type == RDM_PACKET_TYPE_DISCOVERY) {
      timeout = RDM_DISCOVERY_NO_RESPONSE_PACKET_SPACING;
    } else if (driver->data.type == RDM_PACKET_TYPE_BROADCAST) {
      timeout = RDM_BROADCAST_PACKET_SPACING;
    } else if (driver->data.type == RDM_PACKET_TYPE_REQUEST) {
      timeout = RDM_REQUEST_NO_RESPONSE_PACKET_SPACING;
    }
  } else if (driver->data.type != RDM_PACKET_TYPE_NON_RDM) {
    timeout = RDM_RESPOND_TO_REQUEST_PACKET_SPACING;
  }
  elapsed = esp_timer_get_time() - driver->data.timestamp;
  if (elapsed < timeout) {
    dmx_timer_set_counter(context, elapsed);
    dmx_timer_set_alarm(context, timeout);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
    dmx_timer_start(context);
  }
  taskEXIT_CRITICAL(&context->spinlock);

  // Block if an alarm was set
  if (elapsed < timeout) {
    bool notified = xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    if (!notified) {
      dmx_timer_pause(context);
      xTaskNotifyStateClear(driver->task_waiting);
    }
    driver->task_waiting = NULL;
    if (!notified) {
      xSemaphoreGiveRecursive(driver->mux);
      return 0;
    }
  }

  // Turn the DMX bus around and get the send size
  taskENTER_CRITICAL(&context->spinlock);
  if (dmx_uart_get_rts(context) == 1) {
    dmx_uart_disable_interrupt(context, DMX_INTR_RX_ALL);
    dmx_uart_set_rts(context, 0);
  }
  taskEXIT_CRITICAL(&context->spinlock);

  // Update the transmit size if desired
  if (size > 0) {
    if (size > DMX_MAX_PACKET_SIZE) {
      size = DMX_MAX_PACKET_SIZE;
    }
    taskENTER_CRITICAL(&context->spinlock);
    driver->data.tx_size = size;
    taskEXIT_CRITICAL(&context->spinlock);
  } else {
    taskENTER_CRITICAL(&context->spinlock);
    size = driver->data.tx_size;
    taskEXIT_CRITICAL(&context->spinlock);
  }

  // Record the outgoing packet type
  enum rdm_type_t packet_type = RDM_PACKET_TYPE_NON_RDM;
  if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC) {
    if (rdm->cc == RDM_CC_DISC_COMMAND &&
        rdm->pid == bswap16(RDM_PID_DISC_UNIQUE_BRANCH)) {
      packet_type = RDM_PACKET_TYPE_DISCOVERY;
    } else if (RDM_UID_IS_BROADCAST(buf_to_uid(rdm->destination_uid))) {
      packet_type = RDM_PACKET_TYPE_BROADCAST;
    } else if (rdm->cc == RDM_CC_GET_COMMAND || rdm->cc == RDM_CC_SET_COMMAND ||
               rdm->cc == RDM_CC_DISC_COMMAND) {
      packet_type = RDM_PACKET_TYPE_REQUEST;
    } else {
      packet_type = RDM_PACKET_TYPE_RESPONSE;
    }
  } else if (rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER) {
    packet_type = RDM_PACKET_TYPE_DISCOVERY_RESPONSE;
  }
  driver->data.type = packet_type;
  driver->data.sent_last = true;

  // Determine if a DMX break is required and send the packet
  if (packet_type == RDM_PACKET_TYPE_DISCOVERY_RESPONSE) {
    // RDM discovery responses do not send a DMX break - write immediately
    taskENTER_CRITICAL(&context->spinlock);
    driver->is_sending = true;

    size_t write_size = driver->data.tx_size;
    dmx_uart_write_txfifo(context, driver->data.buffer, &write_size);
    driver->data.head = write_size;

    // Enable DMX write interrupts
    dmx_uart_enable_interrupt(context, DMX_INTR_TX_ALL);
    taskEXIT_CRITICAL(&context->spinlock);
  } else {
    // Use the hardware timer to send a DMX break and mark-after-break
    taskENTER_CRITICAL(&context->spinlock);
    const uint32_t break_len = driver->break_len;
    taskEXIT_CRITICAL(&context->spinlock);
    dmx_timer_set_counter(context, 0);
    dmx_timer_set_alarm(context, break_len);

    // Send the packet by starting the DMX break
    taskENTER_CRITICAL(&context->spinlock);
    driver->is_sending = true;
    driver->is_in_break = true;
    driver->data.head = 0;
    dmx_uart_invert_tx(context, 1);
    dmx_timer_start(context);
    taskEXIT_CRITICAL(&context->spinlock);
  }

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return size;
}

bool dmx_wait_sent(dmx_port_t dmx_num, TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const context = &dmx_context[dmx_num];

  // Block until the mutex can be taken
  TimeOut_t timeout;
  vTaskSetTimeOutState(&timeout);
  if (!xSemaphoreTakeRecursive(driver->mux, wait_ticks) ||
      xTaskCheckForTimeOut(&timeout, &wait_ticks)) {
    return false;
  }

  // Determine if the task needs to block
  bool result = true;
  if (wait_ticks > 0) {
    bool task_waiting = false;
    taskENTER_CRITICAL(&context->spinlock);
    if (driver->is_sending) {
      driver->task_waiting = xTaskGetCurrentTaskHandle();
      task_waiting = true;
    }
    taskEXIT_CRITICAL(&context->spinlock);

    // Wait for a notification that the driver is done sending
    if (task_waiting) {
      result = xTaskNotifyWait(0, ULONG_MAX, NULL, wait_ticks);
      driver->task_waiting = NULL;
    }
  } else {
    taskENTER_CRITICAL(&context->spinlock);
    if (driver->is_sending) {
      result = false;
    }
    taskEXIT_CRITICAL(&context->spinlock);
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return result;
}

DMX_ISR_ATTR rdm_uid_t buf_to_uid(const void *buf) {
  rdm_uid_t val = 0;
  ((uint8_t *)&val)[5] = ((uint8_t *)buf)[0];
  ((uint8_t *)&val)[4] = ((uint8_t *)buf)[1];
  ((uint8_t *)&val)[3] = ((uint8_t *)buf)[2];
  ((uint8_t *)&val)[2] = ((uint8_t *)buf)[3];
  ((uint8_t *)&val)[1] = ((uint8_t *)buf)[4];
  ((uint8_t *)&val)[0] = ((uint8_t *)buf)[5];
  return val;
}

void *uid_to_buf(void *buf, rdm_uid_t uid) {
  ((uint8_t *)buf)[0] = ((uint8_t *)&uid)[5];
  ((uint8_t *)buf)[1] = ((uint8_t *)&uid)[4];
  ((uint8_t *)buf)[2] = ((uint8_t *)&uid)[3];
  ((uint8_t *)buf)[3] = ((uint8_t *)&uid)[2];
  ((uint8_t *)buf)[4] = ((uint8_t *)&uid)[1];
  ((uint8_t *)buf)[5] = ((uint8_t *)&uid)[0];
  return buf;
}
