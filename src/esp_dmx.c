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
#include "rdm_tools.h"

// Used for argument checking at the beginning of each function.
#define DMX_CHECK(a, err_code, format, ...) \
  ESP_RETURN_ON_FALSE(a, err_code, TAG, format, ##__VA_ARGS__)

// Initializes the DMX context.
#define DMX_CONTEXT_INIT(uart_num)                                 \
  {                                                                \
    .hal.dev = UART_LL_GET_HW(uart_num),                           \
    .spinlock = portMUX_INITIALIZER_UNLOCKED, .hw_enabled = false, \
  }

/**
 * @brief The context for the DMX driver. Contains a pointer to UART registers
 * as well as a spinlock for synchronizing access to resources and tracks if the
 * UART hardware has been enabled.
 */
typedef struct dmx_context {
  uart_hal_context_t hal;  // The UART context. Points to UART registers.
  spinlock_t spinlock;     // Synchronizes hardware and driver operations.
  int hw_enabled;          // True if the UART hardware has been initialized.
} dmx_context_t;

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
  DMX_INTR_RX_CLASH = UART_INTR_RS485_CLASH,
  DMX_INTR_RX_ALL = DMX_INTR_RX_DATA | DMX_INTR_RX_BREAK | DMX_INTR_RX_ERR |
                    DMX_INTR_RX_CLASH,

  DMX_INTR_TX_DATA = UART_INTR_TXFIFO_EMPTY,
  DMX_INTR_TX_DONE = UART_INTR_TX_DONE,
  DMX_INTR_TX_ALL = DMX_INTR_TX_DATA | DMX_INTR_TX_DONE,

  DMX_ALL_INTR_MASK = -1
};

static void IRAM_ATTR dmx_uart_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const context = &dmx_context[driver->dmx_num];

  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_hal_get_interrupt_status(&context->hal);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_ERR) {
      // Read from the FIFO on a framing error then clear the FIFO and interrupt
      if (intr_flags & DMX_INTR_RX_FRAMING_ERR) {
        size_t read_len = DMX_MAX_PACKET_SIZE - driver->data.head;
        if (!driver->received_packet && read_len > 0) {
          uint8_t *data_ptr = &driver->data.buffer[driver->data.head];
          dmx_hal_read_rxfifo(&context->hal, data_ptr, &read_len);
          driver->data.head += read_len;
        } else {
          dmx_hal_rxfifo_rst(&context->hal);
        }
        driver->data.err = DMX_ERR_IMPROPERLY_FRAMED_SLOT;
      } else {
        driver->data.err = DMX_ERR_HARDWARE_OVERFLOW;
      }
      dmx_hal_rxfifo_rst(&context->hal);
      dmx_hal_clear_interrupt(&context->hal, DMX_INTR_RX_ERR);

      // Don't process errors if the DMX bus is inactive
      if (driver->received_packet) {
        continue;
      }

      // Unset DMX break and receiving flags and notify task
      driver->is_in_break = false;
      driver->received_packet = true;
      taskENTER_CRITICAL_ISR(&context->spinlock);
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                           eSetValueWithOverwrite, &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(&context->spinlock);
    }

    else if (intr_flags & DMX_INTR_RX_BREAK) {
      // Reset the FIFO and clear the interrupt
      dmx_hal_rxfifo_rst(&context->hal);
      dmx_hal_clear_interrupt(&context->hal, DMX_INTR_RX_BREAK);

      // Stop the receive timeout if it is running
      if (driver->timer_running) {
        timer_group_set_counter_enable_in_isr(driver->timer_group,
                                              driver->timer_num, 0);
        driver->timer_running = false;
      }

      // Update packet size guess if driver hasn't received a packet yet
      if (!driver->received_packet) {
        driver->data.rx_size = driver->data.head;
      }

      // Set driver flags and reset data head
      driver->received_packet = false;
      driver->is_in_break = true;
      driver->data.head = 0;
    }

    else if (intr_flags & DMX_INTR_RX_DATA) {
      // Read from the FIFO if ready and clear the interrupt
      size_t read_len = DMX_MAX_PACKET_SIZE - driver->data.head;
      if (!driver->received_packet && read_len > 0) {
        uint8_t *data_ptr = &driver->data.buffer[driver->data.head];
        dmx_hal_read_rxfifo(&context->hal, data_ptr, &read_len);
        driver->data.head += read_len;
      } else {
        dmx_hal_rxfifo_rst(&context->hal);
      }
      dmx_hal_clear_interrupt(&context->hal, DMX_INTR_RX_DATA);

      // Unset DMX break flag and record the timestamp of the last slot
      if (driver->is_in_break) {
        driver->is_in_break = false;
      }
      driver->data.previous_ts = now;

      // Stop the receive timeout if it is running
      if (driver->timer_running) {
        timer_group_set_counter_enable_in_isr(driver->timer_group,
                                              driver->timer_num, 0);
        driver->timer_running = false;
      }

      // Don't process data if the driver is done receiving
      if (driver->received_packet) {
        continue;
      }

      // Determine if a full packet has been received and notify the task
      const rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
      if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC) {
        // An RDM packet is at least 26 bytes long
        if (driver->data.head >= 26) {
          // An RDM packet's length should match the message length slot value
          const rdm_data_t *rdm = (rdm_data_t *)driver->data.buffer;
          if (driver->data.head >= rdm->message_len + 2) {
            driver->data.previous_type = rdm->cc;
            driver->data.previous_uid = buf_to_uid(rdm->destination_uid);
            driver->received_packet = true;
          }
        }
      } else if (rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER) {
        // An RDM discovery response packet is at least 17 bytes long
        if (driver->data.head >= 17) {
          // Find the length of the discovery response preamble (0-7 bytes)
          size_t preamble_len = 0;
          for (; preamble_len < 7; ++preamble_len) {
            if (driver->data.buffer[preamble_len] == RDM_DELIMITER) {
              break;
            }
          }
          // Discovery response packets are 17 bytes long after the preamble
          if (driver->data.head >= preamble_len + 17) {
            driver->data.previous_type = RDM_DISCOVERY_COMMAND_RESPONSE;
            driver->received_packet = true;
          }
        }
      } else {
        // A DMX packet size should be equal to the expected packet size
        if (driver->data.head >= driver->data.rx_size) {
          driver->data.previous_type = RDM_NON_RDM_PACKET;
          driver->received_packet = true;
        }
      }
      if (driver->received_packet) {
        driver->data.err = DMX_OK;
        driver->data.sent_previous = false;
        taskENTER_CRITICAL_ISR(&context->spinlock);
        if (driver->task_waiting) {
          xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                             eSetValueWithOverwrite, &task_awoken);
        }
        taskEXIT_CRITICAL_ISR(&context->spinlock);
      }
    }

    else if (intr_flags & DMX_INTR_RX_CLASH) {
      // Multiple devices sent data at once (typical of RDM discovery)
      dmx_hal_rxfifo_rst(&context->hal);
      dmx_hal_clear_interrupt(&context->hal, DMX_INTR_RX_CLASH);
      driver->data.err = DMX_ERR_DATA_COLLISION;
      taskENTER_CRITICAL_ISR(&context->spinlock);
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                           eSetValueWithOverwrite, &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(&context->spinlock);
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // Write data to the UART and clear the interrupt
      size_t write_size = driver->data.tx_size - driver->data.head;
      const uint8_t *src = &driver->data.buffer[driver->data.head];
      dmx_hal_write_txfifo(&context->hal, src, &write_size);
      driver->data.head += write_size;
      dmx_hal_clear_interrupt(&context->hal, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->data.head == driver->data.tx_size) {
        dmx_hal_disable_interrupt(&context->hal, DMX_INTR_TX_DATA);
      }
    }

    else if (intr_flags & DMX_INTR_TX_DONE) {
      // Disable write interrupts and clear the interrupt
      dmx_hal_disable_interrupt(&context->hal, DMX_INTR_TX_ALL);
      dmx_hal_clear_interrupt(&context->hal, DMX_INTR_TX_DONE);

      // Record timestamp, unset sending flag, and notify task
      taskENTER_CRITICAL_ISR(&context->spinlock);
      driver->is_sending = false;
      driver->data.previous_ts = now;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, 0, eNoAction, &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(&context->spinlock);

      // Turn DMX bus around quickly if expecting an RDM response
      bool turn_bus_around = false;
      const rdm_data_t *rdm = (rdm_data_t *)driver->data.buffer;
      if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC) {
        // If packet was RDM and non-broadcast expect a response
        if (rdm->cc == RDM_GET_COMMAND || rdm->cc == RDM_SET_COMMAND) {
          const uint64_t destination_uid = buf_to_uid(rdm->destination_uid);
          if (destination_uid != RDM_BROADCAST_UID) {
            turn_bus_around = true;
          }
        } else if (rdm->cc == RDM_DISCOVERY_COMMAND) {
          // All discovery commands expect a response
          driver->received_packet = false;
          driver->data.head = 0;  // Response doesn't have a DMX break
          turn_bus_around = true;
        }
      }
      if (turn_bus_around) {
        dmx_hal_rxfifo_rst(&context->hal);
        dmx_hal_set_rts(&context->hal, 1);
        dmx_hal_clear_interrupt(&context->hal, DMX_INTR_RX_ALL);
        dmx_hal_enable_interrupt(&context->hal, DMX_INTR_RX_ALL);
      }
    }
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

static void IRAM_ATTR dmx_gpio_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const context = &dmx_context[driver->dmx_num];
  int task_awoken = false;

  if (dmx_hal_get_rx_level(&context->hal)) {

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

static bool IRAM_ATTR dmx_timer_isr(void *arg) {
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_context_t *const context = &dmx_context[driver->dmx_num];
  int task_awoken = false;

  if (!driver->is_sending && driver->task_waiting) {
    // Notify the task and pause the timer
    timer_group_set_counter_enable_in_isr(driver->timer_group,
                                          driver->timer_num, 0);
    driver->timer_running = false;
    xTaskNotifyFromISR(driver->task_waiting, driver->data.head,
                       eSetValueWithOverwrite, &task_awoken);
  } else if (driver->is_in_break) {
    // End the DMX break
    dmx_hal_invert_tx(&context->hal, 0);
    driver->is_in_break = false;

    // Get the configured length of the DMX mark-after-break
    taskENTER_CRITICAL_ISR(&context->spinlock);
    const uint32_t mab_len = driver->mab_len;
    taskEXIT_CRITICAL_ISR(&context->spinlock);

    // Reset the alarm for the end of the DMX mark-after-break
    timer_group_set_alarm_value_in_isr(driver->timer_group, driver->timer_num,
                                       mab_len);
  } else {
    // Write data to the UART and pause the timer
    size_t write_size = driver->data.tx_size;
    dmx_hal_write_txfifo(&context->hal, driver->data.buffer, &write_size);
    driver->data.head += write_size;
    timer_group_set_counter_enable_in_isr(driver->timer_group,
                                          driver->timer_num, 0);
    driver->timer_running = false;

    // Enable DMX write interrupts
    dmx_hal_enable_interrupt(&context->hal, DMX_INTR_TX_ALL);
  }

  return task_awoken;
}

static const char *TAG = "dmx";  // The log tagline for the file.

esp_err_t dmx_driver_install(dmx_port_t dmx_num, bool use_timer,
                             int intr_flags) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(!dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is already installed");
  
  // TODO: set intr_flags

  dmx_context_t *const context = &dmx_context[dmx_num];
  dmx_driver_t *driver;

  // Enable module and configure the UART hardware
  taskENTER_CRITICAL(&context->spinlock);
  if (!context->hw_enabled) {
    periph_module_enable(uart_periph_signal[dmx_num].module);
    if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
#if SOC_UART_REQUIRE_CORE_RESET
      // ESP32-C3 workaround to prevent UART outputting garbage data.
      uart_hal_set_reset_core(&context->hal, true);
      periph_module_reset(uart_periph_signal[dmx_num].module);
      uart_hal_set_reset_core(&context->hal), false);
#else
      periph_module_reset(uart_periph_signal[dmx_num].module);
#endif
    }
    context->hw_enabled = true;
  }
  taskEXIT_CRITICAL(&context->spinlock);
  dmx_hal_init(&context->hal);

  // Flush the hardware FIFOs
  dmx_hal_rxfifo_rst(&context->hal);
  dmx_hal_txfifo_rst(&context->hal);

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
  if (use_timer) {
    driver->timer_group = dmx_num / 2;
    driver->timer_num = dmx_num % 2;
  } else {
    driver->timer_group = -1;
    driver->timer_num = -1;
  }
  driver->task_waiting = NULL;

  // Initialize driver flags
  driver->is_in_break = false;
  driver->received_packet = false;
  driver->is_sending = false;
  driver->timer_running = false;
  driver->rdm_tn = 0;

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
  if (!(intr_flags & ESP_INTR_FLAG_IRAM)) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    intr_flags |= ESP_INTR_FLAG_IRAM;
  }

  // Install UART interrupt
  dmx_hal_disable_interrupt(&context->hal, DMX_ALL_INTR_MASK);
  dmx_hal_clear_interrupt(&context->hal, DMX_ALL_INTR_MASK);
  dmx_hal_set_txfifo_empty(&context->hal, DMX_UART_EMPTY_DEFAULT);
  dmx_hal_set_rxfifo_full(&context->hal, DMX_UART_FULL_DEFAULT);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, intr_flags, &dmx_uart_isr,
                 driver, &driver->uart_isr_handle);

  // Install hardware timer interrupt if specified by the user
  if (use_timer) {
    const timer_config_t timer_config = {
        .divider = 80,  // (80MHz / 80) == 1MHz resolution timer
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = false,
        .alarm_en = true,
        .auto_reload = true,
    };
    timer_init(driver->timer_group, driver->timer_num, &timer_config);
    timer_isr_callback_add(driver->timer_group, driver->timer_num,
                           dmx_timer_isr, driver, intr_flags);
    timer_enable_intr(driver->timer_group, driver->timer_num);
  }

  // Enable UART read interrupt and set RTS low
  taskENTER_CRITICAL(&context->spinlock);
  dmx_hal_enable_interrupt(&context->hal, DMX_INTR_RX_ALL);
  dmx_hal_set_rts(&context->hal, 1);
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
  if (driver->timer_group != -1) {
    timer_deinit(driver->timer_group, driver->timer_num);
  }

  // Free driver
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;

  // Disable UART module
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
                          TickType_t timeout) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(sniffer_data, ESP_ERR_INVALID_ARG, "sniffer_data is null");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "sniffer is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  return xQueueReceive(driver->sniffer.queue, sniffer_data, timeout);
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
  dmx_hal_set_baud_rate(&context->hal, baud_rate);
  taskEXIT_CRITICAL(&context->spinlock);

  return baud_rate;
}

uint32_t dmx_get_baud_rate(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");

  dmx_context_t *const context = &dmx_context[dmx_num];

  taskENTER_CRITICAL(&context->spinlock);
  const uint32_t baud_rate = dmx_hal_get_baud_rate(&context->hal);
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
  if (driver->is_sending && driver->data.previous_type != RDM_NON_RDM_PACKET) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(&context->spinlock);
    return 0;
  } else if (dmx_hal_get_rts(&context->hal) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_hal_disable_interrupt(&context->hal, DMX_INTR_RX_ALL);
    dmx_hal_set_rts(&context->hal, 0);
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
  if (driver->is_sending && driver->data.previous_type != RDM_NON_RDM_PACKET) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(&context->spinlock);
    return 0;
  } else if (dmx_hal_get_rts(&context->hal) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_hal_disable_interrupt(&context->hal, DMX_INTR_RX_ALL);
    dmx_hal_set_rts(&context->hal, 0);
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
  if (driver->is_sending && driver->data.previous_type != RDM_NON_RDM_PACKET) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(&context->spinlock);
    return 0;
  } else if (dmx_hal_get_rts(&context->hal) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_hal_disable_interrupt(&context->hal, DMX_INTR_RX_ALL);
    dmx_hal_set_rts(&context->hal, 0);
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

size_t dmx_receive(dmx_port_t dmx_num, dmx_event_t *event, TickType_t timeout) {
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
    return 0;
  }

  // Set the RTS pin to read from the DMX bus
  taskENTER_CRITICAL(&context->spinlock);
  if (dmx_hal_get_rts(&context->hal) == 0) {
    dmx_hal_disable_interrupt(&context->hal, DMX_INTR_TX_ALL);
    dmx_hal_set_rts(&context->hal, 1);
    dmx_hal_enable_interrupt(&context->hal, DMX_INTR_RX_ALL);
  }
  taskEXIT_CRITICAL(&context->spinlock);

  // Receive the latest data packet or check if the driver must wait
  dmx_err_t err = DMX_OK;
  uint32_t packet_size = 0;
  taskENTER_CRITICAL(&context->spinlock);
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
  taskEXIT_CRITICAL(&context->spinlock);

  // If a packet hasn't been received, the driver must wait
  if (!received_packet) {
    // Determine if a fail-quick timeout must be set
    uint32_t timeout = 0;
    if (sent_previous &&
        (previous_uid != RDM_BROADCAST_UID ||
         previous_type == RDM_DISCOVERY_COMMAND) &&
        (previous_type == RDM_GET_COMMAND || previous_type == RDM_SET_COMMAND ||
         previous_type == RDM_DISCOVERY_COMMAND)) {
      timeout = RDM_CONTROLLER_RESPONSE_LOST_TIMEOUT;
    }

    // Set the timeout alarm if the timeout hasn't elapsed yet
    taskENTER_CRITICAL(&context->spinlock);
    const int64_t elapsed = esp_timer_get_time() - previous_ts;
    if (elapsed < timeout) {
      timer_set_counter_value(driver->timer_group, driver->timer_num, elapsed);
      timer_set_alarm_value(driver->timer_group, driver->timer_num, timeout);
      timer_start(driver->timer_group, driver->timer_num);
      driver->timer_running = true;
    }
    taskEXIT_CRITICAL(&context->spinlock);

    // Fail immediately if the timeout has elapsed and a response was expected
    if (timeout > 0 && elapsed >= timeout) {
      taskENTER_CRITICAL(&context->spinlock);
      driver->task_waiting = NULL;
      taskEXIT_CRITICAL(&context->spinlock);
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
    taskENTER_CRITICAL(&context->spinlock);
    bool is_rdm = rdm_parse(driver->data.buffer, packet_size, &event->rdm);
    event->sc = driver->data.buffer[0];
    taskEXIT_CRITICAL(&context->spinlock);
    event->err = err;
    event->size = packet_size;
    event->is_rdm = is_rdm;
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
    return 0;
  }

  // Determine if it is too late to send a response packet
  int64_t elapsed = 0;
  taskENTER_CRITICAL(&context->spinlock);
  rdm_data_t *const rdm = (rdm_data_t *)driver->data.buffer;
  if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC &&
      (rdm->cc == RDM_DISCOVERY_COMMAND_RESPONSE ||
       rdm->cc == RDM_GET_COMMAND_RESPONSE ||
       rdm->cc == RDM_SET_COMMAND_RESPONSE)) {
    elapsed = esp_timer_get_time() - driver->data.previous_ts;
  }
  taskEXIT_CRITICAL(&context->spinlock);
  if (elapsed >= RDM_RESPONDER_RESPONSE_LOST_TIMEOUT) {
    return 0;
  }

  // Determine if an alarm needs to be set to wait until driver is ready
  uint32_t timeout = 0;
  taskENTER_CRITICAL(&context->spinlock);
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
  elapsed = esp_timer_get_time() - driver->data.previous_ts;
  if (elapsed < timeout) {
    timer_set_counter_value(driver->timer_group, driver->timer_num, elapsed);
    timer_set_alarm_value(driver->timer_group, driver->timer_num, timeout);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
    timer_start(driver->timer_group, driver->timer_num);
    driver->timer_running = true;
  }
  taskEXIT_CRITICAL(&context->spinlock);

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
  taskENTER_CRITICAL(&context->spinlock);
  if (dmx_hal_get_rts(&context->hal) == 1) {
    dmx_hal_disable_interrupt(&context->hal, DMX_INTR_RX_ALL);
    dmx_hal_set_rts(&context->hal, 0);
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
  const uint8_t sc = driver->data.buffer[0];  // DMX start code.
  if (sc == RDM_SC && driver->data.buffer[1] == RDM_SUB_SC) {
    const rdm_data_t *rdm = (rdm_data_t *)driver->data.buffer;
    driver->data.previous_type = rdm->cc;
    driver->data.previous_uid = buf_to_uid(rdm->destination_uid);
    ++driver->rdm_tn;
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
    taskENTER_CRITICAL(&context->spinlock);
    driver->is_sending = true;

    size_t write_size = driver->data.tx_size;
    dmx_hal_write_txfifo(&context->hal, driver->data.buffer, &write_size);
    driver->data.head = write_size;
    
    // Enable DMX write interrupts
    dmx_hal_enable_interrupt(&context->hal, DMX_INTR_TX_ALL);
    taskEXIT_CRITICAL(&context->spinlock);
  } else {
    // Use the hardware timer to send a DMX break and mark-after-break
    taskENTER_CRITICAL(&context->spinlock);
    const uint32_t break_len = driver->break_len;
    taskEXIT_CRITICAL(&context->spinlock);
    timer_set_counter_value(driver->timer_group, driver->timer_num, 0);
    timer_set_alarm_value(driver->timer_group, driver->timer_num, break_len);
    
    // Send the packet by starting the DMX break
    taskENTER_CRITICAL(&context->spinlock);
    driver->is_sending = true;
    driver->is_in_break = true;
    driver->data.head = 0;
    dmx_hal_invert_tx(&context->hal, 1);
    timer_start(driver->timer_group, driver->timer_num);
    driver->timer_running = true;
    taskEXIT_CRITICAL(&context->spinlock);
  }

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return size;
}

bool dmx_wait_sent(dmx_port_t dmx_num, TickType_t timeout) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_context_t *const context = &dmx_context[dmx_num];

  // Block until the mutex can be taken
  if (!xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY)) {
    return false;
  }

  // Determine if the task needs to block
  bool result = true;
  if (timeout > 0) {
    bool task_waiting = false;
    taskENTER_CRITICAL(&context->spinlock);
    if (driver->is_sending) {
      driver->task_waiting = xTaskGetCurrentTaskHandle();
      task_waiting = true;
    }
    taskEXIT_CRITICAL(&context->spinlock);

    // Wait for a notification that the driver is done sending
    if (task_waiting) {
      result = xTaskNotifyWait(0, ULONG_MAX, NULL, timeout);
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
