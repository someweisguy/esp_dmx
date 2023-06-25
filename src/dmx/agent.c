#include "dmx/agent.h"

#include "dmx/driver.h"
#include "dmx/hal.h"
#include "dmx/sniffer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "endian.h"
#include "rdm/responder.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/gptimer.h"
#include "esp_mac.h"
#include "esp_private/periph_ctrl.h"
#include "esp_timer.h"
#else
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#endif

static const char *TAG = "dmx";

enum dmx_default_interrupt_values_t {
  DMX_UART_FULL_DEFAULT = 1,   // RX FIFO full default interrupt threshold.
  DMX_UART_EMPTY_DEFAULT = 8,  // TX FIFO empty default interrupt threshold.
};

DRAM_ATTR dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {0};
DRAM_ATTR spinlock_t dmx_spinlock[DMX_NUM_MAX] = {portMUX_INITIALIZER_UNLOCKED,
                                                  portMUX_INITIALIZER_UNLOCKED,
#if DMX_NUM_MAX > 2
                                                  portMUX_INITIALIZER_UNLOCKED
#endif
};

static void DMX_ISR_ATTR dmx_uart_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = arg;
  spinlock_t *const restrict spinlock = &dmx_spinlock[driver->dmx_num];
  uart_dev_t *const restrict uart = driver->uart;
  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_uart_get_interrupt_status(uart);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_ALL) {
      // Stop the RDM receive alarm (which may or may not be running)
      if (driver->flags & DMX_FLAGS_TIMER_IS_RUNNING) {
#if ESP_IDF_VERSION_MAJOR >= 5
        gptimer_stop(driver->gptimer_handle);
#else
        timer_group_set_counter_enable_in_isr(driver->timer_group,
                                              driver->timer_idx, 0);
#endif
        driver->flags &= ~DMX_FLAGS_TIMER_IS_RUNNING;
      }

      // Read data into the DMX buffer if there is enough space
      const bool is_in_break = intr_flags & DMX_INTR_RX_BREAK;
      if (driver->data.head >= 0 && driver->data.head < DMX_MAX_PACKET_SIZE) {
        int read_len = DMX_MAX_PACKET_SIZE - driver->data.head;
        if (is_in_break) --read_len;  // Ignore DMX breaks

        // Read the UART data into the DMX buffer and increment the head
        uint8_t *current_slot = &driver->data.buffer[driver->data.head];
        dmx_uart_read_rxfifo(uart, current_slot, &read_len);
        driver->data.head += read_len;

        // Handle receiving a valid packet with larger than expected size
        if (driver->data.head > driver->data.rx_size) {
          driver->data.rx_size = driver->data.head;
        }

      } else {
        // Record the number of slots received for error reporting
        if (driver->data.head > 0) {
          driver->data.head += dmx_uart_get_rxfifo_len(uart);
        }
        dmx_uart_rxfifo_reset(uart);
      }

      // Handle DMX break condition
      if (is_in_break) {
        // Handle receiveing a valid packet with smaller than expected size
        if (!(driver->flags & DMX_FLAGS_DRIVER_IS_IDLE) &&
            driver->data.head > 0 && driver->data.head < DMX_MAX_PACKET_SIZE) {
          driver->data.rx_size = driver->data.head - 1;
        }

        // Set driver flags
        taskENTER_CRITICAL_ISR(spinlock);
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IDLE;
        driver->data.head = 0;          // Driver is ready for data
        taskEXIT_CRITICAL_ISR(spinlock);
      }

      // Set data timestamp, DMX break flag, and clear interrupts
      taskENTER_CRITICAL_ISR(spinlock);
      driver->data.timestamp = now;
      if (is_in_break) {
        driver->flags |= DMX_FLAGS_DRIVER_IS_IN_BREAK;
      } else {
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;
      }
      taskEXIT_CRITICAL_ISR(spinlock);
      dmx_uart_clear_interrupt(uart, DMX_INTR_RX_ALL);

      // Don't process data if end-of-packet condition already reached
      if (driver->flags & DMX_FLAGS_DRIVER_IS_IDLE) {
        continue;
      }



      // Handle DMX errors or process DMX data
      esp_err_t packet_err = ESP_OK;
      int rdm_type = 0;
      if (intr_flags & DMX_INTR_RX_ERR) {
        packet_err = intr_flags & DMX_INTR_RX_FRAMING_ERR
                         ? ESP_FAIL               // Missing stop bits
                         : ESP_ERR_NOT_FINISHED;  // UART overflow
      } else if (intr_flags & DMX_INTR_RX_DATA) {
        // TODO: if driver->data.head >= 17, call rdm_read(dmx_num, &header,
        // NULL, 0)
        // TODO: rdm_read() must be declared DMX_ISR_ATTR

        // Check if a full packet has been received and process packet data
        if (*(uint16_t *)driver->data.buffer == (RDM_SC | (RDM_SUB_SC << 8))) {
          if (driver->data.head < 26 ||
              driver->data.head < driver->data.buffer[2] + 2) {
            continue;  // Haven't received RDM packet yet
          }
          rdm_type |= DMX_FLAGS_RDM_IS_VALID;
          const rdm_pid_t pid = bswap16(*(uint16_t *)&driver->data.buffer[21]);
          rdm_uid_t dest_uid;
          uidcpy(&dest_uid, &driver->data.buffer[3]);
          const rdm_cc_t cc = driver->data.buffer[20];
          if (cc == RDM_CC_DISC_COMMAND || cc == RDM_CC_GET_COMMAND ||
              cc == RDM_CC_SET_COMMAND) {
            rdm_type |= DMX_FLAGS_RDM_IS_REQUEST;
          }
          if (uid_is_broadcast(&dest_uid)) {
            rdm_type |= DMX_FLAGS_RDM_IS_BROADCAST;
          }
          if (pid == RDM_PID_DISC_UNIQUE_BRANCH) {
            rdm_type |= DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
          }
          rdm_uid_t my_uid;
          uid_get(driver->dmx_num, &my_uid);
          if (uid_is_target(&my_uid, &dest_uid)) {
            rdm_type |= DMX_FLAGS_RDM_IS_RECIPIENT;
          }
        } else if ((*(uint8_t *)driver->data.buffer == RDM_PREAMBLE ||
                    *(uint8_t *)driver->data.buffer == RDM_DELIMITER)) {
          rdm_type |= DMX_FLAGS_RDM_IS_VALID;
          size_t preamble_len = 0;
          for (; preamble_len <= 7; ++preamble_len) {
            if (driver->data.buffer[preamble_len] == RDM_DELIMITER) break;
          }
          if (preamble_len <= 7) {
            if (driver->data.head < preamble_len + 17) {
              continue;  // Haven't received DISC_UNIQUE_BRANCH response yet
            }
            rdm_type |= DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
          } else {
            // When preamble_len > 7 the packet should not be considered RDM
            if (driver->data.head < driver->data.rx_size) {
              continue;  // Haven't received DMX packet yet
            }
          }
        } else {
          if (driver->data.head < driver->data.rx_size) {
            continue;  // Haven't received full DMX packet yet
          }
        }
      } else {
        // This code should never run, but can prevent crashes just in case!
        dmx_uart_disable_interrupt(uart, ~(DMX_INTR_RX_ALL | DMX_INTR_TX_ALL));
        dmx_uart_clear_interrupt(uart, ~(DMX_INTR_RX_ALL | DMX_INTR_TX_ALL));
        continue;
      }

      // Set driver flags and notify task
      taskENTER_CRITICAL_ISR(spinlock);
      driver->flags |= (DMX_FLAGS_DRIVER_HAS_DATA | DMX_FLAGS_DRIVER_IS_IDLE);
      driver->flags &= ~DMX_FLAGS_DRIVER_SENT_LAST;
      driver->rdm_type = rdm_type;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, packet_err,
                           eSetValueWithOverwrite, &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(spinlock);
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // Write data to the UART and clear the interrupt
      size_t write_size = driver->data.tx_size - driver->data.head;
      const uint8_t *src = &driver->data.buffer[driver->data.head];
      dmx_uart_write_txfifo(uart, src, &write_size);
      driver->data.head += write_size;
      dmx_uart_clear_interrupt(uart, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->data.head == driver->data.tx_size) {
        dmx_uart_disable_interrupt(uart, DMX_INTR_TX_DATA);
      }
    }

    else if (intr_flags & DMX_INTR_TX_DONE) {
      // Disable write interrupts and clear the interrupt
      dmx_uart_disable_interrupt(uart, DMX_INTR_TX_ALL);
      dmx_uart_clear_interrupt(uart, DMX_INTR_TX_DONE);

      // Record timestamp, unset sending flag, and notify task
      taskENTER_CRITICAL_ISR(spinlock);
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_SENDING;
      driver->data.timestamp = now;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, ESP_OK, eNoAction,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(spinlock);

      // Turn DMX bus around quickly if expecting an RDM response
      bool expecting_response = false;
      if (driver->rdm_type & DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH) {
        expecting_response = true;
        driver->data.head = 0;  // Not expecting a DMX break
      } else if (driver->rdm_type & DMX_FLAGS_RDM_IS_REQUEST) {
        expecting_response = true;
        driver->data.head = -1;  // Expecting a DMX break
      }
      if (expecting_response) {
        taskENTER_CRITICAL_ISR(spinlock);
        driver->flags &=
            ~(DMX_FLAGS_DRIVER_IS_IDLE | DMX_FLAGS_DRIVER_HAS_DATA);
        dmx_uart_rxfifo_reset(uart);
        dmx_uart_set_rts(uart, 1);
        taskEXIT_CRITICAL_ISR(spinlock);
      }
    }
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

static bool DMX_ISR_ATTR dmx_timer_isr(
#if ESP_IDF_VERSION_MAJOR >= 5
    gptimer_handle_t gptimer_handle,
    const gptimer_alarm_event_data_t *event_data,
#endif
    void *arg) {
  dmx_driver_t *const restrict driver = (dmx_driver_t *)arg;
  int task_awoken = false;

  if (driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) {
    if (driver->flags & DMX_FLAGS_DRIVER_IS_IN_BREAK) {
      dmx_uart_invert_tx(driver->uart, 0);
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;

      // Reset the alarm for the end of the DMX mark-after-break
#if ESP_IDF_VERSION_MAJOR >= 5
      const gptimer_alarm_config_t alarm_config = {
          .alarm_count = driver->mab_len, .flags.auto_reload_on_alarm = false};
      gptimer_set_alarm_action(gptimer_handle, &alarm_config);
#else
      timer_group_set_alarm_value_in_isr(driver->timer_group, driver->timer_idx,
                                         driver->mab_len);
#endif
    } else {
      // Write data to the UART
      size_t write_size = driver->data.tx_size;
      dmx_uart_write_txfifo(driver->uart, driver->data.buffer, &write_size);
      driver->data.head += write_size;

      // Pause MAB timer alarm
#if ESP_IDF_VERSION_MAJOR >= 5
      gptimer_stop(gptimer_handle);
      gptimer_set_raw_count(gptimer_handle, 0);
#else
      timer_group_set_counter_enable_in_isr(driver->timer_group,
                                            driver->timer_idx, 0);
#endif
      driver->flags &= ~ DMX_FLAGS_TIMER_IS_RUNNING;

      // Enable DMX write interrupts
      dmx_uart_enable_interrupt(driver->uart, DMX_INTR_TX_ALL);
    }
  } else if (driver->task_waiting) {
    // Notify the task
    xTaskNotifyFromISR(driver->task_waiting, ESP_OK, eSetValueWithOverwrite,
                       &task_awoken);

    // Pause the receive timer alarm
#if ESP_IDF_VERSION_MAJOR >= 5
    gptimer_stop(gptimer_handle);
    gptimer_set_raw_count(gptimer_handle, 0);
#else
    timer_group_set_counter_enable_in_isr(driver->timer_group,
                                          driver->timer_idx, 0);
#endif
    driver->flags &= ~DMX_FLAGS_TIMER_IS_RUNNING;
  }

  return task_awoken;
}

static void rdm_default_identify_cb(dmx_port_t dmx_num, bool identify,
                                    void *context) {
#ifdef ARDUINO
  printf("RDM identify device is %s\n", identify ? "on" : "off");
#else
  ESP_LOGI("rdm_responder", "RDM identify device is %s",
           identify ? "on" : "off");
#endif
}

esp_err_t dmx_driver_install(dmx_port_t dmx_num, dmx_config_t *config,
                             int intr_flags) {
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

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *restrict driver;

  // Allocate the DMX driver dynamically
  driver = heap_caps_malloc(sizeof(dmx_driver_t), MALLOC_CAP_8BIT);
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
  driver->uart = UART_LL_GET_HW(dmx_num);
  driver->flags = DMX_FLAGS_DRIVER_IS_ENABLED | DMX_FLAGS_DRIVER_IS_IDLE;
  driver->rdm_type = 0;

  // Initialize RDM settings
  driver->rdm.tn = 0;
  driver->rdm.num_cbs = 0;

  // Initialize RDM device info
  if (config->personality_count == 0 ||
      config->personality_count > CONFIG_DMX_MAX_PERSONALITIES) {
    ESP_LOGW(TAG, "Personality count is invalid, using default personality");
    config->personalities[0].footprint = 1;
    config->personalities[0].description = "Default Personality";
    config->current_personality = 1;
    config->personality_count = 1;
  }
  if (config->current_personality == 0) {
    ;
    config->current_personality = 1;  // TODO: Check NVS for value
  }
  if (config->current_personality > config->personality_count) {
    ESP_LOGW(TAG, "Current personality is invalid, using personality 1");
    config->current_personality = 1;
  }
  const int footprint_num = config->current_personality - 1;
  uint16_t footprint = config->personalities[footprint_num].footprint;
  if (footprint == 0) {
    config->dmx_start_address = 0xffff;
  }
  if (config->dmx_start_address == 0) {
    config->dmx_start_address = 1;  // TODO: Check NVS for value
  }
  driver->rdm.device_info = (rdm_device_info_t){
      .model_id = config->model_id,
      .product_category = config->product_category,
      .software_version_id = config->software_version_id,
      .footprint = footprint,
      .current_personality = config->current_personality,
      .personality_count = config->personality_count,
      .dmx_start_address = config->dmx_start_address,
      .sub_device_count = 0,  // Sub-devices must be registered
      .sensor_count = 0       // Sensors must be registered
  };
  memcpy(driver->personalities, config->personalities,
         sizeof(config->personalities[0]) * config->personality_count);

  // Initialize the driver buffer
  bzero(driver->data.buffer, DMX_MAX_PACKET_SIZE);
  driver->data.timestamp = 0;
  driver->data.head = -1;  // Wait for DMX break before reading data
  driver->data.rx_size = DMX_MAX_PACKET_SIZE;

  // Initialize DMX transmit settings
  driver->break_len = RDM_BREAK_LEN_US;
  driver->mab_len = RDM_MAB_LEN_US;

  // Initialize sniffer in the disabled state
  driver->sniffer.queue = NULL;

  // Initialize the UART peripheral
  uart_dev_t *const restrict uart = driver->uart;
  taskENTER_CRITICAL(spinlock);
  periph_module_enable(uart_periph_signal[dmx_num].module);
  if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
#if SOC_UART_REQUIRE_CORE_RESET
    // ESP32-C3 workaround to prevent UART outputting garbage data.
    uart_ll_set_reset_core(uart, true);
    periph_module_reset(uart_periph_signal[dmx_num].module);
    uart_ll_set_reset_core(uart, false);
#else
    periph_module_reset(uart_periph_signal[dmx_num].module);
#endif
  }
  taskEXIT_CRITICAL(spinlock);

  // Initialize and flush the UART
  dmx_uart_init(uart);
  dmx_uart_rxfifo_reset(uart);
  dmx_uart_txfifo_reset(uart);

  // Install UART interrupt
  dmx_uart_disable_interrupt(uart, DMX_ALL_INTR_MASK);
  dmx_uart_clear_interrupt(uart, DMX_ALL_INTR_MASK);
  dmx_uart_set_txfifo_empty(uart, DMX_UART_EMPTY_DEFAULT);
  dmx_uart_set_rxfifo_full(uart, DMX_UART_FULL_DEFAULT);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, intr_flags, &dmx_uart_isr,
                 driver, &driver->uart_isr_handle);

  // Initialize hardware timer
#if ESP_IDF_VERSION_MAJOR >= 5
  const gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_APB,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000,  // 1MHz resolution timer
  };
  esp_err_t err = gptimer_new_timer(&timer_config, &driver->gptimer_handle);
  if (err) {
    ESP_LOGE(TAG, "DMX driver gptimer error");
    dmx_driver_delete(dmx_num);
    return err;
  }
  const gptimer_event_callbacks_t gptimer_cb = {.on_alarm = dmx_timer_isr};
  gptimer_register_event_callbacks(driver->gptimer_handle, &gptimer_cb, driver);
  gptimer_enable(driver->gptimer_handle);
#else
  driver->timer_group = dmx_num / 2;
#ifdef CONFIG_IDF_TARGET_ESP32C3
  driver->timer_idx = 0;  // Timer 1 is the watchdog on ESP32C3
#else
  driver->timer_idx = dmx_num % 2;
#endif
  const timer_config_t timer_config = {
      .divider = 80,  // (80MHz / 80) == 1MHz resolution timer
      .counter_dir = TIMER_COUNT_UP,
      .counter_en = false,
      .alarm_en = true,
      .auto_reload = true,
  };
  timer_init(driver->timer_group, driver->timer_idx, &timer_config);
  timer_isr_callback_add(driver->timer_group, driver->timer_idx, dmx_timer_isr,
                         driver, intr_flags);
#endif

  const char *software_version_label =
      "esp_dmx v" __XSTRING(ESP_DMX_VERSION_MAJOR) "." __XSTRING(
          ESP_DMX_VERSION_MINOR) "." __XSTRING(ESP_DMX_VERSION_PATCH);

  // Add required RDM response callbacks
  rdm_register_disc_unique_branch(dmx_num);
  rdm_register_disc_un_mute(dmx_num);
  rdm_register_disc_mute(dmx_num);
  rdm_register_device_info(dmx_num, &dmx_driver[dmx_num]->rdm.device_info);
  rdm_register_software_version_label(dmx_num, software_version_label);
  rdm_register_identify_device(dmx_num, rdm_default_identify_cb, NULL);
  void *start_address = &dmx_driver[dmx_num]->rdm.device_info.dmx_start_address;
  rdm_register_dmx_start_address(dmx_num, start_address);
  // TODO: rdm_register_supported_parameters()

  // Enable UART read interrupt and set RTS low
  taskENTER_CRITICAL(spinlock);
  xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
  dmx_uart_enable_interrupt(uart, DMX_INTR_RX_ALL);
  dmx_uart_set_rts(uart, 1);
  taskEXIT_CRITICAL(spinlock);

  // Give the mutex and return
  xSemaphoreGiveRecursive(driver->mux);
  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
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
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_disable(driver->gptimer_handle);
  gptimer_del_timer(driver->gptimer_handle);
#else
  timer_isr_callback_remove(driver->timer_group, driver->timer_idx);
  timer_deinit(driver->timer_group, driver->timer_idx);
#endif

  // Free driver
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;

  // Disable UART module
  taskENTER_CRITICAL(spinlock);
  if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
    periph_module_disable(uart_periph_signal[dmx_num].module);
  }
  taskEXIT_CRITICAL(spinlock);

  return ESP_OK;
}

bool dmx_driver_is_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && dmx_driver[dmx_num] != NULL;
}

esp_err_t dmx_driver_disable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is not installed");
  DMX_CHECK(dmx_driver_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is already disabled");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  esp_err_t ret = ESP_ERR_NOT_FINISHED;

  // Disable receive interrupts
  taskENTER_CRITICAL(spinlock);
  if (!(driver->flags & DMX_FLAGS_DRIVER_IS_SENDING)) {
    dmx_uart_disable_interrupt(driver->uart, DMX_INTR_RX_ALL);
    dmx_uart_clear_interrupt(driver->uart, DMX_INTR_RX_ALL);
    driver->flags &= ~DMX_FLAGS_DRIVER_IS_ENABLED;
    ret = ESP_OK;
  }
  taskEXIT_CRITICAL(spinlock);

  return ret;
}

esp_err_t dmx_driver_enable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is not installed");
  DMX_CHECK(!dmx_driver_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is already enabled");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Initialize driver flags and reenable interrupts
  taskENTER_CRITICAL(spinlock);
  driver->data.head = -1;  // Wait for DMX break before reading data
  driver->flags |= (DMX_FLAGS_DRIVER_IS_ENABLED | DMX_FLAGS_DRIVER_IS_IDLE);
  driver->flags &= ~(DMX_FLAGS_DRIVER_IS_IN_BREAK | DMX_FLAGS_DRIVER_HAS_DATA);
  dmx_uart_rxfifo_reset(driver->uart);
  dmx_uart_txfifo_reset(driver->uart);
  dmx_uart_enable_interrupt(driver->uart, DMX_INTR_RX_ALL);
  dmx_uart_clear_interrupt(driver->uart, DMX_INTR_RX_ALL);
  taskEXIT_CRITICAL(spinlock);

  return ESP_OK;
}

bool dmx_driver_is_enabled(dmx_port_t dmx_num) {
  bool is_enabled = false;

  if (dmx_driver_is_installed(dmx_num)) {
    spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
    taskENTER_CRITICAL(spinlock);
    is_enabled = dmx_driver[dmx_num]->flags & DMX_FLAGS_DRIVER_IS_ENABLED;
    taskEXIT_CRITICAL(spinlock);
  }

  return is_enabled;
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

uint32_t dmx_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");

  // Clamp the baud rate to within DMX specification
  if (baud_rate < DMX_MIN_BAUD_RATE) {
    baud_rate = DMX_MIN_BAUD_RATE;
  } else if (baud_rate > DMX_MAX_BAUD_RATE) {
    baud_rate = DMX_MAX_BAUD_RATE;
  }

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  taskENTER_CRITICAL(spinlock);
  dmx_uart_set_baud_rate(dmx_driver[dmx_num]->uart, baud_rate);
  taskEXIT_CRITICAL(spinlock);

  return baud_rate;
}

uint32_t dmx_get_baud_rate(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  taskENTER_CRITICAL(spinlock);
  const uint32_t baud_rate = dmx_uart_get_baud_rate(dmx_driver[dmx_num]->uart);
  taskEXIT_CRITICAL(spinlock);

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

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  taskENTER_CRITICAL(spinlock);
  dmx_driver[dmx_num]->break_len = break_len;
  taskEXIT_CRITICAL(spinlock);

  return break_len;
}

uint32_t dmx_get_break_len(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  taskENTER_CRITICAL(spinlock);
  const uint32_t break_len = dmx_driver[dmx_num]->break_len;
  taskEXIT_CRITICAL(spinlock);

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

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  taskENTER_CRITICAL(spinlock);
  dmx_driver[dmx_num]->mab_len = mab_len;
  taskEXIT_CRITICAL(spinlock);

  return mab_len;
}

uint32_t dmx_get_mab_len(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  taskENTER_CRITICAL(spinlock);
  const uint32_t mab_len = dmx_driver[dmx_num]->mab_len;
  taskEXIT_CRITICAL(spinlock);

  return mab_len;
}

uint8_t dmx_get_current_personality(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  taskENTER_CRITICAL(spinlock);
  const uint8_t current_personality =
      dmx_driver[dmx_num]->rdm.device_info.current_personality;
  taskEXIT_CRITICAL(spinlock);

  return current_personality;
}

void dmx_set_current_personality(dmx_port_t dmx_num, uint8_t num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  DMX_CHECK(num > 0 && num <= dmx_get_personality_count(dmx_num), ,
            "num error");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  const uint16_t footprint = dmx_get_footprint(dmx_num, num);
  taskENTER_CRITICAL(spinlock);
  driver->rdm.device_info.footprint = footprint;
  driver->rdm.device_info.current_personality = num;
  taskEXIT_CRITICAL(spinlock);

  if (footprint > 0 &&
      dmx_get_dmx_start_address(dmx_num) + footprint > DMX_MAX_PACKET_SIZE) {
    dmx_set_dmx_start_address(dmx_num, DMX_MAX_PACKET_SIZE - footprint);
  }

  // TODO: use NVS
}

uint8_t dmx_get_personality_count(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  return dmx_driver[dmx_num]->rdm.device_info.personality_count;
}

uint16_t dmx_get_footprint(dmx_port_t dmx_num, uint8_t num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(num > 0 && num <= dmx_get_personality_count(dmx_num), 0,
            "num error");
  return dmx_driver[dmx_num]->personalities[num - 1].footprint;
}

uint16_t dmx_get_dmx_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  uint16_t dmx_start_address;
  taskENTER_CRITICAL(spinlock);
  dmx_start_address = dmx_driver[dmx_num]->rdm.device_info.dmx_start_address;
  taskEXIT_CRITICAL(spinlock);

  return dmx_start_address;
}

void dmx_set_dmx_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  uint16_t f = dmx_get_footprint(dmx_num, dmx_get_current_personality(dmx_num));
  DMX_CHECK(
      dmx_start_address > 0 && dmx_start_address + f <= DMX_MAX_PACKET_SIZE, ,
      "dmx_start_address is invalid");
  DMX_CHECK(f > 0, , "cannot set DMX start address of this personality");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  taskENTER_CRITICAL(spinlock);
  dmx_driver[dmx_num]->rdm.device_info.dmx_start_address = dmx_start_address;
  taskEXIT_CRITICAL(spinlock);

  // TODO: use NVS
}

uint16_t dmx_get_sub_device_count(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  uint16_t sub_device_count;
  taskENTER_CRITICAL(spinlock);
  sub_device_count = dmx_driver[dmx_num]->rdm.device_info.sub_device_count;
  taskEXIT_CRITICAL(spinlock);

  return sub_device_count;
}

uint8_t dmx_get_sensor_count(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  uint16_t sensor_count;
  taskENTER_CRITICAL(spinlock);
  sensor_count = dmx_driver[dmx_num]->rdm.device_info.sensor_count;
  taskEXIT_CRITICAL(spinlock);

  return sensor_count;
}