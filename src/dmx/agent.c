#include "dmx/agent.h"

#include "dmx/caps.h"
#include "dmx/driver.h"
#include "dmx/hal.h"
#include "dmx/sniffer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "endian.h"
#include "nvs_flash.h"
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

#ifdef CONFIG_DMX_ISR_IN_IRAM
#define DMX_ISR_ATTR IRAM_ATTR
#else
#define DMX_ISR_ATTR
#endif

#define DMX_UART_FULL_DEFAULT 1
#define DMX_UART_EMPTY_DEFAULT 8

static const char *TAG = "dmx";

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
      // Stop the DMX driver hardware timer if it is running
      taskENTER_CRITICAL_ISR(spinlock);
      if (driver->flags & DMX_FLAGS_TIMER_IS_RUNNING) {
#if ESP_IDF_VERSION_MAJOR >= 5
        gptimer_stop(driver->gptimer_handle);
#else
        timer_group_set_counter_enable_in_isr(driver->timer_group,
                                              driver->timer_idx, 0);
#endif
        driver->flags &= ~DMX_FLAGS_TIMER_IS_RUNNING;
      }
      taskEXIT_CRITICAL_ISR(spinlock);

      // Read data into the DMX buffer if there is enough space
      const bool is_in_break = (intr_flags & DMX_INTR_RX_BREAK);
      taskENTER_CRITICAL_ISR(spinlock);
      if (driver->head >= 0 && driver->head < DMX_PACKET_SIZE_MAX) {
        int read_len = DMX_PACKET_SIZE_MAX - driver->head - is_in_break;
        dmx_uart_read_rxfifo(uart, &driver->data[driver->head], &read_len);
        driver->head += read_len;
        if (driver->head > driver->rx_size) {
          driver->rx_size = driver->head;  // Update expected rx_size
        }
      } else {
        if (driver->head > 0) {
          // Record the number of slots received for error reporting
          driver->head += dmx_uart_get_rxfifo_len(uart);
        }
        dmx_uart_rxfifo_reset(uart);
      }
      taskEXIT_CRITICAL_ISR(spinlock);

      // Handle DMX break condition
      if (is_in_break) {
        taskENTER_CRITICAL_ISR(spinlock);
        // Handle receiveing a valid packet with smaller than expected size
        if (!(driver->flags & DMX_FLAGS_DRIVER_IS_IDLE) && driver->head > 0 &&
            driver->head < DMX_PACKET_SIZE_MAX) {
          driver->rx_size = driver->head - 1;
        }

        // Set driver flags
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IDLE;
        driver->head = 0;  // Driver is ready for data
        taskEXIT_CRITICAL_ISR(spinlock);
      }

      // Set last slot timestamp, DMX break flag, and clear interrupts
      taskENTER_CRITICAL_ISR(spinlock);
      driver->last_slot_ts = now;
      if (is_in_break) {
        driver->flags |= DMX_FLAGS_DRIVER_IS_IN_BREAK;
      } else {
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;
      }
      const int dmx_flags = driver->flags;
      taskEXIT_CRITICAL_ISR(spinlock);
      dmx_uart_clear_interrupt(uart, DMX_INTR_RX_ALL);

      // Don't process data if end-of-packet condition already reached
      if (dmx_flags & DMX_FLAGS_DRIVER_IS_IDLE) {
        continue;
      }

      // Check for potential end-of-packet condition
      int rdm_type = 0;
      rdm_header_t header;
      esp_err_t err = ESP_OK;
      if (intr_flags & DMX_INTR_RX_ERR) {
        err = intr_flags & DMX_INTR_RX_FIFO_OVERFLOW
                  ? ESP_ERR_NOT_FINISHED  // UART overflow
                  : ESP_FAIL;             // Missing stop bits
      } else if (driver->head > 16 &&
                 driver->head == rdm_read(driver->dmx_num, &header, NULL, 0)) {
        rdm_type |= DMX_FLAGS_RDM_IS_VALID;
        rdm_uid_t my_uid;
        uid_get(driver->dmx_num, &my_uid);
        if (header.cc == RDM_CC_DISC_COMMAND ||
            header.cc == RDM_CC_GET_COMMAND ||
            header.cc == RDM_CC_SET_COMMAND) {
          rdm_type |= DMX_FLAGS_RDM_IS_REQUEST;
        }
        if (uid_is_broadcast(&header.dest_uid)) {
          rdm_type |= DMX_FLAGS_RDM_IS_BROADCAST;
        }
        if (header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
          rdm_type |= DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
        }
        if (uid_is_target(&my_uid, &header.dest_uid)) {
          rdm_type |= DMX_FLAGS_RDM_IS_RECIPIENT;
        }
      } else if (driver->head < driver->rx_size) {
        continue;
      }

      // Set driver flags and notify task
      taskENTER_CRITICAL_ISR(spinlock);
      driver->flags |= (DMX_FLAGS_DRIVER_HAS_DATA | DMX_FLAGS_DRIVER_IS_IDLE);
      driver->flags &= ~DMX_FLAGS_DRIVER_SENT_LAST;
      driver->rdm_type = rdm_type;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, err, eSetValueWithOverwrite,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(spinlock);
    }

    // DMX Transmit #####################################################
    else if (intr_flags & DMX_INTR_TX_DATA) {
      // Write data to the UART and clear the interrupt
      size_t write_size = driver->tx_size - driver->head;
      dmx_uart_write_txfifo(uart, &driver->data[driver->head], &write_size);
      driver->head += write_size;
      dmx_uart_clear_interrupt(uart, DMX_INTR_TX_DATA);

      // Allow FIFO to empty when done writing data
      if (driver->head == driver->tx_size) {
        dmx_uart_disable_interrupt(uart, DMX_INTR_TX_DATA);
      }
    } else if (intr_flags & DMX_INTR_TX_DONE) {
      // Disable write interrupts and clear the interrupt
      dmx_uart_disable_interrupt(uart, DMX_INTR_TX_ALL);
      dmx_uart_clear_interrupt(uart, DMX_INTR_TX_DONE);

      // Record timestamp, unset sending flag, and notify task
      taskENTER_CRITICAL_ISR(spinlock);
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_SENDING;
      driver->last_slot_ts = now;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, ESP_OK, eNoAction,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(spinlock);

      // Turn DMX bus around quickly if expecting an RDM response
      bool expecting_response = false;
      if (driver->rdm_type & DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH) {
        expecting_response = true;
        driver->head = 0;  // Not expecting a DMX break
      } else if (driver->rdm_type & DMX_FLAGS_RDM_IS_REQUEST) {
        expecting_response = true;
        driver->head = -1;  // Expecting a DMX break
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
      size_t write_size = driver->tx_size;
      dmx_uart_write_txfifo(driver->uart, driver->data, &write_size);
      driver->head += write_size;

      // Pause MAB timer alarm
#if ESP_IDF_VERSION_MAJOR >= 5
      gptimer_stop(gptimer_handle);
      gptimer_set_raw_count(gptimer_handle, 0);
#else
      timer_group_set_counter_enable_in_isr(driver->timer_group,
                                            driver->timer_idx, 0);
#endif
      driver->flags &= ~DMX_FLAGS_TIMER_IS_RUNNING;

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

static void rdm_default_identify_cb(dmx_port_t dmx_num,
                                    const rdm_header_t *header, void *context) {
  if (header->cc == RDM_CC_SET_COMMAND) {
    const uint8_t *identify =
        rdm_get_pid(dmx_num, RDM_PID_IDENTIFY_DEVICE, NULL);
#ifdef ARDUINO
    printf("RDM identify device is %s\n", *identify ? "on" : "off");
#else
    ESP_LOGI(TAG, "RDM identify device is %s", *identify ? "on" : "off");
#endif
  }
}

esp_err_t dmx_driver_install(dmx_port_t dmx_num, const dmx_config_t *config,
                             int intr_flags) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(!dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is already installed");
  DMX_CHECK(config->dmx_start_address < DMX_PACKET_SIZE_MAX ||
                config->dmx_start_address == DMX_START_ADDRESS_NONE,
            ESP_ERR_INVALID_ARG, "dmx_start_address error");
  DMX_CHECK((config->personality_count == 0 &&
             config->dmx_start_address == DMX_START_ADDRESS_NONE) ||
                (config->personality_count > 0 &&
                 config->personality_count < DMX_PERSONALITIES_MAX),
            false, "personality_count error");
  DMX_CHECK(config->current_personality <= config->personality_count,
            ESP_ERR_INVALID_ARG, "current_personality error");
  for (int i = 0; i < config->personality_count; ++i) {
    DMX_CHECK(config->personalities[i].footprint < DMX_PACKET_SIZE_MAX,
              ESP_ERR_INVALID_ARG, "footprint error");
  }

  // Initialize NVS
  nvs_flash_init_partition("nvs");  // TODO: allow rename partition in Kconfig

  // Initialize RDM UID
  uid_get(dmx_num, NULL);

#ifdef CONFIG_DMX_ISR_IN_IRAM
  // Driver ISR is in IRAM so interrupt flags must include IRAM flag
  if (!(intr_flags & ESP_INTR_FLAG_IRAM)) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    intr_flags |= ESP_INTR_FLAG_IRAM;
  }
#endif

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *restrict driver;

  // Allocate the DMX driver
  driver = heap_caps_malloc(sizeof(dmx_driver_t), MALLOC_CAP_8BIT);
  if (driver == NULL) {
    ESP_LOGE(TAG, "DMX driver malloc error");
    return ESP_ERR_NO_MEM;
  }
  dmx_driver[dmx_num] = driver;

  // Initialize hardware timer
#if ESP_IDF_VERSION_MAJOR >= 5
  gptimer_handle_t gptimer_handle;
  const gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_APB,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000,  // 1MHz resolution timer
  };
  esp_err_t err = gptimer_new_timer(&timer_config, &gptimer_handle);
  if (err) {
    ESP_LOGE(TAG, "DMX driver gptimer error");
    dmx_driver_delete(dmx_num);
    return err;
  }
#else
  timer_group_t timer_group = dmx_num / 2;
  timer_idx_t timer_idx;
#ifdef CONFIG_IDF_TARGET_ESP32C3
  timer_idx = 0;  // ESP32C3 uses timer_idx 1 for Watchdog
#else
  timer_idx = dmx_num % 2;
#endif
  const timer_config_t timer_config = {
      .divider = 80,  // (80MHz / 80) == 1MHz resolution timer
      .counter_dir = TIMER_COUNT_UP,
      .counter_en = false,
      .alarm_en = true,
      .auto_reload = true,
  };
  esp_err_t err = timer_init(timer_group, timer_idx, &timer_config);
  if (err) {
    ESP_LOGE(TAG, "DMX driver timer error");
    dmx_driver_delete(dmx_num);
    return err;
  }
#endif

  // Allocate mutex
  SemaphoreHandle_t mux = xSemaphoreCreateRecursiveMutex();
  if (mux == NULL) {
    ESP_LOGE(TAG, "DMX driver mutex malloc error");
    dmx_driver_delete(dmx_num);
    return ESP_ERR_NO_MEM;
  }

  // Allocate DMX buffer
  uint8_t *data = heap_caps_malloc(DMX_PACKET_SIZE, MALLOC_CAP_8BIT);
  if (data == NULL) {
    ESP_LOGE(TAG, "DMX driver buffer malloc error");
    dmx_driver_delete(dmx_num);
    return ESP_ERR_NO_MEM;
  }
  bzero(data, DMX_PACKET_SIZE_MAX);

  // Allocate the RDM parameter buffer
  bool enable_rdm;
  size_t alloc_size;
  if (config->alloc_size < 53) {
    // Allocate space for DMX start address and personality info
    alloc_size = sizeof(dmx_driver_personality_t);
    enable_rdm = false;
  } else {
    alloc_size = config->alloc_size;
    enable_rdm = false;
  }
  uint8_t *alloc_data = heap_caps_malloc(alloc_size, MALLOC_CAP_8BIT);
  if (alloc_data == NULL) {
    ESP_LOGE(TAG, "DMX driver RDM buffer malloc error");
    dmx_driver_delete(dmx_num);
    return ESP_ERR_NO_MEM;
  }

  // UART configuration
  driver->dmx_num = dmx_num;
  driver->uart = UART_LL_GET_HW(dmx_num);
  // The uart_isr_handle field is left uninitialized

// Hardware timer configuration
#if ESP_IDF_VERSION_MAJOR >= 5
  driver->gptimer_handle = gptimer_handle;
#else
  driver->timer_group = timer_group;
  driver->timer_idx = timer_idx;
#endif

  // Synchronization state
  driver->mux = mux;
  driver->task_waiting = NULL;

  // Data buffer
  driver->head = -1;
  driver->data = data;
  driver->tx_size = DMX_PACKET_SIZE_MAX;
  driver->rx_size = DMX_PACKET_SIZE_MAX;

  // Driver state
  driver->flags = (DMX_FLAGS_DRIVER_IS_ENABLED | DMX_FLAGS_DRIVER_IS_IDLE);
  driver->rdm_type = 0;
  driver->tn = 0;
  driver->last_slot_ts = 0;

  // DMX configuration
  driver->break_len = RDM_BREAK_LEN_US;
  driver->mab_len = RDM_MAB_LEN_US;

  driver->alloc_size = alloc_size;
  driver->alloc_data = alloc_data;
  driver->alloc_head = 0;

  // RDM responder configuration
  driver->num_rdm_cbs = 0;
  // The driver->rdm_cbs field is left uninitialized

  // DMX sniffer configuration
  // The driver->metadata field is left uninitialized
  driver->metadata_queue = NULL;
  driver->sniffer_pin = -1;
  driver->last_pos_edge_ts = -1;
  driver->last_neg_edge_ts = -1;

  // Copy the personality table to the driver
  memcpy(driver->personalities, config->personalities,
         sizeof(*config->personalities) * config->personality_count);

  // Configure required variables for RDM or DMX-only
  if (enable_rdm) {
    uint16_t footprint;
    if (config->personality_count > 0 && config->current_personality > 0) {
      const uint8_t personality_idx = config->current_personality - 1;
      footprint = driver->personalities[personality_idx].footprint;
    } else {
      footprint = 0;
    }
    rdm_device_info_t device_info = {
        .model_id = config->model_id,
        .product_category = config->product_category,
        .software_version_id = config->software_version_id,
        .footprint = footprint,
        .current_personality = config->current_personality,
        .personality_count = config->personality_count,
        .dmx_start_address = config->dmx_start_address,
        .sub_device_count = 0,  // Sub-devices must be registered
        .sensor_count = 0,      // Sensors must be registered
    };
    rdm_register_disc_unique_branch(dmx_num, NULL, NULL);
    rdm_register_disc_un_mute(dmx_num, NULL, NULL);
    rdm_register_disc_mute(dmx_num, NULL, NULL);
    rdm_register_device_info(dmx_num, &device_info, NULL, NULL);
    rdm_register_software_version_label(dmx_num, config->software_version_label,
                                        NULL, NULL);
    rdm_register_identify_device(dmx_num, rdm_default_identify_cb, NULL);
    if (device_info.dmx_start_address != DMX_START_ADDRESS_NONE) {
      rdm_register_dmx_start_address(dmx_num, NULL, NULL);
    }
    // TODO: rdm_register_supported_parameters()
  } else {
    dmx_driver_personality_t *dmx = rdm_alloc(dmx_num, alloc_size);
    assert(dmx != NULL);

    // Load the DMX start address from NVS
    uint16_t dmx_start_address;
    if (config->dmx_start_address == 0) {
      size_t size = sizeof(dmx_start_address);
      esp_err_t err =
          rdm_get_pid_from_nvs(dmx_num, RDM_PID_DMX_START_ADDRESS,
                               RDM_DS_UNSIGNED_WORD, &dmx_start_address, &size);
      if (err) {
        dmx_start_address = 1;
      }
    } else {
      dmx_start_address = config->dmx_start_address;
    }

    // Load the current DMX personality from NVS
    uint8_t current_personality;
    if (config->current_personality == 0 &&
        dmx_start_address != DMX_START_ADDRESS_NONE) {
      rdm_dmx_personality_t personality;
      size_t size = sizeof(personality);
      esp_err_t err =
          rdm_get_pid_from_nvs(dmx_num, RDM_PID_DMX_PERSONALITY,
                               RDM_DS_BIT_FIELD, &personality, &size);
      if (err || personality.personality_count != config->personality_count) {
        current_personality = 1;
      } else {
        current_personality = personality.current_personality;
      }
    } else {
      current_personality = config->current_personality;
    }

    dmx->dmx_start_address = dmx_start_address;
    dmx->current_personality = current_personality;
    dmx->personality_count = config->personality_count;
  }

  // Enable the UART peripheral
  taskENTER_CRITICAL(spinlock);
  periph_module_enable(uart_periph_signal[dmx_num].module);
  if (dmx_num != CONFIG_ESP_CONSOLE_UART_NUM) {
#if SOC_UART_REQUIRE_CORE_RESET
    // ESP32C3 workaround to prevent UART outputting garbage data
    uart_ll_set_reset_core(driver->uart, true);
    periph_module_reset(uart_periph_signal[dmx_num].module);
    uart_ll_set_reset_core(driver->uart, false);
#else
    periph_module_reset(uart_periph_signal[dmx_num].module);
#endif
  }
  taskEXIT_CRITICAL(spinlock);
  dmx_uart_init(driver->uart);
  dmx_uart_rxfifo_reset(driver->uart);
  dmx_uart_txfifo_reset(driver->uart);
  dmx_uart_disable_interrupt(driver->uart, DMX_ALL_INTR_MASK);
  dmx_uart_clear_interrupt(driver->uart, DMX_ALL_INTR_MASK);
  dmx_uart_set_txfifo_empty(driver->uart, DMX_UART_EMPTY_DEFAULT);
  dmx_uart_set_rxfifo_full(driver->uart, DMX_UART_FULL_DEFAULT);
  esp_intr_alloc(uart_periph_signal[dmx_num].irq, intr_flags, &dmx_uart_isr,
                 driver, &driver->uart_isr_handle);

  // Enable the hardware timer
#if ESP_IDF_VERSION_MAJOR >= 5
  const gptimer_event_callbacks_t gptimer_cb = {.on_alarm = dmx_timer_isr};
  gptimer_register_event_callbacks(driver->gptimer_handle, &gptimer_cb, driver);
  gptimer_enable(driver->gptimer_handle);
#else
  timer_isr_callback_add(driver->timer_group, driver->timer_idx, dmx_timer_isr,
                         driver, intr_flags);
#endif

  // Enable reading on the DMX port
  taskENTER_CRITICAL(spinlock);
  xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
  dmx_uart_enable_interrupt(driver->uart, DMX_INTR_RX_ALL);
  dmx_uart_set_rts(driver->uart, 1);
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
  if (driver->data != NULL) {
    heap_caps_free(driver->data);
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
  driver->head = -1;  // Wait for DMX break before reading data
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
  if (baud_rate < DMX_BAUD_RATE_MIN) {
    baud_rate = DMX_BAUD_RATE_MIN;
  } else if (baud_rate > DMX_BAUD_RATE_MAX) {
    baud_rate = DMX_BAUD_RATE_MAX;
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
  if (break_len < DMX_BREAK_LEN_MIN_US) {
    break_len = DMX_BREAK_LEN_MIN_US;
  } else if (break_len > DMX_BREAK_LEN_MAX_US) {
    break_len = DMX_BREAK_LEN_MAX_US;
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
  if (mab_len < DMX_MAB_LEN_MIN_US) {
    mab_len = DMX_MAB_LEN_MIN_US;
  } else if (mab_len > DMX_MAB_LEN_MAX_US) {
    mab_len = DMX_MAB_LEN_MAX_US;
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
  uint8_t current_personality;

  const rdm_device_info_t *device_info =
      rdm_get_pid(dmx_num, RDM_PID_DEVICE_INFO, NULL);
  taskENTER_CRITICAL(spinlock);
  if (device_info == NULL) {
    const dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->alloc_data;
    current_personality =  personality->current_personality;
  } else {
    current_personality =  device_info->current_personality;
  }
  taskEXIT_CRITICAL(spinlock);

  return current_personality;
}

bool dmx_set_current_personality(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num error");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];

  // Get the required personality values from RDM device info or DMX driver
  uint8_t *current_personality;
  uint16_t *footprint;
  rdm_device_info_t *device_info =
      rdm_get_pid(dmx_num, RDM_PID_DEVICE_INFO, NULL);
  if (device_info == NULL) {
    dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->alloc_data;
    current_personality = &personality->current_personality;
    footprint = NULL;
  } else {
    current_personality = &device_info->current_personality;
    footprint = (void *)device_info + offsetof(rdm_device_info_t, footprint);
  }

  // Set the new personality
  taskENTER_CRITICAL(spinlock);
  *current_personality = personality_num;
  if (footprint != NULL) {
    *footprint = dmx_get_footprint(dmx_num, personality_num);
  }
  taskEXIT_CRITICAL(spinlock);


  return true;
}

uint8_t dmx_get_personality_count(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_device_info_t *device_info =
      rdm_get_pid(dmx_num, RDM_PID_DEVICE_INFO, NULL);
  if (device_info == NULL) {
    const dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->alloc_data;
    return personality->personality_count;
  } else {
    return device_info->personality_count;
  }
}

size_t dmx_get_footprint(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            0, "personality_num is invalid");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];

  --personality_num;  // Personalities are indexed starting at 1
  taskENTER_CRITICAL(spinlock);
  size_t fp = dmx_driver[dmx_num]->personalities[personality_num].footprint;
  taskEXIT_CRITICAL(spinlock);

  return fp;
}

const char *dmx_get_personality_description(dmx_port_t dmx_num,
                                            uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, NULL, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), NULL, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            NULL, "personality_num is invalid");

  --personality_num;  // Personalities are indexed starting at 1
  return dmx_driver[dmx_num]->personalities[personality_num].description;
}

uint16_t dmx_get_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  uint16_t dmx_start_address;

  const rdm_device_info_t *device_info =
      rdm_get_pid(dmx_num, RDM_PID_DEVICE_INFO, NULL);
  taskENTER_CRITICAL(spinlock);
  if (device_info == NULL) {
    const dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->alloc_data;
    dmx_start_address = personality->dmx_start_address;
  } else {
    dmx_start_address = device_info->dmx_start_address;
  }
  taskEXIT_CRITICAL(spinlock);

  return dmx_start_address;
}

bool dmx_set_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_start_address > 0 && dmx_start_address < DMX_PACKET_SIZE_MAX,
            false, "dmx_start_address error");
  DMX_CHECK(dmx_get_start_address(dmx_num) != DMX_START_ADDRESS_NONE, false,
            "cannot set DMX start address");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];

  rdm_device_info_t *device_info =
      rdm_get_pid(dmx_num, RDM_PID_DEVICE_INFO, NULL);
  taskENTER_CRITICAL(spinlock);
  if (device_info == NULL) {
    dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->alloc_data;
    personality->dmx_start_address = dmx_start_address;
  } else {
    device_info->dmx_start_address = dmx_start_address;
  }
  taskEXIT_CRITICAL(spinlock);

  return true;
}