#include "dmx/hal.h"

#include <string.h>

#include "dmx/caps.h"
#include "dmx/nvs.h"
#include "dmx/struct.h"
#include "dmx/timer.h"
#include "dmx/types.h"
#include "dmx/uart.h"
#include "dmx/utils.h"
#include "endian.h"
#include "esp_dmx.h"
#include "esp_timer.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/gptimer.h"
#else
#include "driver/timer.h"
#endif

const char *TAG = "dmx";  // The log tagline for the library

enum dmx_interrupt_mask_t {
  DMX_INTR_RX_FIFO_OVERFLOW = UART_INTR_RXFIFO_OVF,
  DMX_INTR_RX_FRAMING_ERR = UART_INTR_PARITY_ERR | UART_INTR_FRAM_ERR,
  DMX_INTR_RX_ERR = DMX_INTR_RX_FIFO_OVERFLOW | DMX_INTR_RX_FRAMING_ERR,

  DMX_INTR_RX_BREAK = UART_INTR_BRK_DET,
  DMX_INTR_RX_DATA = UART_INTR_RXFIFO_FULL,
  DMX_INTR_RX_ALL = DMX_INTR_RX_DATA | DMX_INTR_RX_BREAK | DMX_INTR_RX_ERR,

  DMX_INTR_TX_DATA = UART_INTR_TXFIFO_EMPTY,
  DMX_INTR_TX_DONE = UART_INTR_TX_DONE,
  DMX_INTR_TX_ALL = DMX_INTR_TX_DATA | DMX_INTR_TX_DONE,
};

dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {};

static struct dmx_context_t {
  dmx_uart_handle_t uart;
  dmx_timer_handle_t timer;
} dmx_context[DMX_NUM_MAX] = {};

static void DMX_ISR_ATTR dmx_uart_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = arg;
  const dmx_port_t dmx_num = driver->dmx_num;
  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;
  dmx_timer_handle_t timer = dmx_context[dmx_num].timer;
  int task_awoken = false;

  while (true) {
    const uint32_t intr_flags = dmx_uart_get_interrupt_status(uart);
    if (intr_flags == 0) break;

    // DMX Receive ####################################################
    if (intr_flags & DMX_INTR_RX_ALL) {
      // Stop the DMX driver hardware timer if it is running
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      dmx_timer_stop(timer);
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

      // Read data into the DMX buffer if there is enough space
      const bool is_in_break = (intr_flags & DMX_INTR_RX_BREAK);
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
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
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

      // Handle DMX break condition
      if (is_in_break) {
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        // Handle receiveing a valid packet with smaller than expected size
        if (!(driver->flags & DMX_FLAGS_DRIVER_IS_IDLE) && driver->head > 0 &&
            driver->head < DMX_PACKET_SIZE_MAX) {
          driver->rx_size = driver->head - 1;
        }

        // Set driver flags
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IDLE;
        driver->head = 0;  // Driver is ready for data
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      }

      // Set last slot timestamp, DMX break flag, and clear interrupts
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->last_slot_ts = now;
      if (is_in_break) {
        driver->flags |= DMX_FLAGS_DRIVER_IS_IN_BREAK;
      } else {
        driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;
      }
      const int dmx_flags = driver->flags;
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
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
                 driver->head ==
                     dmx_read_rdm(driver->dmx_num, &header, NULL, 0)) {
        rdm_type |= DMX_FLAGS_RDM_IS_VALID;
        rdm_uid_t my_uid;
        rdm_uid_get(driver->dmx_num, &my_uid);
        if (header.cc == RDM_CC_DISC_COMMAND ||
            header.cc == RDM_CC_GET_COMMAND ||
            header.cc == RDM_CC_SET_COMMAND) {
          rdm_type |= DMX_FLAGS_RDM_IS_REQUEST;
        }
        if (rdm_uid_is_broadcast(&header.dest_uid)) {
          rdm_type |= DMX_FLAGS_RDM_IS_BROADCAST;
        }
        if (header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
          rdm_type |= DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
        }
        if (rdm_uid_is_target(&my_uid, &header.dest_uid)) {
          rdm_type |= DMX_FLAGS_RDM_IS_RECIPIENT;
        }
      } else if (driver->head < driver->rx_size) {
        continue;
      }

      // Set driver flags and notify task
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->flags |= (DMX_FLAGS_DRIVER_HAS_DATA | DMX_FLAGS_DRIVER_IS_IDLE);
      driver->flags &= ~DMX_FLAGS_DRIVER_SENT_LAST;
      driver->rdm_type = rdm_type;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, err, eSetValueWithOverwrite,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
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
      taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_SENDING;
      driver->last_slot_ts = now;
      if (driver->task_waiting) {
        xTaskNotifyFromISR(driver->task_waiting, ESP_OK, eNoAction,
                           &task_awoken);
      }
      taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));

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
        taskENTER_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
        driver->flags &=
            ~(DMX_FLAGS_DRIVER_IS_IDLE | DMX_FLAGS_DRIVER_HAS_DATA);
        dmx_uart_rxfifo_reset(uart);
        dmx_uart_set_rts(uart, 1);
        taskEXIT_CRITICAL_ISR(DMX_SPINLOCK(dmx_num));
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
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  dmx_uart_handle_t uart = dmx_context[driver->dmx_num].uart;
  dmx_timer_handle_t timer = dmx_context[driver->dmx_num].timer;
  int task_awoken = false;

  if (driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) {
    if (driver->flags & DMX_FLAGS_DRIVER_IS_IN_BREAK) {
      dmx_uart_invert_tx(uart, 0);
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_BREAK;

      // Reset the alarm for the end of the DMX mark-after-break
      dmx_timer_set_alarm(timer, driver->mab_len);
    } else {
      // Write data to the UART
      size_t write_size = driver->tx_size;
      dmx_uart_write_txfifo(uart, driver->data, &write_size);
      driver->head += write_size;

      // Pause MAB timer alarm
      dmx_timer_stop(timer);

      // Enable DMX write interrupts
      dmx_uart_enable_interrupt(uart, DMX_INTR_TX_ALL);
    }
  } else if (driver->task_waiting) {
    // Notify the task
    xTaskNotifyFromISR(driver->task_waiting, ESP_OK, eSetValueWithOverwrite,
                       &task_awoken);

    // Pause the receive timer alarm
    dmx_timer_stop(timer);
  }

  return task_awoken;
}

static void DMX_ISR_ATTR dmx_gpio_isr(void *arg) {
  const int64_t now = esp_timer_get_time();
  dmx_driver_t *const driver = (dmx_driver_t *)arg;
  const dmx_port_t dmx_num = ((dmx_driver_t *)arg)->dmx_num;
  int task_awoken = false;

  if (dmx_uart_get_rx_level(dmx_context[dmx_num].uart)) {
    /* If this ISR is called on a positive edge and the current DMX frame is in
    a break and a negative edge timestamp has been recorded then a break has
    just finished. Therefore the DMX break length is able to be recorded. It can
    also be deduced that the driver is now in a DMX mark-after-break. */

    if ((driver->flags & DMX_FLAGS_DRIVER_IS_IN_BREAK) &&
        driver->last_neg_edge_ts > -1) {
      driver->metadata.break_len = now - driver->last_neg_edge_ts;
      driver->flags |= DMX_FLAGS_DRIVER_IS_IN_BREAK;
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_MAB;
    }
    driver->last_pos_edge_ts = now;
  } else {
    /* If this ISR is called on a negative edge in a DMX mark-after-break then
    the DMX mark-after-break has just finished. It can be recorded. Sniffer data
    is now available to be read by the user. */

    if (driver->flags & DMX_FLAGS_DRIVER_IS_IN_MAB) {
      driver->metadata.mab_len = now - driver->last_pos_edge_ts;
      driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_MAB;

      // Send the sniffer data to the queue
      xQueueOverwriteFromISR(driver->metadata_queue, &driver->metadata,
                             &task_awoken);
    }
    driver->last_neg_edge_ts = now;
  }

  if (task_awoken) portYIELD_FROM_ISR();
}

static void rdm_default_identify_cb(dmx_port_t dmx_num,
                                    const rdm_header_t *header, void *context) {
  if (header->cc == RDM_CC_SET_COMMAND) {
    const uint8_t *identify = rdm_pd_find(dmx_num, RDM_PID_IDENTIFY_DEVICE);
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
  dmx_nvs_init(dmx_num);

  // Initialize RDM UID
  rdm_uid_get(dmx_num, NULL);

#ifdef DMX_ISR_IN_IRAM
  // Driver ISR is in IRAM so interrupt flags must include IRAM flag
  if (!(intr_flags & ESP_INTR_FLAG_IRAM)) {
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
    intr_flags |= ESP_INTR_FLAG_IRAM;
  }
#endif

  dmx_driver_t *restrict driver;

  // Allocate the DMX driver
  driver = heap_caps_malloc(sizeof(dmx_driver_t), MALLOC_CAP_8BIT);
  DMX_CHECK(driver != NULL, ESP_ERR_NO_MEM, "DMX driver malloc error");
  dmx_driver[dmx_num] = driver;
  driver->mux = NULL;
  driver->data = NULL;
  driver->pd = NULL;
  driver->spinlock = (spinlock_t)portMUX_INITIALIZER_UNLOCKED;

  // Allocate mutex
  SemaphoreHandle_t mux = xSemaphoreCreateRecursiveMutex();
  if (mux == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(mux != NULL, ESP_ERR_NO_MEM, "DMX driver mutex malloc error");
  }

  // Allocate DMX buffer
  uint8_t *data = heap_caps_malloc(DMX_PACKET_SIZE, MALLOC_CAP_8BIT);
  if (data == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(data != NULL, ESP_ERR_NO_MEM, "DMX driver buffer malloc error");
  }
  bzero(data, DMX_PACKET_SIZE_MAX);

  // Allocate the RDM parameter buffer
  bool enable_rdm;
  size_t pd_size;
  if (config->pd_size < 53) {
    // Allocate space for DMX start address and personality info
    pd_size = sizeof(dmx_driver_personality_t);
    enable_rdm = false;
  } else {
    pd_size = config->pd_size;
    enable_rdm = true;
  }
  uint8_t *pd = heap_caps_malloc(pd_size, MALLOC_CAP_8BIT);
  if (pd == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(pd != NULL, ESP_ERR_NO_MEM, "DMX driver pd buffer malloc error");
  }

  // UART configuration
  driver->dmx_num = dmx_num;

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

  driver->pd_size = pd_size;
  driver->pd = pd;
  driver->pd_head = 0;

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
    dmx_driver_personality_t *dmx = rdm_pd_alloc(dmx_num, pd_size);
    assert(dmx != NULL);

    // Load the DMX start address from NVS
    uint16_t dmx_start_address;
    if (config->dmx_start_address == 0) {
      size_t size = sizeof(dmx_start_address);
      if (!dmx_nvs_get(dmx_num, RDM_PID_DMX_START_ADDRESS, RDM_DS_UNSIGNED_WORD,
                       &dmx_start_address, &size)) {
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
      if (!dmx_nvs_get(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_DS_BIT_FIELD,
                       &personality, &size) ||
          personality.personality_count != config->personality_count) {
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
  dmx_uart_handle_t uart =
      dmx_uart_init(dmx_num, dmx_uart_isr, driver, intr_flags);
  if (uart == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(uart != NULL, ESP_FAIL, "UART init error");
  }
  dmx_context[dmx_num].uart = uart;

  dmx_timer_handle_t timer =
      dmx_timer_init(dmx_num, dmx_timer_isr, driver, intr_flags);
  if (timer == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(timer != NULL, ESP_FAIL, "timer init error");
  }
  dmx_context[dmx_num].timer = timer;

  // Enable reading on the DMX port
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
  dmx_uart_enable_interrupt(uart, DMX_INTR_RX_ALL);
  dmx_uart_set_rts(uart, 1);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Give the mutex and return
  xSemaphoreGiveRecursive(driver->mux);
  return ESP_OK;
}

esp_err_t dmx_driver_delete(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is not installed");

  dmx_timer_handle_t timer = dmx_context[dmx_num].timer;
  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Free driver mutex
  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return ESP_FAIL;
  }
  xSemaphoreGiveRecursive(driver->mux);
  vSemaphoreDelete(driver->mux);

  // Uninstall sniffer ISR
  if (dmx_sniffer_is_enabled(dmx_num)) {
    dmx_sniffer_disable(dmx_num);
  }

  // Free driver data buffer
  if (driver->data != NULL) {
    heap_caps_free(driver->data);
  }

  // Free RDM parameter data buffer
  if (driver->pd != NULL) {
    heap_caps_free(driver->pd);
  }

  // Free hardware timer ISR
  dmx_timer_deinit(timer);
  dmx_context[dmx_num].timer = NULL;

  // Disable UART module
  dmx_uart_deinit(uart);
  dmx_context[dmx_num].uart = NULL;

  // Free driver
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;

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

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;

  esp_err_t ret = ESP_ERR_NOT_FINISHED;

  // Disable receive interrupts
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (!(driver->flags & DMX_FLAGS_DRIVER_IS_SENDING)) {
    dmx_uart_disable_interrupt(uart, DMX_INTR_RX_ALL);
    dmx_uart_clear_interrupt(uart, DMX_INTR_RX_ALL);
    driver->flags &= ~DMX_FLAGS_DRIVER_IS_ENABLED;
    ret = ESP_OK;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return ret;
}

esp_err_t dmx_driver_enable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is not installed");
  DMX_CHECK(!dmx_driver_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "driver is already enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;

  // Initialize driver flags and reenable interrupts
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  driver->head = -1;  // Wait for DMX break before reading data
  driver->flags |= (DMX_FLAGS_DRIVER_IS_ENABLED | DMX_FLAGS_DRIVER_IS_IDLE);
  driver->flags &= ~(DMX_FLAGS_DRIVER_IS_IN_BREAK | DMX_FLAGS_DRIVER_HAS_DATA);
  dmx_uart_rxfifo_reset(uart);
  dmx_uart_txfifo_reset(uart);
  dmx_uart_enable_interrupt(uart, DMX_INTR_RX_ALL);
  dmx_uart_clear_interrupt(uart, DMX_INTR_RX_ALL);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return ESP_OK;
}

esp_err_t dmx_set_pin(dmx_port_t dmx_num, int tx_pin, int rx_pin, int rts_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(DMX_TX_PIN_IS_VALID(tx_pin), ESP_ERR_INVALID_ARG, "tx_pin error");
  DMX_CHECK(DMX_RX_PIN_IS_VALID(rx_pin), ESP_ERR_INVALID_ARG, "rx_pin error");
  DMX_CHECK(DMX_RTS_PIN_IS_VALID(rts_pin), ESP_ERR_INVALID_ARG,
            "rts_pin error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;

  bool success = dmx_uart_set_pin(uart, tx_pin, rx_pin, rts_pin);
  // FIXME: change return type to bool

  return success ? ESP_OK : ESP_FAIL;
}

uint32_t dmx_set_baud_rate(dmx_port_t dmx_num, uint32_t baud_rate) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");

  // Clamp the baud rate to within DMX specification
  if (baud_rate < DMX_BAUD_RATE_MIN) {
    baud_rate = DMX_BAUD_RATE_MIN;
  } else if (baud_rate > DMX_BAUD_RATE_MAX) {
    baud_rate = DMX_BAUD_RATE_MAX;
  }

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_uart_set_baud_rate(dmx_context[dmx_num].uart, baud_rate);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return baud_rate;
}

uint32_t dmx_get_baud_rate(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const uint32_t baud_rate = dmx_uart_get_baud_rate(dmx_context[dmx_num].uart);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return baud_rate;
}

bool dmx_set_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_start_address > 0 && dmx_start_address < DMX_PACKET_SIZE_MAX,
            false, "dmx_start_address error");
  DMX_CHECK(dmx_get_start_address(dmx_num) != DMX_START_ADDRESS_NONE, false,
            "cannot set DMX start address");

  rdm_device_info_t *device_info = rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (device_info == NULL) {
    dmx_driver_personality_t *personality = (void *)dmx_driver[dmx_num]->pd;
    personality->dmx_start_address = dmx_start_address;
  } else {
    device_info->dmx_start_address = dmx_start_address;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  if (device_info != NULL) {
    // TODO: send message to RDM queue
  }

  dmx_nvs_set(dmx_num, RDM_PID_DMX_START_ADDRESS, RDM_DS_UNSIGNED_WORD,
              &dmx_start_address, sizeof(dmx_start_address));

  return true;
}

size_t dmx_write_offset(dmx_port_t dmx_num, size_t offset, const void *source,
                        size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(offset < DMX_PACKET_SIZE_MAX, 0, "offset error");
  DMX_CHECK(source, 0, "source is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp size to the maximum DMX packet size
  if (size + offset > DMX_PACKET_SIZE_MAX) {
    size = DMX_PACKET_SIZE_MAX - offset;
  } else if (size == 0) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if ((driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) && driver->rdm_type != 0) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    return 0;
  } else if (dmx_uart_get_rts(uart) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_uart_set_rts(uart, 0);
  }
  driver->tx_size = offset + size;  // Update driver transmit size

  // Copy data from the source to the driver buffer asynchronously
  memcpy(driver->data + offset, source, size);

  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

size_t dmx_write_rdm(dmx_port_t dmx_num, rdm_header_t *header, const void *pd) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL || pd != NULL, 0,
            "header is null and pd does not contain a UID");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t written = 0;

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;

  // Get pointers to driver data buffer locations and declare checksum
  uint16_t checksum = 0;
  uint8_t *header_ptr = driver->data;
  void *pd_ptr = header_ptr + 24;

  // RDM writes must be synchronous to prevent data corruption
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) {
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    return written;
  } else if (dmx_uart_get_rts(uart) == 1) {
    dmx_uart_set_rts(uart, 0);  // Stops writes from being overwritten
  }

  if (header != NULL && !(header->cc == RDM_CC_DISC_COMMAND_RESPONSE &&
                          header->pid == RDM_PID_DISC_UNIQUE_BRANCH)) {
    // Copy the header, pd, message_len, and pdl into the driver
    const size_t copy_size = header->pdl <= 231 ? header->pdl : 231;
    header->message_len = copy_size + 24;
    rdm_pd_emplace(header_ptr, "#cc01hbuubbbwbwb", header, sizeof(*header),
                   false);
    memcpy(pd_ptr, pd, copy_size);

    // Calculate and copy the checksum
    checksum = RDM_SC + RDM_SUB_SC;
    for (int i = 2; i < header->message_len; ++i) {
      checksum += header_ptr[i];
    }
    *(uint16_t *)(header_ptr + header->message_len) = bswap16(checksum);

    // Update written size
    written = header->message_len + 2;
  } else {
    // Encode the preamble bytes
    const size_t preamble_len = 7;
    for (int i = 0; i < preamble_len; ++i) {
      header_ptr[i] = RDM_PREAMBLE;
    }
    header_ptr[preamble_len] = RDM_DELIMITER;
    header_ptr += preamble_len + 1;

    // Encode the UID and calculate the checksum
    uint8_t uid[6];
    if (header == NULL) {
      memcpy(uid, pd, sizeof(rdm_uid_t));
    } else {
      rdm_uidcpy(uid, &header->src_uid);
    }
    for (int i = 0, j = 0; j < sizeof(rdm_uid_t); i += 2, ++j) {
      header_ptr[i] = uid[j] | 0xaa;
      header_ptr[i + 1] = uid[j] | 0x55;
      checksum += uid[j] + (0xaa | 0x55);
    }
    header_ptr += sizeof(rdm_uid_t) * 2;

    // Encode the checksum
    header_ptr[0] = (uint8_t)(checksum >> 8) | 0xaa;
    header_ptr[1] = (uint8_t)(checksum >> 8) | 0x55;
    header_ptr[2] = (uint8_t)(checksum) | 0xaa;
    header_ptr[3] = (uint8_t)(checksum) | 0x55;

    // Update written size
    written = preamble_len + 1 + 16;
  }

  // Update driver transmission size
  driver->tx_size = written;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return written;
}

size_t dmx_receive(dmx_port_t dmx_num, dmx_packet_t *packet,
                   TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_driver_is_enabled(dmx_num), 0, "driver is not enabled");

  dmx_driver_t *const restrict driver = dmx_driver[dmx_num];
  dmx_timer_handle_t timer = dmx_context[dmx_num].timer;
  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;

  // Set default return value and default values for output argument
  esp_err_t err = ESP_OK;
  uint32_t packet_size = 0;
  if (packet != NULL) {
    packet->err = ESP_ERR_TIMEOUT;
    packet->sc = -1;
    packet->size = 0;
    packet->is_rdm = false;
  }

  // Block until mutex is taken and driver is idle, or until a timeout
  TimeOut_t timeout;
  vTaskSetTimeOutState(&timeout);
  if (!xSemaphoreTakeRecursive(driver->mux, wait_ticks) ||
      (wait_ticks && xTaskCheckForTimeOut(&timeout, &wait_ticks))) {
    return packet_size;
  } else if (!dmx_wait_sent(dmx_num, wait_ticks) ||
             (wait_ticks && xTaskCheckForTimeOut(&timeout, &wait_ticks))) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Set the RTS pin to enable reading from the DMX bus
  if (dmx_uart_get_rts(uart) == 0) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    driver->head = -1;  // Wait for DMX break before reading data
    driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
    dmx_uart_set_rts(uart, 1);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Wait for new DMX packet to be received
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  int driver_flags = driver->flags;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (!(driver_flags & DMX_FLAGS_DRIVER_HAS_DATA) && wait_ticks > 0) {
    // Set task waiting and get additional DMX driver flags
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->task_waiting = xTaskGetCurrentTaskHandle();
    const int rdm_type = driver->rdm_type;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

    // Check for early timeout according to RDM specification
    const int RDM_EARLY_TIMEOUT =
        (DMX_FLAGS_RDM_IS_REQUEST | DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH);
    if ((driver_flags & DMX_FLAGS_DRIVER_SENT_LAST) &&
        (rdm_type & RDM_EARLY_TIMEOUT) == RDM_EARLY_TIMEOUT) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      const int64_t last_timestamp = driver->last_slot_ts;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

      // Guard against setting hardware alarm durations with negative values
      int64_t elapsed = esp_timer_get_time() - last_timestamp;
      if (elapsed >= RDM_PACKET_SPACING_CONTROLLER_NO_RESPONSE) {
        xSemaphoreGiveRecursive(driver->mux);
        return packet_size;
      }

      // Set an early timeout with the hardware timer
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      dmx_timer_set_counter(timer, elapsed);
      dmx_timer_set_alarm(timer, RDM_PACKET_SPACING_CONTROLLER_NO_RESPONSE);
      dmx_timer_start(timer);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }

    // Wait for a task notification
    const bool notified = xTaskNotifyWait(0, -1, (uint32_t *)&err, wait_ticks);
    dmx_timer_stop(timer);
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->task_waiting = NULL;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (packet_size == -1) {
      packet_size = 0;
    }

    if (!notified) {
      xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
      xSemaphoreGiveRecursive(driver->mux);
      return packet_size;
    }
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
    packet_size = driver->head;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else if (!(driver_flags & DMX_FLAGS_DRIVER_HAS_DATA)) {
    // Fail early if there is no data available and this function cannot block
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Parse DMX data packet
  if (packet != NULL) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    packet->sc = packet_size > 0 ? driver->data[0] : -1;
    driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    packet->err = err;
    packet->size = packet_size;
    packet->is_rdm = false;
  }

  // Return early if the no data was received
  if (packet_size == 0) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Return early if the packet is neither RDM nor an RDM request
  rdm_header_t header;
  if (!dmx_read_rdm(dmx_num, &header, NULL, 0) ||
      (header.cc != RDM_CC_DISC_COMMAND && header.cc != RDM_CC_GET_COMMAND &&
       header.cc != RDM_CC_SET_COMMAND)) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }
  if (packet != NULL) {
    packet->is_rdm = true;
  }

  // Ignore the packet if it does not target this device
  rdm_uid_t my_uid;
  rdm_uid_get(dmx_num, &my_uid);
  if (!rdm_uid_is_target(&my_uid, &header.dest_uid)) {
    // The packet should be ignored
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Prepare the response packet parameter data and find the correct callback
  rdm_response_type_t response_type;
  uint8_t pdl_out;
  uint8_t pd[231];
  int cb_num = 0;
  for (; cb_num < driver->num_rdm_cbs; ++cb_num) {
    if (driver->rdm_cbs[cb_num].desc.pid == header.pid) {
      break;
    }
  }
  const rdm_pid_description_t *desc;
  void *param;
  if (cb_num < driver->num_rdm_cbs) {
    desc = &driver->rdm_cbs[cb_num].desc;
    param = driver->rdm_cbs[cb_num].param;
  } else {
    desc = NULL;
    param = NULL;
  }

  // Determine how this device should respond to the request
  if (header.pdl > sizeof(pd) || header.port_id == 0 ||
      rdm_uid_is_broadcast(&header.src_uid)) {
    // The packet format is invalid
    response_type = RDM_RESPONSE_TYPE_NACK_REASON;
    pdl_out = rdm_pd_emplace_word(pd, RDM_NR_FORMAT_ERROR);
  } else if (cb_num == driver->num_rdm_cbs) {
    // The requested PID is unknown
    response_type = RDM_RESPONSE_TYPE_NACK_REASON;
    pdl_out = rdm_pd_emplace_word(pd, RDM_NR_UNKNOWN_PID);
  } else if ((header.cc == RDM_CC_DISC_COMMAND && desc->cc != RDM_CC_DISC) ||
             (header.cc == RDM_CC_GET_COMMAND && !(desc->cc & RDM_CC_GET)) ||
             (header.cc == RDM_CC_SET_COMMAND && !(desc->cc & RDM_CC_SET))) {
    // The PID does not support the request command class
    response_type = RDM_RESPONSE_TYPE_NACK_REASON;
    pdl_out = rdm_pd_emplace_word(pd, RDM_NR_UNSUPPORTED_COMMAND_CLASS);
  } else if ((header.sub_device > 512 &&
              header.sub_device != RDM_SUB_DEVICE_ALL) ||
             (header.sub_device == RDM_SUB_DEVICE_ALL &&
              header.cc == RDM_CC_GET_COMMAND)) {
    // The sub-device is out of range
    response_type = RDM_RESPONSE_TYPE_NACK_REASON;
    pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  } else {
    // Call the appropriate driver-side RDM callback to process the request
    pdl_out = 0;
    dmx_read_rdm(dmx_num, NULL, pd, sizeof(pd));
    const char *param_str = driver->rdm_cbs[cb_num].param_str;
    response_type = driver->rdm_cbs[cb_num].driver_cb(
        dmx_num, &header, pd, &pdl_out, param, desc, param_str);

    // Verify that the driver-side callback returned correctly
    if (pdl_out > sizeof(pd)) {
      ESP_LOGW(TAG, "PID 0x%04x pdl is too large", header.pid);
      // TODO: set the boot-loader flag
      response_type = RDM_RESPONSE_TYPE_NACK_REASON;
      pdl_out = rdm_pd_emplace_word(pd, RDM_NR_HARDWARE_FAULT);
    } else if ((response_type != RDM_RESPONSE_TYPE_NONE &&
                response_type != RDM_RESPONSE_TYPE_ACK &&
                response_type != RDM_RESPONSE_TYPE_ACK_TIMER &&
                response_type != RDM_RESPONSE_TYPE_NACK_REASON &&
                response_type != RDM_RESPONSE_TYPE_ACK_OVERFLOW) ||
               (response_type == RDM_RESPONSE_TYPE_NONE &&
                (header.pid != RDM_PID_DISC_UNIQUE_BRANCH ||
                 !rdm_uid_is_broadcast(&header.dest_uid))) ||
               ((response_type != RDM_RESPONSE_TYPE_ACK &&
                 response_type != RDM_RESPONSE_TYPE_NONE) &&
                header.cc == RDM_CC_DISC_COMMAND)) {
      ESP_LOGW(TAG, "PID 0x%04x returned invalid response type", header.pid);
      // TODO: set the boot-loader flag to indicate an error with this device
      response_type = RDM_RESPONSE_TYPE_NACK_REASON;
      pdl_out = rdm_pd_emplace_word(pd, RDM_NR_HARDWARE_FAULT);
    }
  }

  // Check if NVS needs to be updated
  bool must_update_nvs = false;
  if (header.cc == RDM_CC_SET_COMMAND &&
      (response_type == RDM_RESPONSE_TYPE_ACK ||
       response_type == RDM_RESPONSE_TYPE_NONE)) {
    const uint16_t nvs_pids[] = {
        RDM_PID_DEVICE_LABEL,    RDM_PID_LANGUAGE,
        RDM_PID_DMX_PERSONALITY, RDM_PID_DMX_START_ADDRESS,
        RDM_PID_DEVICE_HOURS,    RDM_PID_LAMP_HOURS,
        RDM_PID_LAMP_STRIKES,    RDM_PID_LAMP_STATE,
        RDM_PID_LAMP_ON_MODE,    RDM_PID_DEVICE_POWER_CYCLES,
        RDM_PID_DISPLAY_INVERT,  RDM_PID_DISPLAY_LEVEL,
        RDM_PID_PAN_INVERT,      RDM_PID_TILT_INVERT,
        RDM_PID_PAN_TILT_SWAP};
    for (int i = 0; i < sizeof(nvs_pids) / sizeof(uint16_t); ++i) {
      if (nvs_pids[i] == header.pid) {
        must_update_nvs = true;
        break;
      }
    }
  }

  // Don't respond to non-discovery broadcasts nor send NACK to DISC packets
  if ((rdm_uid_is_broadcast(&header.dest_uid) &&
       header.pid != RDM_PID_DISC_UNIQUE_BRANCH) ||
      (response_type == RDM_RESPONSE_TYPE_NACK_REASON &&
       header.cc == RDM_CC_DISC_COMMAND)) {
    response_type = RDM_RESPONSE_TYPE_NONE;
  }

  // Rewrite the header for the response packet
  header.message_len = 24 + pdl_out;  // Set for user callback
  header.dest_uid = header.src_uid;
  header.src_uid = my_uid;
  header.response_type = response_type;
  header.message_count = 0;  // TODO: update this if messages are queued
  header.cc += 1;            // Set to RCM_CC_x_COMMAND_RESPONSE
  header.pdl = pdl_out;
  // These fields should not change: tn, sub_device, and pid

  // Send the response packet
  if (response_type != RDM_RESPONSE_TYPE_NONE) {
    const size_t response_size = dmx_write_rdm(dmx_num, &header, pd);
    if (!dmx_send(dmx_num, response_size)) {
      ESP_LOGW(TAG, "PID 0x%04x did not send a response", header.pid);
      // TODO set the boot-loader flag
    } else if (response_size > 0) {
      dmx_wait_sent(dmx_num, 10);
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->head = -1;  // Wait for DMX break before reading data
      dmx_uart_set_rts(uart, 1);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  }

  // Call the user-side callback
  if (driver->rdm_cbs[cb_num].user_cb != NULL) {
    void *context = driver->rdm_cbs[cb_num].context;
    driver->rdm_cbs[cb_num].user_cb(dmx_num, &header, context);
  }

  // Update NVS values
  if (must_update_nvs) {
    if (!dmx_nvs_set(dmx_num, header.pid, desc->data_type, param,
                     desc->pdl_size)) {
      ESP_LOGW(TAG, "unable to save PID 0x%04x to NVS", header.pid);
      // TODO: set boot-loader flag
    }
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return packet_size;
}

size_t dmx_send(dmx_port_t dmx_num, size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_driver_is_enabled(dmx_num), 0, "driver is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];
  dmx_timer_handle_t timer = dmx_context[dmx_num].timer;
  dmx_uart_handle_t uart = dmx_context[dmx_num].uart;

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
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const rdm_cc_t cc = driver->data[20];
  if (*(uint16_t *)driver->data == (RDM_SC | (RDM_SUB_SC << 8)) &&
      (cc == RDM_CC_DISC_COMMAND_RESPONSE ||
       cc == RDM_CC_GET_COMMAND_RESPONSE ||
       cc == RDM_CC_SET_COMMAND_RESPONSE)) {
    elapsed = esp_timer_get_time() - driver->last_slot_ts;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (elapsed >= RDM_PACKET_SPACING_RESPONDER_NO_RESPONSE) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Determine if an alarm needs to be set to wait until driver is ready
  uint32_t timeout = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->flags & DMX_FLAGS_DRIVER_SENT_LAST) {
    if (driver->rdm_type & DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH) {
      timeout = RDM_PACKET_SPACING_DISCOVERY_NO_RESPONSE;
    } else if (driver->rdm_type & DMX_FLAGS_RDM_IS_BROADCAST) {
      timeout = RDM_PACKET_SPACING_BROADCAST;
    } else if (driver->rdm_type == DMX_FLAGS_RDM_IS_REQUEST) {
      timeout = RDM_PACKET_SPACING_REQUEST_NO_RESPONSE;
    }
  } else if (driver->rdm_type & DMX_FLAGS_RDM_IS_VALID) {
    timeout = RDM_PACKET_SPACING_RESPONSE;
  }
  elapsed = esp_timer_get_time() - driver->last_slot_ts;
  if (elapsed < timeout) {
    dmx_timer_set_counter(timer, elapsed);
    dmx_timer_set_alarm(timer, timeout);
    dmx_timer_start(timer);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Block if an alarm was set
  if (elapsed < timeout) {
    bool notified = xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    if (!notified) {
      dmx_timer_stop(timer);
      xTaskNotifyStateClear(driver->task_waiting);
    }
    driver->task_waiting = NULL;
    if (!notified) {
      xSemaphoreGiveRecursive(driver->mux);
      return 0;
    }
  }

  // Turn the DMX bus around and get the send size
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (dmx_uart_get_rts(uart) == 1) {
    xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    dmx_uart_set_rts(uart, 0);
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Update the transmit size if desired
  if (size > 0) {
    if (size > DMX_PACKET_SIZE_MAX) {
      size = DMX_PACKET_SIZE_MAX;
    }
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->tx_size = size;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    size = driver->tx_size;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Record the outgoing packet type
  const rdm_pid_t pid = bswap16(*(uint16_t *)&driver->data[21]);
  rdm_uid_t dest_uid;
  rdm_uidcpy(&dest_uid, &driver->data[3]);
  int rdm_type = 0;
  if (*(uint16_t *)driver->data == (RDM_SC | (RDM_SUB_SC << 8))) {
    rdm_type |= DMX_FLAGS_RDM_IS_VALID;
    if (cc == RDM_CC_DISC_COMMAND || cc == RDM_CC_GET_COMMAND ||
        cc == RDM_CC_SET_COMMAND) {
      rdm_type |= DMX_FLAGS_RDM_IS_REQUEST;
    }
    if (rdm_uid_is_broadcast(&dest_uid)) {
      rdm_type |= DMX_FLAGS_RDM_IS_BROADCAST;
    }
    if (pid == RDM_PID_DISC_UNIQUE_BRANCH) {
      rdm_type |= DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
    }
  } else if (driver->data[0] == RDM_PREAMBLE ||
             driver->data[0] == RDM_DELIMITER) {
    rdm_type |= DMX_FLAGS_RDM_IS_VALID | DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
  }
  driver->rdm_type = rdm_type;
  driver->flags |= DMX_FLAGS_DRIVER_SENT_LAST;
  if ((rdm_type & (DMX_FLAGS_RDM_IS_VALID | DMX_FLAGS_RDM_IS_REQUEST)) ==
      (DMX_FLAGS_RDM_IS_VALID | DMX_FLAGS_RDM_IS_REQUEST)) {
    ++driver->tn;
  }

  // Determine if a DMX break is required and send the packet
  if (rdm_type ==
      (DMX_FLAGS_RDM_IS_VALID | DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH)) {
    // RDM discovery responses do not send a DMX break - write immediately
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->flags |= DMX_FLAGS_DRIVER_IS_SENDING;

    size_t write_size = driver->tx_size;
    dmx_uart_write_txfifo(uart, driver->data, &write_size);
    driver->head = write_size;

    // Enable DMX write interrupts
    dmx_uart_enable_interrupt(uart, DMX_INTR_TX_ALL);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    // Send the packet by starting the DMX break
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->head = 0;
    driver->flags |=
        (DMX_FLAGS_DRIVER_IS_IN_BREAK | DMX_FLAGS_DRIVER_IS_SENDING);
    dmx_timer_set_counter(timer, 0);
    dmx_timer_set_alarm(timer, driver->break_len);
    dmx_timer_start(timer);

    dmx_uart_invert_tx(uart, 1);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return size;
}

esp_err_t dmx_sniffer_enable(dmx_port_t dmx_num, int intr_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, ESP_ERR_INVALID_ARG, "dmx_num error");
  DMX_CHECK(intr_pin > 0 && GPIO_IS_VALID_GPIO(intr_pin), ESP_ERR_INVALID_ARG,
            "intr_pin error");
  DMX_CHECK(!dmx_sniffer_is_enabled(dmx_num), ESP_ERR_INVALID_STATE,
            "sniffer is already enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Allocate the sniffer queue
  driver->metadata_queue = xQueueCreate(1, sizeof(dmx_metadata_t));
  DMX_CHECK(driver->metadata_queue != NULL, ESP_ERR_NO_MEM,
            "DMX sniffer queue malloc error");

  // Add the GPIO interrupt handler
  esp_err_t err = gpio_isr_handler_add(intr_pin, dmx_gpio_isr, driver);
  if (err) {
    ESP_LOGE(TAG, "DMX sniffer ISR handler error");
    vQueueDelete(driver->metadata_queue);
    driver->metadata_queue = NULL;
    return ESP_FAIL;
  }
  driver->sniffer_pin = intr_pin;

  // Set sniffer default values
  driver->last_neg_edge_ts = -1;  // Negative edge hasn't been seen yet
  driver->flags &= ~DMX_FLAGS_DRIVER_IS_IN_MAB;

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
  gpio_set_intr_type(driver->sniffer_pin, GPIO_INTR_DISABLE);
  esp_err_t err = gpio_isr_handler_remove(driver->sniffer_pin);
  DMX_CHECK(!err, err, "DMX sniffer ISR handler error");

  // Deallocate the sniffer queue
  vQueueDelete(driver->metadata_queue);
  driver->metadata_queue = NULL;

  return ESP_OK;
}