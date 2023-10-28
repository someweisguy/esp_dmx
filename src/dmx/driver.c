#include "dmx/driver.h"

#include "dmx/hal/nvs.h"
#include "dmx/hal/timer.h"
#include "dmx/hal/uart.h"
#include "dmx/sniffer.h"
#include "dmx/struct.h"
#include "endian.h"
#include "rdm/types.h"
#include "rdm/uid.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_mac.h"  // TODO: Make this hardware agnostic
#endif

const char *TAG = "dmx";  // The log tagline for the library.

dmx_port_t rdm_binding_port;    // The DMX port which owns the RDM binding UID.
rdm_uid_t rdm_device_uid = {};  // The UID of the RDM device.

dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {};  // The DMX drivers for each port.

static void rdm_default_identify_cb(dmx_port_t dmx_num,
                                    const rdm_header_t *header, void *context) {
  if (header->cc == RDM_CC_SET_COMMAND) {
    const uint8_t *identify =
        rdm_pd_get(dmx_num, RDM_PID_IDENTIFY_DEVICE, header->sub_device);
#ifdef ARDUINO
    printf("RDM identify device is %s\n", *identify ? "on" : "off");
#else
    ESP_LOGI(TAG, "RDM identify device is %s", *identify ? "on" : "off");
#endif
  }
}

bool dmx_driver_install(dmx_port_t dmx_num, const dmx_config_t *config,
                        int intr_flags) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(!dmx_driver_is_installed(dmx_num), false,
            "driver is already installed");
  DMX_CHECK(config->dmx_start_address < DMX_PACKET_SIZE_MAX ||
                config->dmx_start_address == DMX_START_ADDRESS_NONE,
            false, "dmx_start_address error");
  DMX_CHECK((config->personality_count == 0 &&
             config->dmx_start_address == DMX_START_ADDRESS_NONE) ||
                (config->personality_count > 0 &&
                 config->personality_count < DMX_PERSONALITY_COUNT_MAX),
            false, "personality_count error");
  DMX_CHECK(config->current_personality <= config->personality_count, false,
            "current_personality error");
  for (int i = 0; i < config->personality_count; ++i) {
    DMX_CHECK(config->personalities[i].footprint < DMX_PACKET_SIZE_MAX, false,
              "footprint error");
  }

#ifdef DMX_ISR_IN_IRAM
  // Driver ISR is in IRAM so interrupt flags must include IRAM flag
  if (!(intr_flags & ESP_INTR_FLAG_IRAM)) {
    intr_flags |= ESP_INTR_FLAG_IRAM;
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
  }
#endif

  // Initialize RDM UID
  if (rdm_uid_is_null(&rdm_device_uid)) {
    rdm_device_uid.man_id = RDM_UID_MANUFACTURER_ID;
    uint32_t dev_id;
#if RDM_UID_DEVICE_ID == 0xffffffff
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    dev_id = bswap32(*(uint32_t *)(mac + 2));
#else
    dev_id = RDM_UID_DEVICE_UID;
#endif
    rdm_device_uid.dev_id = dev_id;
    rdm_binding_port = dmx_num;
  }

  // Initialize NVS
  dmx_nvs_init(dmx_num);

  dmx_driver_t *driver;

  // Allocate the DMX driver
  driver = heap_caps_malloc(sizeof(dmx_driver_t), MALLOC_CAP_8BIT);
  DMX_CHECK(driver != NULL, false, "DMX driver malloc error");
  dmx_driver[dmx_num] = driver;
  driver->mux = NULL;
  driver->data = NULL;
  driver->pd = NULL;
#ifdef DMX_USE_SPINLOCK
  driver->spinlock = (dmx_spinlock_t)DMX_SPINLOCK_INIT;
#endif

  // Allocate mutex
  SemaphoreHandle_t mux = xSemaphoreCreateRecursiveMutex();
  if (mux == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(mux != NULL, false, "DMX driver mutex malloc error");
  }

  // Allocate DMX buffer
  uint8_t *data = heap_caps_malloc(DMX_PACKET_SIZE, MALLOC_CAP_8BIT);
  if (data == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(data != NULL, false, "DMX driver buffer malloc error");
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
    DMX_CHECK(pd != NULL, false, "DMX driver pd buffer malloc error");
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
  driver->rdm_queue_size = 0;
  driver->rdm_queue_last_sent = 0;  // A queued message has not yet been sent
  driver->num_parameters = 0;
  // The driver->params field is left uninitialized

  // DMX sniffer configuration
  // The driver->metadata field is left uninitialized
  driver->metadata_queue = NULL;
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
    rdm_register_supported_parameters(dmx_num, NULL, NULL);
    rdm_register_disc_unique_branch(dmx_num, NULL, NULL);
    rdm_register_disc_un_mute(dmx_num, NULL, NULL);
    rdm_register_disc_mute(dmx_num, NULL, NULL);
    rdm_register_device_info(dmx_num, &device_info, NULL, NULL);
    rdm_register_device_label(dmx_num, config->device_label, NULL, NULL);
    rdm_register_software_version_label(dmx_num, config->software_version_label,
                                        NULL, NULL);
    rdm_register_identify_device(dmx_num, rdm_default_identify_cb, NULL);
    rdm_register_dmx_personality(dmx_num, NULL, NULL);
    rdm_register_dmx_personality_description(dmx_num, NULL, NULL);
    rdm_register_parameter_description(dmx_num, NULL, NULL);

    if (device_info.dmx_start_address != DMX_START_ADDRESS_NONE) {
      rdm_register_dmx_start_address(dmx_num, NULL, NULL);
    }
  } else {
    dmx_driver_personality_t *dmx = driver->pd;

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

  // Initialize the UART peripheral
  driver->uart = dmx_uart_init(dmx_num, driver, intr_flags);
  if (driver->uart == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(driver->uart != NULL, false, "UART init error");
  }

  // Initialize the timer peripheral
  driver->timer = dmx_timer_init(dmx_num, driver, intr_flags);
  if (driver->timer == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(driver->timer != NULL, false, "timer init error");
  }

  // Enable reading on the DMX port
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
  dmx_uart_enable_interrupt(driver->uart, DMX_INTR_RX_ALL);
  dmx_uart_set_rts(driver->uart, 1);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Give the mutex and return
  xSemaphoreGiveRecursive(driver->mux);
  return true;
}

bool dmx_driver_delete(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Free driver mutex
  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return false;
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
  dmx_timer_deinit(driver->timer);

  // Disable UART module
  dmx_uart_deinit(driver->uart);

  // Free driver
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;

  return true;
}

bool dmx_driver_disable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(dmx_driver_is_enabled(dmx_num), false,
            "driver is already disabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Disable receive interrupts
  bool ret = false;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (!(driver->flags & DMX_FLAGS_DRIVER_IS_SENDING)) {
    dmx_uart_disable_interrupt(driver->uart, DMX_INTR_RX_ALL);
    dmx_uart_clear_interrupt(driver->uart, DMX_INTR_RX_ALL);
    driver->flags &= ~DMX_FLAGS_DRIVER_IS_ENABLED;
    ret = true;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return ret;
}

bool dmx_driver_enable(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(!dmx_driver_is_enabled(dmx_num), false,
            "driver is already enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Initialize driver flags and reenable interrupts
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  driver->head = -1;  // Wait for DMX break before reading data
  driver->flags |= (DMX_FLAGS_DRIVER_IS_ENABLED | DMX_FLAGS_DRIVER_IS_IDLE);
  driver->flags &= ~(DMX_FLAGS_DRIVER_IS_IN_BREAK | DMX_FLAGS_DRIVER_HAS_DATA);
  dmx_uart_rxfifo_reset(driver->uart);
  dmx_uart_txfifo_reset(driver->uart);
  dmx_uart_enable_interrupt(driver->uart, DMX_INTR_RX_ALL);
  dmx_uart_clear_interrupt(driver->uart, DMX_INTR_RX_ALL);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return true;
}

bool dmx_set_pin(dmx_port_t dmx_num, int tx_pin, int rx_pin, int rts_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_tx_pin_is_valid(tx_pin), false, "tx_pin error");
  DMX_CHECK(dmx_rx_pin_is_valid(rx_pin), false, "rx_pin error");
  DMX_CHECK(dmx_rts_pin_is_valid(rts_pin), false, "rts_pin error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  return dmx_uart_set_pin(dmx_driver[dmx_num]->uart, tx_pin, rx_pin, rts_pin);
}

bool DMX_ISR_ATTR dmx_driver_is_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && dmx_driver[dmx_num] != NULL;
}

bool dmx_driver_is_enabled(dmx_port_t dmx_num) {
  bool is_enabled;
  if (dmx_driver_is_installed(dmx_num)) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    is_enabled = dmx_driver[dmx_num]->flags & DMX_FLAGS_DRIVER_IS_ENABLED;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    is_enabled = false;
  }

  return is_enabled;
}

uint32_t dmx_get_baud_rate(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");

  uint32_t baud_rate;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  baud_rate = dmx_uart_get_baud_rate(dmx_driver[dmx_num]->uart);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return baud_rate;
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
  dmx_uart_set_baud_rate(dmx_driver[dmx_num]->uart, baud_rate);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return baud_rate;
}

uint32_t dmx_get_break_len(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint32_t break_len;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  break_len = dmx_driver[dmx_num]->break_len;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return break_len;
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

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->break_len = break_len;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return break_len;
}

uint32_t dmx_get_mab_len(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint32_t mab_len;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  mab_len = dmx_driver[dmx_num]->mab_len;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return mab_len;
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

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->mab_len = mab_len;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return mab_len;
}