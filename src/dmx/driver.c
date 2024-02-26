#include "dmx/include/driver.h"

#include <string.h>

#include "dmx/hal/include/gpio.h"
#include "dmx/hal/include/nvs.h"
#include "dmx/hal/include/timer.h"
#include "dmx/hal/include/uart.h"
#include "dmx/include/service.h"
#include "dmx/sniffer.h"
#include "endian.h"
#include "rdm/include/types.h"
#include "rdm/responder/include/utils.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_mac.h"  // TODO: Make this hardware agnostic
#endif

#ifdef CONFIG_RDM_DEVICE_UID_MAN_ID
/** @brief This is the RDM Manufacturer ID used with this library. It may be set
 * using the Kconfig file. The default value is 0x05e0.*/
#define RDM_UID_MANUFACTURER_ID (CONFIG_RDM_DEVICE_UID_MAN_ID)
#else
/** @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID (as
 * long as it is used responsibly) or may choose to register their own
 * manufacturer ID.*/
#define RDM_UID_MANUFACTURER_ID (0x05e0)
#endif

#ifdef CONFIG_RDM_DEVICE_UID_DEV_ID
/** @brief This is the RDM Device ID used with this library. It may be set
 * using the Kconfig file. The default value is a function of this device's MAC
 * address.*/
#define RDM_UID_DEVICE_ID (CONFIG_RDM_DEVICE_UID_DEV_ID)
#else
/** @brief This is the RDM Device ID used with this library. The default value
 * is a function of this device's MAC address.*/
#define RDM_UID_DEVICE_ID (0xffffffff)
#endif

#ifdef CONFIG_RDM_MANUFACTURER_LABEL
/** @brief This is the default manufacturer label for the RDM responder. Its
 * value may be updated using the Kconfig file.*/
#define RDM_MANUFACTURER_LABEL CONFIG_RDM_MANUFACTURER_LABEL
#else
/** @brief This is the default manufacturer label for the RDM responder.*/
#define RDM_MANUFACTURER_LABEL "esp_dmx"
#endif

const char *TAG = "dmx";  // The log tagline for the library.

dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {};  // The DMX drivers for each port.

static void rdm_default_identify_cb(dmx_port_t dmx_num, rdm_header_t *request,
                                    rdm_header_t *response, void *context) {
  if (request->cc == RDM_CC_SET_COMMAND &&
      request->sub_device == RDM_SUB_DEVICE_ROOT) {
    const uint8_t *identify = dmx_parameter_get_data(dmx_num, request->sub_device,
                                                RDM_PID_IDENTIFY_DEVICE);
#ifdef ARDUINO
    printf("RDM identify device is %s\n", *identify ? "on" : "off");
#else
    ESP_LOGI(TAG, "RDM identify device is %s", *identify ? "on" : "off");
#endif
  }
}

bool dmx_driver_install(dmx_port_t dmx_num, const dmx_config_t *config,
                        const dmx_personality_t *personalities,
                        int personality_count) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(config != NULL, false, "config is null");
  DMX_CHECK(personality_count >= 0 && personality_count <= 255, false,
            "personality_count error");
  DMX_CHECK(!dmx_driver_is_installed(dmx_num), false,
            "driver is already installed");
  bool uses_dmx = false;
  for (int i = 0; i < personality_count; ++i) {
    DMX_CHECK((personalities[i].footprint > 0 &&
               personalities[i].footprint < DMX_PACKET_SIZE_MAX),
              false, "footprint error");
    if (personalities[i].footprint > 0) {
      uses_dmx = true;
    }
  }

  int interrupt_flags = config->interrupt_flags;
#ifdef DMX_ISR_IN_IRAM
  // Driver ISR is in IRAM so interrupt flags must include IRAM flag
  if (!(interrupt_flags & ESP_INTR_FLAG_IRAM)) {
    interrupt_flags |= ESP_INTR_FLAG_IRAM;
    ESP_LOGI(TAG, "ESP_INTR_FLAG_IRAM flag not set, flag updated");
  }
#endif
  
  // Ensure the parameter count is valid
  const int required_parameter_count = uses_dmx ? 7 : 6;
  int root_param_count = config->root_device_parameter_count;
  if (root_param_count > 0 && root_param_count < required_parameter_count) {
    DMX_WARN(
        "root_device_parameter_count must be 0 or at least %i, "
        "root_device_parameter_count updated to %i",
        required_parameter_count, required_parameter_count);
    root_param_count = required_parameter_count;
  }

  // Initialize NVS
  dmx_nvs_init(dmx_num);

  // Allocate the DMX driver
  const size_t driver_size =
      sizeof(dmx_driver_t) + (sizeof(dmx_parameter_t) * root_param_count);
  dmx_driver_t *driver = heap_caps_malloc(driver_size, MALLOC_CAP_8BIT);
  DMX_CHECK(driver != NULL, false, "DMX driver malloc error");
  dmx_driver[dmx_num] = driver;
  driver->mux = NULL;
#ifdef DMX_USE_SPINLOCK
  driver->spinlock = (dmx_spinlock_t)DMX_SPINLOCK_INIT;
#endif

  // Allocate mutex
  driver->mux = xSemaphoreCreateRecursiveMutex();
  if (driver->mux == NULL) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(driver->mux != NULL, false, "DMX driver mutex malloc error");
  }

  // Driver configuration
  driver->dmx_num = dmx_num;
  driver->uid.man_id = RDM_UID_MANUFACTURER_ID;
#if RDM_UID_DEVICE_ID == 0xffffffff
  // Set the device ID based on the device's MAC address
  uint8_t mac[8];
  esp_efuse_mac_get_default(mac);
  driver->uid.dev_id = bswap32(*(uint32_t *)(mac + 2));
#else
  // Set the device ID based on what the user set in the kconfig
  driver->uid.dev_id = RDM_UID_DEVICE_UID;
#endif
  *(uint8_t *)(&driver->uid.dev_id) += dmx_num;  // Increment last octect
  driver->break_len = RDM_BREAK_LEN_US;
  driver->mab_len = RDM_MAB_LEN_US;

  // Set the default values for the root device
  driver->device.root.num = RDM_SUB_DEVICE_ROOT;
  driver->device.root.next = NULL;
  for (int i = 0; i < config->root_device_parameter_count; ++i) {
    driver->device.root.parameters[i].pid = 0;
  }

  // Set the default values for the DMX device
  driver->device.parameter_count.root = config->root_device_parameter_count;
  driver->device.parameter_count.sub_devices =
      config->sub_device_parameter_count;
  driver->device.parameter_count.staged = 0;
  driver->is_controller = false;  // Assume false until dmx_send_num()
  driver->is_enabled = true;

  // Synchronization state
  driver->task_waiting = NULL;

  // Data buffer
  driver->dmx.head = DMX_HEAD_WAITING_FOR_BREAK;
  driver->dmx.size = DMX_PACKET_SIZE_MAX;
  memset(driver->dmx.data, 0, sizeof(driver->dmx.data));
  driver->dmx.status = DMX_STATUS_IDLE;
  driver->dmx.progress = DMX_PROGRESS_STALE;
  driver->dmx.last_controller_pid = 0;
  driver->dmx.controller_eop_timestamp = 0;
  driver->dmx.last_responder_pid = 0;
  driver->dmx.responder_sent_last = false;
  driver->dmx.last_request_pid = 0;
  driver->dmx.last_request_pid_repeats = 0;

  // RDM responder configuration
  driver->rdm.tn = 0;

  // DMX sniffer configuration
  driver->sniffer.is_enabled = false;
  driver->sniffer.buffer_index = 0;
  // The driver->metadata field is left uninitialized
  driver->sniffer.last_pos_edge_ts = -1;
  driver->sniffer.last_neg_edge_ts = -1;

  // Add the personality numbers to the DMX personalities
  rdm_dmx_personality_description_t *personality_description =
      (void *)personalities;
  for (int i = 0; i < personality_count; ++i) {
    // Personalities are indexed beginning at 1
    personality_description[i].personality_num = i + 1;
  }

  // Register the default RDM parameters
  rdm_register_disc_unique_branch(dmx_num, NULL, NULL);
  rdm_register_disc_mute(dmx_num, NULL, NULL);
  rdm_register_disc_un_mute(dmx_num, NULL, NULL);
  rdm_register_device_info(dmx_num, config->model_id, config->product_category,
                           config->software_version_id, NULL, NULL);
  rdm_register_software_version_label(dmx_num, config->software_version_label,
                                      NULL, NULL);
  rdm_register_identify_device(dmx_num, rdm_default_identify_cb, NULL);

  // The registration of DMX parameters is optional
  if (uses_dmx > 0) {
    rdm_register_dmx_start_address(dmx_num, NULL, NULL);
  }

  // Register additional RDM parameters
  if (config->queue_size_max > 0) {
    rdm_register_queued_message(dmx_num, config->queue_size_max, NULL, NULL);
  }
  rdm_register_manufacturer_label(dmx_num, RDM_MANUFACTURER_LABEL, NULL, NULL);
  if (uses_dmx > 0) {
    rdm_register_dmx_personality(dmx_num, personality_count, NULL, NULL);
    rdm_register_dmx_personality_description(dmx_num, personality_description,
                                             personality_count, NULL, NULL);
  }
  const char *default_device_label = "";
  rdm_register_device_label(dmx_num, default_device_label, NULL, NULL);
  rdm_register_supported_parameters(dmx_num, NULL, NULL);
  rdm_register_parameter_description(dmx_num, NULL, NULL);

  // Initialize the UART peripheral
  if (!dmx_uart_init(dmx_num, driver, interrupt_flags)) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(false, false, "UART init error");
  }

  // Initialize the timer peripheral
  if (!dmx_timer_init(dmx_num, driver, interrupt_flags)) {
    dmx_driver_delete(dmx_num);
    DMX_CHECK(false, false, "timer init error");
  }

  // Enable reading on the DMX port
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
  dmx_uart_enable_interrupt(dmx_num, DMX_INTR_RX_ALL);
  dmx_uart_set_rts(dmx_num, 1);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Give the mutex and return
  xSemaphoreGiveRecursive(driver->mux);
  return true;
}

bool dmx_driver_delete(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Take the mutex
  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return false;
  }
  SemaphoreHandle_t mux = driver->mux;

  // Uninstall sniffer ISR
  if (dmx_sniffer_is_enabled(dmx_num)) {
    dmx_sniffer_disable(dmx_num);
  }

  // Free hardware timer ISR
  dmx_timer_deinit(dmx_num);

  // Disable UART module
  dmx_uart_deinit(dmx_num);

  // Free parameters
  dmx_device_t *device = &driver->device.root;
  do {
    int param_count = device->num == RDM_SUB_DEVICE_ROOT
                          ? driver->device.parameter_count.root
                          : driver->device.parameter_count.sub_devices;
    for (int i = 0; i < param_count; ++i) {
      if (device->parameters[i].pid == 0) {
        break;  // No more parameters remaining
      } else if (device->parameters[i].type != DMX_PARAMETER_TYPE_DYNAMIC) {
        continue;  // Nothing to free
      }
      free(device->parameters[i].data);
    }
    device = device->next;   
  } while (device != NULL);

  // Free devices
  device = driver->device.root.next;  // Can't free root device
  while (device != NULL) {
    dmx_device_t *next_device = device->next;
    free(device);
    device = next_device;
  } 

  // Free driver
  heap_caps_free(driver);
  dmx_driver[dmx_num] = NULL;

  // Free driver mutex
  xSemaphoreGiveRecursive(mux);
  vSemaphoreDelete(mux);

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
  if (driver->dmx.status != DMX_STATUS_SENDING) {
    dmx_uart_disable_interrupt(dmx_num, DMX_INTR_RX_ALL);
    dmx_uart_clear_interrupt(dmx_num, DMX_INTR_RX_ALL);
    driver->is_enabled = false;
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
  driver->dmx.head = -1;  // Wait for DMX break before reading data
  driver->is_enabled = true;
  dmx_uart_rxfifo_reset(dmx_num);
  dmx_uart_txfifo_reset(dmx_num);
  dmx_uart_enable_interrupt(dmx_num, DMX_INTR_RX_ALL);
  dmx_uart_clear_interrupt(dmx_num, DMX_INTR_RX_ALL);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return true;
}

bool dmx_set_pin(dmx_port_t dmx_num, int tx_pin, int rx_pin, int rts_pin) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_tx_pin_is_valid(tx_pin), false, "tx_pin error");
  DMX_CHECK(dmx_rx_pin_is_valid(rx_pin), false, "rx_pin error");
  DMX_CHECK(dmx_rts_pin_is_valid(rts_pin), false, "rts_pin error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  return dmx_uart_set_pin(dmx_num, tx_pin, rx_pin,
                          rts_pin);
}

bool dmx_driver_is_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && dmx_driver[dmx_num] != NULL;
}

bool dmx_driver_is_enabled(dmx_port_t dmx_num) {
  bool is_enabled;
  if (dmx_driver_is_installed(dmx_num)) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    is_enabled = dmx_driver[dmx_num]->is_enabled;
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
  baud_rate = dmx_uart_get_baud_rate(dmx_num);
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
  dmx_uart_set_baud_rate(dmx_num, baud_rate);
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

const rdm_uid_t *rdm_uid_get(dmx_port_t dmx_num) {
  return dmx_driver_is_installed(dmx_num) ? &dmx_driver[dmx_num]->uid : NULL;
}
