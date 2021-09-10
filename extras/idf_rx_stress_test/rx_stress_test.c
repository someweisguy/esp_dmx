#include <stdio.h>

#include "esp_dmx.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "esp_wifi.h"
#include <string.h>
#include "nvs_flash.h"

#define TX_PIN  17
#define RX_PIN  16
#define EN_PIN  21

#define LED_PIN 13

#define EXAMPLE_SSID "MySSID"
#define EXAMPLE_PASS "MyPassword"

static const char *TAG = "main";

uint8_t data[DMX_MAX_PACKET_SIZE] = {};

static void wifi_handler(void *handler_args, esp_event_base_t base, 
    int event_id, void *event_data) {
  if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG, "wifi started");
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGI(TAG, "wifi disconnected");
    esp_wifi_connect();
  } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ESP_LOGI(TAG, "wifi connected");
  }
}

void dmx_rx_task(void *arg) {
  const dmx_port_t dmx_num = *(dmx_port_t *)arg;

  // install the DMX driver
  QueueHandle_t queue;
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);
  const dmx_config_t dmx_config = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmx_num, &dmx_config);
  dmx_driver_install(dmx_num, DMX_MAX_PACKET_SIZE, 1, &queue, 1);

  dmx_set_mode(dmx_num, DMX_MODE_RX); // not required - in rx mode by default
  
  // install the default gpio isr
  gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
  dmx_rx_timing_enable(dmx_num, RX_PIN);

  // light up the built-in LED when DMX connected
  gpio_reset_pin(LED_PIN);
  gpio_set_pull_mode(LED_PIN, GPIO_PULLUP_DISABLE);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 0);

  uint32_t packet_counter = 0;
  bool timeout = true;
  while (!xTaskNotifyWait(0, 0, 0, 0)) {
    dmx_event_t packet;
    if (xQueueReceive(queue, &packet, DMX_RX_PACKET_TOUT_TICK)) {
      if (packet.status == DMX_OK) {
        if (timeout) {
          ESP_LOGI(TAG, "dmx connected");
          timeout = false;
          packet_counter = 0;
          gpio_set_level(LED_PIN, 1); // LED on
        }
        ++packet_counter;
        dmx_read_packet(dmx_num, data, packet.size);
        ESP_LOGI(TAG,
            "Frame: %i, Start Code: %02X, Size: %i, Slot 1: %03i, Break: %ius, "
            "MAB: %ius Packet: %.2fms",
            packet_counter, packet.start_code, packet.size, data[1],
            packet.timing.brk, packet.timing.mab, packet.duration / 1000.0);
      } else {
        char *err_name;
        switch (packet.status) {
          case DMX_ERR_BUFFER_SIZE:
            err_name = "DMX_ERR_BUFFER_SIZE";
            break;
          case DMX_ERR_IMPROPER_SLOT:
            err_name = "DMX_ERR_IMPROPER_SLOT";
            break;
          case DMX_ERR_PACKET_SIZE:
            err_name = "DMX_ERR_PACKET_SIZE";
            break;
          case DMX_ERR_DATA_OVERFLOW:
            err_name = "DMX_ERR_DATA_OVERFLOW";
            break;
          default:
            err_name = "UNKNOWN_ERR";
        }
        ESP_LOGE(TAG, "dmx error %s", err_name);
      }
    } else if (timeout == false) {
      ESP_LOGW(TAG, "lost dmx signal");
      timeout = true;
      gpio_set_level(LED_PIN, 0); // LED off
    }
    // do other stuff here...
  }

  // if task is notified, the loop is exited and the driver deleted
  dmx_driver_delete(dmx_num);

  vTaskDelete(NULL);
}

void app_main(void) { 
  nvs_flash_init();

  // initialize esp network STA interface
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  // init wifi driver with default configuration
  wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&wifi_init_config);

  // register wifi event handlers
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_handler, NULL);

  // configure the wifi connection and start the driver
  wifi_config_t wifi_config = {
    .sta = {
      .ssid = EXAMPLE_SSID,
      .password = EXAMPLE_PASS,
    }
  };
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
  esp_wifi_start();

  // create the DMX task
  ESP_LOGI(TAG, "Creating DMX task...");
  TaskHandle_t dmx_task_handle;
  static dmx_port_t dmx_num = 2;
  xTaskCreate(dmx_rx_task, "dmx_task", 3072, &dmx_num, 10, &dmx_task_handle);


  // uncomment the code below to stop the task after 10 seconds

  //vTaskDelay(10000 / portTICK_PERIOD_MS);
  //xTaskNotify(dmx_task_handle, 0, eNoAction);
  //dmx_task_handle = NULL;
  //ESP_LOGI(TAG, "Stopped DMX!");

}
