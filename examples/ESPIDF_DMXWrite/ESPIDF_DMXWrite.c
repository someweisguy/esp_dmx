/*

  ESP-IDF DMX Write

  Writes data to the DMX bus. The value of every byte in the DMX packet (except
  the start code) is incremented by 1 every 1 second.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 20 June 2022
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/task.h"

#define TX_PIN 17  // the pin we are using to TX with
#define RX_PIN 16  // the pin we are using to RX with
#define EN_PIN 21  // the pin we are using to enable TX on the DMX transceiver

static const char *TAG = "main";

static uint8_t data[DMX_PACKET_SIZE] = {};  // Buffer to store DMX data

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  const int personality_count = 0;
  dmx_driver_install(dmx_num, &config, NULL, personality_count);
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  TickType_t last_update = xTaskGetTickCount();
  while (1) {
    TickType_t now = xTaskGetTickCount();

    // Send data and block until it's sent
    dmx_send_num(dmx_num, DMX_PACKET_SIZE);
    dmx_wait_sent(dmx_num, DMX_TIMEOUT_TICK);

    if (now - last_update >= pdMS_TO_TICKS(1000)) {
      // Only update data every 1000ms
      for (int i = 1; i < DMX_PACKET_SIZE; i++) {
        data[i]++;
      }
      dmx_write(dmx_num, data, DMX_PACKET_SIZE);
      ESP_LOGI(TAG, "Incremented packet slots to 0x%02x", data[1]);
      last_update = now;
    }

    // Only send a packet every 30ms
    vTaskDelayUntil(&now, pdMS_TO_TICKS(30));
  }
}