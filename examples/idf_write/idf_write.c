/*

  ESP-IDF DMX Write

  Writes data to the DMX bus. The value of every byte in the DMX packet (except
  the start code) is incremented by 1 every 1 second. After packet update
  thirty, the DMX driver will be uninstalled and the program will terminate.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 20 June 2022
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"

#define TX_PIN 17  // the pin we are using to TX with
#define RX_PIN 16  // the pin we are using to RX with
#define EN_PIN 21  // the pin we are using to enable TX on the DMX transceiver

static const char* TAG = "main";

// declare the user buffer to read in DMX data
static uint8_t data[DMX_MAX_PACKET_SIZE] = {};

void app_main() {
  // use DMX port 2
  const dmx_port_t dmx_num = DMX_NUM_2;

  // set communications pins
  ESP_ERROR_CHECK(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));

  // initialize the DMX driver without an event queue
  dmx_config_t driver_config = DMX_DEFAULT_CONFIG;
  ESP_ERROR_CHECK(dmx_driver_install(dmx_num, &driver_config, 0, NULL));
  dmx_set_mode(dmx_num, DMX_MODE_WRITE);

  // keep track of when we update the packet
  TickType_t next_tick = xTaskGetTickCount() + (1000 / portTICK_PERIOD_MS);
  bool update = true;
  int num_updates = 0;

  while (1) {
    const TickType_t now_tick = xTaskGetTickCount();

    dmx_send_packet(dmx_num, DMX_MAX_PACKET_SIZE);

    if (now_tick >= next_tick) {
      for (int i = 1; i < DMX_MAX_PACKET_SIZE; ++i) ++data[i];
      next_tick = now_tick + (1000 / portTICK_PERIOD_MS);
      update = true;
      ++num_updates;
    }

    if (update) {
      dmx_wait_write_sync(dmx_num, DMX_TX_PACKET_TOUT_TICK);
      dmx_write_packet(dmx_num, data, DMX_MAX_PACKET_SIZE);
      ESP_LOGI(TAG, "incrementing to %02X", data[1]);
      update = false;
    }

    dmx_wait_sent(dmx_num, DMX_TX_PACKET_TOUT_TICK);

    if (num_updates > 30) {
      ESP_LOGI(TAG, "uninstalling DMX driver");
      dmx_driver_delete(dmx_num);
      break;
    }

    // 40ms == ~25fps bytestream
    vTaskDelayUntil(&now_tick, 40 / portTICK_PERIOD_MS);
  }
}