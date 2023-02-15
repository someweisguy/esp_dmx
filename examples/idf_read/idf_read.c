/*

  ESP-IDF DMX Read

  Read synchronously from the DMX bus. If the packet is a standard DMX packet
  and there are no errors in the packet, logs a hex dump every 1 second
  containing the first 16 bytes of data from the data packet. When the data
  times out the driver is uninstalled and the program terminates.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 22 June 2022
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/task.h"

#define TX_PIN 17  // The DMX transmit pin.
#define RX_PIN 16  // The DMX receive pin.
#define EN_PIN 21  // The DMX transmit enable pin.

static const char *TAG = "main";  // The log tagline.

static uint8_t data[DMX_PACKET_SIZE] = {};  // Buffer to store DMX data

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_2;

  // Set communication pins and install the driver
  ESP_ERROR_CHECK(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));
  ESP_ERROR_CHECK(dmx_driver_install(dmx_num, DMX_DEFAULT_INTR_FLAGS));

  dmx_packet_t packet;
  bool is_connected = false;

  TickType_t last_update = xTaskGetTickCount();
  while (true) {
    // Block until a packet is received
    if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK)) {
      const TickType_t now = xTaskGetTickCount();

      if (!is_connected) {
        // Log when we first connect
        ESP_LOGI(TAG, "DMX is connected.");
        is_connected = true;
      }

      if (now - last_update >= pdMS_TO_TICKS(1000)) {
        // Only read data every 1000ms
        dmx_read(dmx_num, data, DMX_PACKET_SIZE);
        ESP_LOGI(TAG, "Start code: %02x, Size: %i", packet.sc, packet.size);
        ESP_LOG_BUFFER_HEX(TAG, data, 16);  // Log first 16 bytes
        last_update = now;
      }

    } else if (is_connected) {
      // DMX timed out after having been previously connected
      ESP_LOGI(TAG, "DMX was disconnected.");
      break;
    }
  }

  ESP_LOGI(TAG, "Uninstalling the DMX driver.");
  dmx_driver_delete(dmx_num);
}
