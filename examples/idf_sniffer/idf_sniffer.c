/*

  ESP-IDF Sniffer

  This example demonstrates how to use the DMX sniffer to read packet
  metadata from incoming DMX data packets. Data is read synchronously from the
  DMX bus. If there are no errors in the packet a log is written every 1 second
  that contains information about the received DMX data packet. When the data
  times out, the driver is uninstalled and the program terminates.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 28 January 2021
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
#define SN_PIN 4   // The DMX sniffer pin.

static const char *TAG = "main";  // The log tagline.

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_2;

  // Set communication pins and install the driver
  ESP_ERROR_CHECK(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));
  ESP_ERROR_CHECK(dmx_driver_install(dmx_num, DMX_DEFAULT_INTR_FLAGS));

  // Install the default GPIO ISR and enable the sniffer
  ESP_ERROR_CHECK(gpio_install_isr_service(DMX_DEFAULT_SNIFFER_INTR_FLAGS));
  ESP_ERROR_CHECK(dmx_sniffer_enable(dmx_num, SN_PIN));

  dmx_metadata_t metadata;
  bool is_connected = false;

  TickType_t last_update = xTaskGetTickCount();
  while (true) {
    // Block until a packet is received
    if (dmx_sniffer_get_data(dmx_num, &metadata, DMX_TIMEOUT_TICK)) {
      const TickType_t now = xTaskGetTickCount();

      if (!is_connected) {
        // Log when we first connect
        ESP_LOGI(TAG, "DMX is connected.");
        is_connected = true;
      }

      if (now - last_update >= pdMS_TO_TICKS(1000)) {
        // Only log data every 1000ms
        ESP_LOGI(TAG, "Break: %li, MAB: %li", metadata.break_len,
                 metadata.mab_len);
        last_update = now;
      }

    } else if (is_connected) {
      // DMX timed out after having been previously connected
      ESP_LOGI(TAG, "DMX was disconnected.");
      dmx_sniffer_disable(dmx_num);
      break;
    }
  }

  ESP_LOGI(TAG, "Uninstalling the DMX driver.");
  dmx_driver_delete(dmx_num);
}
