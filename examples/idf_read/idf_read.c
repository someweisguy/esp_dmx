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

#define TX_PIN 17  // the pin we are using to TX with
#define RX_PIN 16  // the pin we are using to RX with
#define EN_PIN 21  // the pin we are using to enable TX on the DMX transceiver

static const char* TAG = "main";

// declare the user buffer to read in DMX data
static uint8_t data[DMX_MAX_PACKET_SIZE] = {0xaa};

void app_main() {
  // use DMX port 2
  const dmx_port_t dmx_num = DMX_NUM_2;

  // set communications pins
  ESP_ERROR_CHECK(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));

  // initialize the DMX driver with an event queue to read data
  QueueHandle_t queue;

  // initialize the DMX driver without an event queue
  dmx_config_t driver_config = DMX_DEFAULT_CONFIG;
  ESP_ERROR_CHECK(dmx_driver_install(dmx_num, &driver_config, 10, &queue));

  // allows us to know when the packet times out after it connects
  bool timeout = true;

  TickType_t next_tick = xTaskGetTickCount();

  while (1) {
    const TickType_t now_tick = xTaskGetTickCount();
    dmx_event_t packet;
    // wait until a packet is received or times out
    if (xQueueReceive(queue, &packet, DMX_RX_PACKET_TOUT_TICK)) {
      if (packet.status == DMX_OK) {
        // print a message upon initial DMX connection
        if (timeout) {
          ESP_LOGI(TAG, "dmx connected");
          timeout = false;  // establish connection!
          next_tick = now_tick + (1000 / portTICK_PERIOD_MS);
        }

        // read the packet into the data buffer
        dmx_read_packet(dmx_num, data, packet.size);

        // print a log message every 1 second
        if (xTaskGetTickCount() >= next_tick) {
          ESP_LOG_BUFFER_HEX(TAG, data, 16);

          next_tick = now_tick + (1000 / portTICK_PERIOD_MS);
        }

      } else if (packet.status != DMX_OK) {
        // something went wrong receiving data
        ESP_LOGE(TAG, "dmx error %i - size: %i", packet.status, packet.size);
        break;
      }

    } else if (timeout == false) {
      // lost connection
      ESP_LOGW(TAG, "lost dmx signal");
      break;
    }
  }
}