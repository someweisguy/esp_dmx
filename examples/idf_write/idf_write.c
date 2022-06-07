/*

  ESP-IDF DMX Write

  Writes data to the DMX bus. The value of every byte in the DMX packet (except
  the start code) is incremented by 1 every 1 second.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 28 January 2021
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

// declare the user buffer to write DMX data
static uint8_t data[DMX_MAX_PACKET_SIZE] = {};

void app_main() {
  // use DMX port 2
  const dmx_port_t dmx_num = DMX_NUM_2;

  // configure the UART hardware to the default DMX settings
  const dmx_config_t dmx_config = DMX_DEFAULT_CONFIG;
  ESP_ERROR_CHECK(dmx_param_config(dmx_num, &dmx_config));

  // set communications pins
  ESP_ERROR_CHECK(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));

  // initialize the DMX driver without an event queue
  ESP_ERROR_CHECK(dmx_driver_install(dmx_num, DMX_MAX_PACKET_SIZE, 0, NULL, 1));

  // set DMX to TX mode
  dmx_set_mode(dmx_num, DMX_MODE_WRITE);

  // keeps track of how often we are logging messages to console
  uint32_t packet_counter = 0;
  uint8_t inc_value = 0;

  while (1) {
    // block until the packet is done being sent
    dmx_wait_send_done(dmx_num, DMX_PACKET_TIMEOUT_TICK);

    // transmit the packet on the DMX bus
    dmx_send_packet(dmx_num, DMX_MAX_PACKET_SIZE);

    // increment the packet counter
    ++packet_counter;

    // increment every data slot in the frame by 1 every 1 second (44fps)
    if (packet_counter >= 44) {
      ESP_LOGI(TAG, "incrementing data to %02X", ++inc_value);
      // don't increment the start code
      for (int i = 1; i < DMX_MAX_PACKET_SIZE; ++i) data[i]++;

      // write the packet to the DMX driver
      dmx_write_packet(dmx_num, data, DMX_MAX_PACKET_SIZE);

      // decrement our packet counter timer
      packet_counter -= 44;
    }
  }
}