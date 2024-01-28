/*

  ESP-IDF RDM Responder

  This example configures the ESP32 to act as an RDM responder. The required
  RDM responses are already configured, but this reconfigures the
  RDM_PID_IDENTIFY_DEVICE response. The identify device callback is configured
  to illuminate an LED on GPIO 13 when identify mode is active.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 19 June 2023
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include "driver/gpio.h"
#include "esp_dmx.h"
#include "esp_log.h"
#include "rdm/responder.h"

#define TX_PIN 17  // The DMX transmit pin.
#define RX_PIN 16  // The DMX receive pin.
#define EN_PIN 21  // The DMX transmit enable pin.

#define LED_PIN 13  // The LED pin to use for identification.

static const char *TAG = "main";

void custom_rdm_identify_cb(dmx_port_t dmx_num, rdm_header_t *request,
                            rdm_header_t *response, void *context) {
  // Illuminate the LED if the identify state is set to true
  if (request->cc == RDM_CC_SET_COMMAND) {
    bool identify;
    rdm_get_identify_device(dmx_num, &identify);
    gpio_set_level(LED_PIN, identify);
  }
}

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {1, "Default Personality"}
  };
  const int personality_count = 1;
  dmx_driver_install(dmx_num, &config, personalities, personality_count);
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  // Register identify device response and setup pins on success
  if (!rdm_register_identify_device(dmx_num, custom_rdm_identify_cb, NULL)) {
    ESP_LOGE(TAG, "Unable to register new identify device callback!");
  } else {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED_PIN, 0);
  }

  // Continuously handle DMX and RDM packets
  dmx_packet_t packet;
  while (1) {
    if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK)) {
      if (packet.sc == DMX_SC) {
        ESP_LOGI(TAG, "Got DMX packet!");
      } else if (packet.is_rdm) {
        rdm_send_response(dmx_num);
      }
    }

    // Do other work here...

  }

}
