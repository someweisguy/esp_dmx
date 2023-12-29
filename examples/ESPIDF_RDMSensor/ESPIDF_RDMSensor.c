/*

  ESP-IDF RDM Sensor

  This sketch creates an RDM sensor which keeps track of the amount of time in
  seconds that the ESP32 has been on. The sensor value will be accurate for
  approximately 18 hours (65535 seconds) before the value overflows.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 29 December 2023
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include <time.h>

#include "esp_dmx.h"
#include "esp_log.h"
#include "rdm/responder.h"

#define TX_PIN 17  // The DMX transmit pin.
#define RX_PIN 16  // The DMX receive pin.
#define EN_PIN 21  // The DMX transmit enable pin.

static const char *TAG = "main";

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {{1, "Default Personality"}};
  const int personality_count = 1;
  dmx_driver_install(dmx_num, &config, personalities, personality_count);
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  // Register sensors
  const int sensor_count = 1;
  rdm_callback_t callback = NULL;
  void *context = NULL;
  if (!rdm_register_sensor_value(dmx_num, sensor_count, callback, context)) {
    ESP_LOGE(TAG, "Unable to register sensors");
  } else {
    rdm_register_record_sensors(dmx_num, callback, context);
  }

  const int sensor_num = 0;
  const rdm_sub_device_t device_num = RDM_SUB_DEVICE_ROOT;

  // Set inital value of sensor
  time_t last_time = time(NULL);
  time_t current_time = last_time;
  rdm_sensor_set(dmx_num, device_num, sensor_num, current_time);

  // Continuously handle DMX and RDM packets
  dmx_packet_t packet;
  while (1) {
    if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK)) {
      if (packet.sc == DMX_SC) {
        ESP_LOGI(TAG, "Got DMX packet!");
      }
    }

    // Update the sensor value every 1 second
    current_time = time(NULL);
    if (current_time != last_time) {
      rdm_sensor_set(dmx_num, device_num, sensor_num, current_time);
      last_time = current_time;
    }
  }
}