/*

  ESP-IDF RDM Discovery

  This example demonstrates how to perform RDM discovery, which is vital to be
  able to send RDM requests on an RDM network. Two functions are shown in this
  sketch: rdm_discover_devices_simple() and rdm_discover_with_callback(). The
  former simply discovers UIDs and places them in an array for users to save for
  later use. The latter runs the same algorithm, but allows the user to specify
  a callback which is called whenever a new RDM device is discovered.

  Discovery can take several seconds to complete, especially when there are
  several responder devices on the RDM network. In this example, the amount of
  time that discovery takes is measured and logged to the terminal.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 13 November 2022
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "rdm/controller.h"

#define TX_PIN 17  // The DMX transmit pin.
#define RX_PIN 16  // The DMX receive pin.
#define EN_PIN 21  // The DMX transmit enable pin.

static const char *TAG = "main";

void discover_cb(dmx_port_t dmx_num, rdm_uid_t uid, int num_found,
                 const rdm_disc_mute_t *mute, void *context) {
  /* This is a callback which is called during RDM discovery. It can be used to
    fetch additional device info when a device is discovered. It can also be
    used to increment a progress bar. In this example, we will just log that a
    new device has been found. The function rdm_discover_devices_simple() will
    not execute this callback. */
  ESP_LOGI(TAG, "Device %i has UID " UIDSTR, num_found, UID2STR(uid));
}

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {};
  const int personality_count = 0;
  dmx_driver_install(dmx_num, &config, personalities, personality_count);
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  /* RDM Discovery can take several seconds to complete. With just 1 RDM capable
    device in the RDM network, discovery should take around 30ms. */

  rdm_uid_t uids[32];
  const int64_t start = esp_timer_get_time();

  /* Call the default discovery implementation. This implementation searches for
    RDM devices on the network. When a device is found, its UID is added to an
    array of UIDs. This function will never overflow the UID array as long as
    the size argument is set properly. */
  size_t devices_found = rdm_discover_devices_simple(dmx_num, uids, 32);

  const int64_t stop = esp_timer_get_time();

  // Log how long discovery took and iterate through each UID
  ESP_LOGI(TAG, "Discovery took %.01fms and found %i device(s).",
           (stop - start) / 1000.0, devices_found);
  if (devices_found) {
    // Print the UID of each device found
    for (int i = 0; i < devices_found; ++i) {
      ESP_LOGI(TAG, "Device %i has UID " UIDSTR, i, UID2STR(uids[i]));
    }
  } else {
    ESP_LOGE(TAG, "Could not find any RDM capable devices.");
  }

  ESP_LOGI(TAG, "Starting discovery with callback...");

  /* Call discovery with a callback. This discovery implementation is exactly
    the same as rdm_discover_devices_simple() except that when a new device is
    discovered, a user-specified callback is called. */
  devices_found = rdm_discover_with_callback(dmx_num, discover_cb, NULL);
  ESP_LOGI(TAG, "Discovery completed and found %i device(s).", devices_found);
}
