/*

  ESP-IDF RDM Write

  This example uses RDM discovery to find devices on the RDM network. If devices
  are found, it iterates through each devices and sends a GET DEVICE_INFO
  request. If a response is received, the device's UID, DMX start address and
  DMX footprint is printed to the terminal.

  Discovery can take several seconds to complete, especially when there are
  several responder devices on the RDM network. For a more comprehensive
  explanation of RDM discovery, see the idf_rdm_discovery example.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 18 May 2023
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"

#define TX_PIN 17  // The DMX transmit pin.
#define RX_PIN 16  // The DMX receive pin.
#define EN_PIN 21  // The DMX transmit enable pin.

static const char *TAG = "main";

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_2;
  ESP_ERROR_CHECK(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));
  ESP_ERROR_CHECK(dmx_driver_install(dmx_num, DMX_DEFAULT_INTR_FLAGS));

  // Discover devices on the RDM network
  rdm_uid_t uids[32];
  int devices_found = rdm_discover_devices_simple(dmx_num, uids, 32);

  if (devices_found) {
    
    // Iterate through each device to send it a GET DEVICE_INFO request.
    for (int i = 0; i < devices_found; ++i) {
      rdm_header_t header = {
        .dest_uid = uids[i],
        .sub_device = RDM_SUB_DEVICE_ROOT
      };
      rdm_ack_t ack;
      rdm_device_info_t device_info;
      rdm_get_device_info(dmx_num, &header, &ack, &device_info);

      // Log the result
      if (ack.type == RDM_RESPONSE_TYPE_ACK) {
        ESP_LOGI(TAG,
                 "Device " UIDSTR
                 " has a DMX address of %i and a footprint of size %i.",
                 UID2STR(uids[i]), device_info.start_address,
                 device_info.footprint);
      } else if (ack.type == RDM_RESPONSE_TYPE_NACK_REASON) {
        ESP_LOGI(TAG,
                 "Device " UIDSTR " responded with a NACK reason of 0x%04x.\n",
                 ack.nack_reason);
        /* The list of NACK reasons is enumerated in the appendix of this
          library's README. */
      } else {
        ESP_LOGI(TAG, "Unable to get device info for " UIDSTR,
                 UID2STR(uids[i]));
      }
    }
  } else {
    // No devices were found during RDM discovery
    ESP_LOGE(TAG, "Could not find any RDM capable devices.");
  }
}
