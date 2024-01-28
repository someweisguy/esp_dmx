/*

  ESP-IDF RDM Controller

  This example uses RDM discovery to find devices on the RDM network. If devices
  are found, it iterates through each devices and sends several RDM requests. If
  a response is received, the response is printed to the terminal.

  Discovery can take several seconds to complete, especially when there are
  several responder devices on the RDM network. For a more comprehensive
  explanation of RDM discovery, see the idf_rdm_discovery example.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 19 June 2023
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include "esp_dmx.h"
#include "esp_log.h"
#include "rdm/controller.h"

#define TX_PIN 17  // The DMX transmit pin.
#define RX_PIN 16  // The DMX receive pin.
#define EN_PIN 21  // The DMX transmit enable pin.

static const char *TAG = "main";

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  const int personality_count = 0;
  dmx_driver_install(dmx_num, &config, NULL, personality_count);
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  rdm_uid_t uids[32];
  size_t devices_found = rdm_discover_devices_simple(dmx_num, uids, 32);

  if (devices_found) {
    // Print the UID of each device found
    for (int i = 0; i < devices_found; ++i) {
      ESP_LOGI(TAG, "Device %i has UID " UIDSTR, i, UID2STR(uids[i]));
      const rdm_uid_t *dest_uid = &uids[i];
      const rdm_sub_device_t sub_device = RDM_SUB_DEVICE_ROOT;

      rdm_ack_t ack;

      // Get the device info
      rdm_device_info_t device_info;
      if (rdm_send_get_device_info(dmx_num, dest_uid, sub_device, &device_info,
                                   &ack)) {
        ESP_LOGI(TAG,
                 "DMX Footprint: %i, Sub-device count: %i, Sensor count: %i",
                 device_info.footprint, device_info.sub_device_count,
                 device_info.sensor_count);
      }

      // Get the software version label
      char sw_label[33];
      size_t sw_label_size = sizeof(sw_label);
      if (rdm_send_get_software_version_label(dmx_num, dest_uid, sub_device,
                                              sw_label, sw_label_size, &ack)) {
        ESP_LOGI(TAG, "Software version label: %s", sw_label);
      }

      // Get and set the identify state
      bool identify;
      if (rdm_send_get_identify_device(dmx_num, dest_uid, sub_device, &identify,
                                       &ack)) {
        ESP_LOGI(TAG, UIDSTR " %s identifying.", UID2STR(*dest_uid),
                 identify ? "is" : "is not");

        identify = !identify;
        if (rdm_send_set_identify_device(dmx_num, dest_uid, sub_device,
                                         identify, &ack)) {
          ESP_LOGI(TAG, UIDSTR " %s identifying.", UID2STR(*dest_uid),
                   identify ? "is" : "is not");
        }
      }

      // Get and set the DMX start address
      uint16_t dmx_start_address = 0;
      if (rdm_send_get_dmx_start_address(dmx_num, dest_uid, sub_device,
                                         &dmx_start_address, &ack)) {
        ESP_LOGI(TAG, "DMX start address is %i", dmx_start_address);

        ++dmx_start_address;
        if (dmx_start_address > 512) {
          dmx_start_address = 1;
        }
        if (rdm_send_set_dmx_start_address(dmx_num, dest_uid, sub_device,
                                           dmx_start_address, &ack)) {
          ESP_LOGI(TAG, "DMX address has been set to %i", dmx_start_address);
        }
      }
    }

  } else {
    ESP_LOGE(TAG, "Could not find any RDM capable devices.");
  }
}
