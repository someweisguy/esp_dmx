/*

  RDM Write

  This example uses RDM discovery to find devices on the RDM network. If devices
  are found, it iterates through each devices and sends a GET DEVICE_INFO
  request. If a response is received, the device's UID, DMX start address and
  DMX footprint is printed to the terminal.

  Discovery can take several seconds to complete, especially when there are
  several responder devices on the RDM network. For a more comprehensive
  explanation of RDM discovery, see the idf_rdm_discovery example.

  Created 19 May 2023
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include <Arduino.h>
#include <esp_dmx.h>
#include <rdm/controller.h>

/* First, lets define the hardware pins that we are using with our ESP32. We
  need to define which pin is transmitting data and which pin is receiving data.
  DMX circuits also often need to be told when we are transmitting and when we
  are receiving data. We can do this by defining an enable pin. */
int transmitPin = 17;
int receivePin = 16;
int enablePin = 21;
/* Make sure to double-check that these pins are compatible with your ESP32!
  Some ESP32s, such as the ESP32-WROVER series, do not allow you to read or
  write data on pins 16 or 17, so it's always good to read the manuals. */

/* Next, lets decide which DMX port to use. The ESP32 has either 2 or 3 ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 1! */
dmx_port_t dmxPort = 1;

/* Now lets allocate an array of UIDs to store the UIDs of any RDM devices that
  we may find. 32 should be more than plenty, but this can be increased if
  desired! */
rdm_uid_t uids[32];

void setup() {
  /* Start the serial connection back to the computer so that we can log
   messages to the Serial Monitor. Lets set the baud rate to 115200. */
  Serial.begin(115200);

  /* Set the DMX hardware pins to the pins that we want to use. */
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  /* Now we can install the DMX driver! We'll tell it which DMX port to use and
    which interrupt priority it should have. If you aren't sure which interrupt
    priority to use, you can use the macro `DMX_DEFAULT_INTR_FLAG` to set the
    interrupt to its default settings.*/
  dmx_driver_install(dmxPort, DMX_DEFAULT_INTR_FLAGS);

  /* Call the default discovery implementation. This implementation searches for
    RDM devices on the network. When a device is found, its UID is added to an
    array of UIDs. This function will never overflow the UID array as long as
    the size argument is set properly. */
  rdm_uid_t uids[32];
  size_t devices_found = rdm_discover_devices_simple(dmx_num, uids, 32);

  if (devices_found) {
    // Print the UID of each device found
    for (int i = 0; i < devices_found; ++i) {
      ESP_LOGI(TAG, "Device %i has UID " UIDSTR, i, UID2STR(uids[i]));
      rdm_header_t header = {.dest_uid = uids[0]};

      rdm_ack_t ack;

      // Get the device info
      rdm_device_info_t device_info;
      if (rdm_send_get_device_info(dmx_num, &header, &device_info, &ack)) {
        ESP_LOGI(TAG,
                 "DMX Footprint: %i, Sub-device count: %i, Sensor count: %i",
                 device_info.footprint, device_info.sub_device_count,
                 device_info.sensor_count);
      }

      // Get the software version label
      char sw_label[33];
      if (rdm_send_get_software_version_label(dmx_num, &header, sw_label, 32,
                                              &ack)) {
        ESP_LOGI(TAG, "Software version label: %s", sw_label);
      }

      // Get and set the identify state
      uint8_t identify;
      if (rdm_send_get_identify_device(dmx_num, &header, &identify, &ack)) {
        ESP_LOGI(TAG, UIDSTR " is%s identifying.", UID2STR(uids[0]),
                 identify ? "" : " not");

        identify = !identify;
        if (rdm_send_set_identify_device(dmx_num, &header, identify, &ack)) {
          ESP_LOGI(TAG, UIDSTR " is%s identifying.", UID2STR(uids[0]),
                   identify ? "" : " not");
        }
      }

      // Get and set the DMX start address
      uint16_t dmx_start_address = 0;
      if (rdm_send_get_dmx_start_address(dmx_num, &header, &dmx_start_address,
                                         &ack)) {
        ESP_LOGI(TAG, "DMX start address is %i", dmx_start_address);

        ++dmx_start_address;
        if (dmx_start_address > 512) {
          dmx_start_address = 1;
        }
        if (rdm_send_set_dmx_start_address(dmx_num, &header, dmx_start_address,
                                           &ack)) {
          ESP_LOGI(TAG, "DMX address has been set to %i", dmx_start_address);
        }
      }
    }

  } else {
    ESP_LOGE(TAG, "Could not find any RDM capable devices.");
  }
}

void loop() {
  /* The main loop has been left empty for this example. Feel free to add your
    own DMX loop here! */
}
