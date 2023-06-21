
#include <Arduino.h>
#include "esp_dmx.h"
#include "rdm/controller.h"


int transmitPin = 17;
int receivePin = 16;
int enablePin = 21;

dmx_port_t dmxPort = 1;


rdm_uid_t uids[32];

void setup() {

  Serial.begin(115200);

  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  dmx_driver_install(dmxPort, DMX_DEFAULT_INTR_FLAGS);

  size_t devices_found = rdm_discover_devices_simple(dmxPort, uids, 32);

  if (devices_found) {
    for (int i = 0; i < devices_found; ++i) {
      Serial.printf("Device %i has UID " UIDSTR "\n", i, UID2STR(uids[i]));
      rdm_header_t header = { .dest_uid = uids[0] };

      rdm_ack_t ack;

      // Get the device info
      rdm_device_info_t device_info;
      if (rdm_send_get_device_info(dmxPort, &header, &device_info, &ack)) {
        Serial.printf("DMX Footprint: %i, Sub-device count: %i, Sensor count: %i\n",
                      device_info.footprint, device_info.sub_device_count,
                      device_info.sensor_count);
      }

      // Get the software version label
      char sw_label[33];
      if (rdm_send_get_software_version_label(dmxPort, &header, sw_label, 32,
                                              &ack)) {
        Serial.printf("Software version label: %s\n", sw_label);
      }

      // Get and set the identify state
      uint8_t identify;
      if (rdm_send_get_identify_device(dmxPort, &header, &identify, &ack)) {

        Serial.printf(UIDSTR " is%s identifying.\n", UID2STR(uids[0]), identify ? "" : " not");

        identify = !identify;
        if (rdm_send_set_identify_device(dmxPort, &header, identify, &ack)) {
          Serial.printf(UIDSTR " is%s identifying.\n", UID2STR(uids[0]),
                        identify ? "" : " not");
        }
      }

      // Get and set the DMX start address
      uint16_t dmx_start_address = 0;
      if (rdm_send_get_dmx_start_address(dmxPort, &header, &dmx_start_address,
                                         &ack)) {
        Serial.printf("DMX start address is %i\n", dmx_start_address);

        ++dmx_start_address;
        if (dmx_start_address > 512) {
          dmx_start_address = 1;
        }
        if (rdm_send_set_dmx_start_address(dmxPort, &header, dmx_start_address,
                                           &ack)) {
          Serial.printf("DMX address has been set to %i\n", dmx_start_address);
        }
      }
    }

  } else {
    Serial.printf("Could not find any RDM capable devices.\n");
  }
}

void loop() {
  /* The main loop has been left empty for this example. Feel free to add your
    own DMX loop here! */
}
