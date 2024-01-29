/*

  RDM Controller

  This example uses RDM discovery to find devices on the RDM network. If devices
  are found, it iterates through each devices and sends several RDM requests. If
  a response is received, the response is printed to the Serial Monitor.

  Discovery can take several seconds to complete, especially when there are
  several responder devices on the RDM network. For a more comprehensive
  explanation of RDM discovery, see the RDMDiscovery example.

  Created 20 June 2023
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

  /* Now we will install the DMX driver! We'll tell it which DMX port to use,
    what device configuration to use, and what DMX personalities it should have.
    If you aren't sure which configuration to use, you can use the macros
    `DMX_CONFIG_DEFAULT` to set the configuration to its default settings.
    Because the device is being setup as an RDM controller, this device won't
    use any DMX personalities. */
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {};
  int personality_count = 0;
  dmx_driver_install(dmxPort, &config, personalities, personality_count);

  /* Now set the DMX hardware pins to the pins that we want to use and DMX
    driver setup will be complete! */
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  /* RDM Discovery can take several seconds to complete. Don't panic if it seems
    like your device freezes! */

  /* Call the default discovery implementation. This implementation searches for
    RDM devices on the network. When a device is found, its UID is added to an
    array of UIDs. This function will never overflow the UID array as long as
    the size argument is set properly. */
  int devicesFound = rdm_discover_devices_simple(dmxPort, uids, 32);

  /* If any devices were found during discovery, lets iterate through them. */
  for (int i = 0; i < devicesFound; ++i) {
    Serial.printf("Device %i has UID " UIDSTR "\n", i, UID2STR(uids[i]));

    /* Now we will send RDM requests to the devices we found. We first need to
      address our requests to the proper device and sub-device. We will get a
      pointer to one of the UIDs we got in discovery to properly address our RDM
      requests. We will address our requests to the root RDM sub device. We will
      also declare an RDM ACK to get information about RDM responses, but this
      isn't necessary if it is not desired. */
    rdm_uid_t destUID = uids[i];
    rdm_sub_device_t subDevice = RDM_SUB_DEVICE_ROOT;
    rdm_ack_t ack;

    /* First, we will send a request to get the device information of our RDM
      device. We can pass our DMX port and pointers to our header, device info,
      and ACK to our request function. If the request is successful, we will
      print out some of the device information we received. */
    rdm_device_info_t deviceInfo;
    if (rdm_send_get_device_info(dmxPort, &destUID, subDevice, &deviceInfo,
                                 &ack)) {
      Serial.printf(
          "DMX Footprint: %i, Sub-device count: %i, Sensor count: %i\n",
          deviceInfo.footprint, deviceInfo.sub_device_count,
          deviceInfo.sensor_count);
    }

    /* Second, we will send a request to get the device's software version
      label. Strings in RDM typically have a maximum length of 32 characters. We
      should allocate space for 32 characters and one null terminator. A
      constant, RDM_ASCII_SIZE_MAX, can be used! */
    char softwareVersionLabel[RDM_ASCII_SIZE_MAX];
    int softwareVersionLabelSize = RDM_ASCII_SIZE_MAX;
    if (rdm_send_get_software_version_label(dmxPort, &destUID, subDevice,
                                            softwareVersionLabel,
                                            softwareVersionLabelSize, &ack)) {
      Serial.printf("Software version label: %s\n", softwareVersionLabel);
    }

    /* Now we will get and set the identify device parameter. Unlike the
      previous two parameters, identify device can be both get and set. We will
      first get the identify device parameter and set it to its opposite
      state. */
    bool identify;
    if (rdm_send_get_identify_device(dmxPort, &destUID, subDevice, &identify,
                                     &ack)) {
      Serial.printf(UIDSTR " is%s identifying.\n", UID2STR(destUID),
                    identify ? "" : " not");

      /* Set the identify device parameter to its opposite state. */
      identify = !identify;
      if (rdm_send_set_identify_device(dmxPort, &destUID, subDevice, identify,
                                       &ack)) {
        Serial.printf(UIDSTR " is now%s identifying.\n", UID2STR(destUID),
                      identify ? "" : " not");
      }
    }

    /* Finally, we will get and set the DMX start address. It is not required
      for all RDM devices to support the DMX start address parameter so it is
      possible (but unlikely) that your device does not support this parameter.
      After getting the DMX start address, we will increment it by one. */
    uint16_t dmxStartAddress;
    if (rdm_send_get_dmx_start_address(dmxPort, &destUID, subDevice,
                                       &dmxStartAddress, &ack)) {
      Serial.printf("DMX start address is %i\n", dmxStartAddress);

      /* Increment the DMX start address and ensure it is between 1 and 512. */
      ++dmxStartAddress;
      if (dmxStartAddress > 512) {
        dmxStartAddress = 1;
      }

      /* Set the updated DMX start address! */
      if (rdm_send_set_dmx_start_address(dmxPort, &destUID, subDevice,
                                         dmxStartAddress, &ack)) {
        Serial.printf("DMX address has been set to %i\n", dmxStartAddress);
      }
    } else if (ack.type == RDM_RESPONSE_TYPE_NACK_REASON) {
      /* In the event that the DMX start address request fails, print the reason
        for the failure. The NACK reason and other information on the response
        can be found in the ack variable we declared earlier. */
      Serial.printf(UIDSTR " GET DMX_START_ADDRESS NACK reason: 0x%02x\n",
                    ack.src_uid, ack.nack_reason);
    }
  }

  if (devicesFound == 0) {
    /* Oops! No RDM devices were found. Double-check your DMX connections and
      try again. */
    Serial.printf("Could not find any RDM capable devices.\n");
  }
}

void loop() {
  /* The main loop has been left empty for this example. Feel free to add your
    own DMX loop here! */
}
