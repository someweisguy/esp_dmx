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
  int devicesFound = rdm_discover_devices_simple(dmxPort, uids, 32);
  if (devicesFound) {
    /* If this code runs, that means we've found some devices! Now we want to
      iterate through the number of devices we've found and send an RDM request
      to each UID that is on the network. We will be sending GET DEVICE_INFO
      requests! */
    for (int i = 0; i < devicesFound; ++i) {
      /* We will instantiate a rdm_header_t so that we can properly address our
        messages to the correct UID and sub-device. We will set the destination
        UID to the i-th UID that we found and set the sub-device to the root
        device. */
      rdm_header_t header = {.dest_uid = uids[i],
                             .sub_device = RDM_SUB_DEVICE_ROOT};

      /* Now we will want somewhere to store the acknowledgement of our request
        (also known as an ACK) and the device info that we are requesting. For
        this, we will need an rdm_ack_t and an rdm_device_info_t. Because these
        variables will be filled out by the response to our request, we can just
        declare the variables without defining them. */
      rdm_ack_t ack;
      rdm_device_info_t deviceInfo;

      /* Now we are ready to send the request! */
      rdm_get_device_info(dmxPort, &header, &ack, &deviceInfo);

      /* Our request has been sent so now we should check to see if a response
        was successfully received. Each RDM request function returns the number
        of bytes that were received in the response, but that doesn't
        necessarily mean that the response was valid. This is what we use the
        rdm_ack_t for! We can check the ack.type to ensure that we received an
        RDM_RESPONSE_TYPE_ACK. If we received any other ack type, that means our
        request did not receive a successful response. */
      if (ack.type == RDM_RESPONSE_TYPE_ACK) {
        /* We received a valid response. Log the results! */
        Serial.printf("Device " UIDSTR
                      " has a DMX address of %i and a footprint of size %i.\n",
                      UID2STR(uids[i]), deviceInfo.start_address,
                      deviceInfo.footprint);
      } else if (ack.type == RDM_RESPONSE_TYPE_NACK_REASON) {
        /* The responder responded to our request by saying it was unable to
          process the request. We can learn more by logging the NACK reason in
          the rdm_ack_t. */
        Serial.printf("Device " UIDSTR
                      " responded with a NACK reason of 0x%04x.\n",
                      ack.nack_reason);
        /* The list of NACK reasons is enumerated in the appendix of this
          library's README. */
      } else {
        /* Uh-oh. We didn't receive a valid response. This can happen for
          various reasons including issues with the RDM network or a
          non-compliant RDM device. Some RDM request are not supported by all
          RDM devices. All RDM devices are required to support GET DEVICE_INFO
          requests. If you get this log message and are able to rule out a bad
          DMX connection, it may be possible that your RDM device is
          non-compliant. */
        Serial.printf("Unable to get device info for " UIDSTR ".\n",
                      UID2STR(uids[i]));
      }
    }
  } else {
    /* Oops! No RDM devices were found. Double-check your DMX connections and
      try again. */
    Serial.println("Could not find any RDM capable devices.");
  }
}

void loop() {
  /* The main loop has been left empty for this example. Feel free to add your
    own DMX loop here! */
}
