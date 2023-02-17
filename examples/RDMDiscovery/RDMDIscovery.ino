/*

  RDM Discovery

  This example demonstrates how to perform RDM discovery with a standard DMX
  shield, such SparkFun ESP32 Thing Plus DMX to LED Shield. This sketch was
  made for the Arduino framework!

  Created 26 November 2022
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

  /* RDM Discovery can take several seconds to complete. With just 1 RDM capable
    device in the RDM network, discovery should take around 30ms. */

  /* Call the default discovery implementation. This implementation searches for
    RDM devices on the network. When a device is found, its UID is added to an
    array of UIDs. This function will never overflow the UID array as long as
    the size argument is set properly. */
  size_t devicesFound = rdm_discover_devices_simple(dmxPort, uids, 32);
  if (devicesFound) {
    /* Now we'll print the devices we found to the Serial Monitor! */
    Serial.printf("Discovery found %i device(s).\n", devicesFound);
    for (int i = 0; i < devicesFound; i++) {
      /* Log each UID that we found! */
      Serial.printf("Device %i has UID " UIDSTR "\n", i, UID2STR(uids[i]));
    }
  } else {
    /* Oops! No RDM devices were found. Perhaps double-check your DMX 
      connections and try again. */
    Serial.println("Could not find any RDM capable devices.");
  }
}

void loop() {
  /* Discovery should be performed periodically, but shouldn't be called too
    frequently because it takes a long time and because it probably isn't
    necessary to constantly discover new devices. Therefore, we'll leave the
    main event loop empty. Feel free to put your own DMX loop here! */
}
