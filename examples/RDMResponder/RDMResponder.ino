/*

  RDM Responder

  Configure this device to act as an RDM responder. This example sets the
  device's device info, which defines many of the device's basic capabilities.
  This example also registers a custom callback for RDM_PID_DMX_START_ADDRESS
  requests. This PID is already registered when installing the DMX driver, but
  is overwritten for demonstration purposes. Afterwards, this example loops
  continuously in a simple DMX receive loop which allows for processing and
  responding to RDM requests.

  Created 20 June 2023
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include <Arduino.h>
#include <esp_dmx.h>
#include <rdm/responder.h>

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

// TODO: docs
int ledPin = 13;  // The LED pin to use for identification.

/* Next, lets decide which DMX port to use. The ESP32 has either 2 or 3 ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 1! */
dmx_port_t dmxPort = 1;

// TODO: docs
const char *softwareVersionLabel = "My Custom Software!";

void rdmIdentifyCallback(dmx_port_t dmx_num, bool identify, void *context) {
  // Illuminate the LED if the identify state is set to true
  digitalWrite(ledPin, identify);
  
  const char *statusString;
  if (identify) {
    statusString = "on";
  } else {
    statusString = "off";
  }
  Serial.printf("Identify mode is %s!", statusString);
}

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


  /* Register the custom callback. This overwrites the default
    RDM_PID_DMX_START_ADDRESS response. */
  rdm_register_software_version_label(dmxPort, softwareVersionLabel);
  
  // TODO: docs
  pinMode(ledPin, OUTPUT);

  // TODO: docs
  rdm_register_identify_device(dmxPort, rdmIdentifyCallback, NULL);

  /* Care should be taken to ensure that the user context never goes out of
    scope. Allowing this to happen can lead to undesired behavior. User contexts
    are not copied into the driver. In this example, we won't need to worry
    about this because the user context is NULL. */
}

void loop() {
  /* We need a place to store information about the DMX packets we receive. We
    will use a dmx_packet_t to store that packet information.  */
  dmx_packet_t packet;

  /* Now we will block until data is received. If an RDM request for this device
    is received, the dmx_receive() function will automatically respond to the
    requesting RDM device with the appropriate callback. */
  dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK);

  /* Typically, you would handle your packet information here. Since this is
    just an example, this section has been left blank. */
}
