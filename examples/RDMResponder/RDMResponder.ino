/*

  RDM Responder

  Configure this device to act as an RDM responder. This example registers a
  two custom RDM response callbacks: one for RDM_PID_SOFTWARE_VERSION_LABEL and
  one for RDM_PID_IDENTIFY_DEVICE. These PIDs are already registered when
  installing the DMX driver, but are overwritten for demonstration purposes.
  Afterwards, this example loops continuously in a simple DMX receive loop which
  allows for processing and responding to RDM requests.

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

/* Next, lets decide which DMX port to use. The ESP32 has either 2 or 3 ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 1! */
dmx_port_t dmxPort = 1;

/* The two RDM responses we will register are RDM_PID_SOFTWARE_VERSION_LABEL and
  RDM_PID_IDENTIFY_DEVICE. These responses are already registered when calling
  dmx_driver_install() but we can overwrite them. */

/* To overwrite RDM_PID_SOFTWARE_VERSION_LABEL we must, of course, define a new
  software version label! We will define it below. */
const char *softwareVersionLabel = "My Custom Software!";

/* To overwrite RDM_PID_IDENTIFY_DEVICE, we will define a callback which will
  illuminate an LED when the identify device mode is active, and extinguish the
  LED when identify is inactive. Don't forget to also declare which pin your LED
  is using! */
int ledPin = 13;
void rdmIdentifyCallback(dmx_port_t dmxPort, const rdm_header_t *header,
                         void *context) {
  // Illuminate the LED if the identify state is set to true
  uint8_t identify;
  rdm_get_identify_device(dmx_num, &identify);
  digitalWrite(ledPin, identify);
  Serial.printf("Identify mode is %s.\n", identify ? "on" : "off");
}

void setup() {
  /* Start the serial connection back to the computer so that we can log
   messages to the Serial Monitor. Lets set the baud rate to 115200. */
  Serial.begin(115200);

  /* Set the DMX hardware pins to the pins that we want to use. */
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  /* Now we can install the DMX driver! We'll tell it which DMX port to use, 
    what device configure to use, and which interrupt priority it should have. 
    If you aren't sure which configuration or interrupt priority to use, you can
    use the macros `DMX_CONFIG_DEFAULT` and `DMX_INTR_FLAGS_DEFAULT` to set the
    configuration and interrupt to their default settings. */
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmxPort, &config, DMX_INTR_FLAGS_DEFAULT);

  /* Register the custom RDM_PID_SOFTWARE_VERSION_LABEL callback. This
    overwrites the default response. If you would like you may add a callback
    function and a context which will be called when receiving this request. For
    now, we will leave these NULL. */
  rdm_register_software_version_label(dmxPort, softwareVersionLabel, NULL,
                                      NULL);

  /* Register the custom RDM_PID_IDENTIFY_DEVICE callback. This overwrites the
    default response. Since we aren't using a user context in the callback, we
    can pass NULL as the final argument. Don't forget to set the pin mode for
    your LED pin! */
  rdm_register_identify_device(dmxPort, rdmIdentifyCallback, NULL);
  pinMode(ledPin, OUTPUT);

  /* Care should be taken to ensure that the parameters registered for callbacks
    never go out of scope. The variables passed as parameter data for responses
    must be valid throughout the lifetime of the DMX driver. Allowing parameter
    variables to go out of scope can result in undesired behavior during RDM
    response callbacks. */
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
