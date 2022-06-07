/*

  DMX Write

  This sketch allows you to write DMX to a DMX listener using a standard DMX
  shield, such SparkFun ESP32 Thing Plus DMX to LED Shield. This sketch was 
  made for the Arduino framework!

  Created 10 September 2021
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include <Arduino.h>
#include <esp_dmx.h>

/* First, lets define the hardware pins that we are using with our ESP32. We
  need to define which pin is transmitting data and which pin is receiving data.
  DMX circuits also often need to be told when we are transmitting and when we are
  receiving data. We can do this by defining an enable pin. */
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

/* Now we want somewhere to store our DMX data. Since a single packet of DMX
  data can be up to 513 bytes long, we want our array to be at least that long.
  This library knows that the max DMX packet size is 513, so we can fill in the
  array size with `DMX_MAX_PACKET_SIZE`. */
byte data[DMX_MAX_PACKET_SIZE];

/* The last few main variables that we need allow us to log data to the Serial
  Monitor. In this sketch, we want to log some information about the DMX we are
  transmitting once every 44 packets. We'll declare a packet counter variable
  that will keep track of the amount of packets that have been sent since we
  last logged a serial message. We'll also want to increment the value of each
  byte we send so we'll declare a variable to track what value we should
  increment to. */
int packetCounter = 44;
byte incrementValue = 0;

void setup() {
  /* Start the serial connection back to the computer so that we can log
     messages to the Serial Monitor. Lets set the baud rate to 115200. */
  Serial.begin(115200);

  /* Configure the DMX hardware to the default DMX settings and tell the DMX
      driver which hardware pins we are using. */
  dmx_config_t dmxConfig = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmxPort, &dmxConfig);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  /* Now we can install the DMX driver! We'll tell it which DMX port to use and
    how big our DMX packet is expected to be. Typically, we'd pass it a handle
    to a queue, but since we are only transmitting DMX, we don't need a queue.
    We can write `NULL` where we'd normally put our queue handle. We'll also
    pass some interrupt priority information. The interrupt priority can be set
    to 1. */
  int queueSize = 0;
  int interruptPriority = 1;
  dmx_driver_install(dmxPort, DMX_MAX_PACKET_SIZE, queueSize, NULL,
                     interruptPriority);

  /* Finally, since we are transmitting DMX, we should tell the DMX driver that
    we are transmitting, not receiving. We should also set our DMX start code
    to 0.*/
  dmx_set_mode(dmxPort, DMX_MODE_WRITE);
  data[0] = 0;
}

void loop() {
  if (packetCounter >= 44) {

    /* Increment every byte in our packet to the new increment value. Notice
      we don't increment the zeroeth byte, since that is our DMX start code.
      Then we must write our changes to the DMX packet. */
    for (int i = 1; i < DMX_MAX_PACKET_SIZE; ++i) {
      data[i] = incrementValue;
    }
    dmx_write_packet(dmxPort, data, DMX_MAX_PACKET_SIZE);

    /* Log a message to the Serial Monitor, decrement the packet counter, and
      increment our increment value. */
    Serial.printf("Sending DMX 0x%02X\n", incrementValue);
    packetCounter -= 44;
    incrementValue++;
  }

  /* Now we can transmit the DMX packet! */
  dmx_send_packet(dmxPort, DMX_MAX_PACKET_SIZE);
  packetCounter++;

  /* We can do some other work here if we want! */
  // Do other work here...

  /* If we have no more work to do, we will wait until we are done sending our
    DMX packet. */
  dmx_wait_send_done(dmxPort, DMX_PACKET_TIMEOUT_TICK);
}
