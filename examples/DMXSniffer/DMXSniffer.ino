/*

  DMX Sniffer

  This sketch allows you to read and get detailed metadata DMX from a DMX
  controller using a standard DMX shield, such SparkFun ESP32 Thing Plus 
  DMX to LED Shield. This sketch was made for the Arduino framework!

  Created 10 September 2021
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include <Arduino.h>
#include <esp_dmx.h>

/* First, lets define the hardware pins that we are using with our ESP32. We
  need to define which pin is transmitting data and which pin is receiving data.
  DMX circuits also often need to be told when we are transmitting and when we are
  receiving data. We can do this by defining an enable pin. Since we are using
  the sniffer, we also need to define which pin to use to sniff DMX frames. It
  is best to use the lowest number pin as possible. The lowest number pin
  available on the Adafruit ESP32 Feather is 4, so lets use that. */
int transmitPin = 17;
int receivePin = 16;
int enablePin = 21;
int timingPin = 4;
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

/* The last variable that we need to read DMX is a queue handle. It's not
  important to know all the details about queues right now. All that you need to
  know is that when we receive a packet of DMX, our queue will be populated with
  an event that informs us that we've received new data. This allows us to wait
  until a new packet is received. Then we can then process the incoming data! */
QueueHandle_t queue;

/* The last few main variables that we need allow us to log data to the Serial
  Monitor. In this sketch, we want to log some information about the DMX we've
  received once every second. We'll declare a timer variable that will keep track
  of the amount of time that has elapsed since we last logged a serial message.
  We'll also want a flag that tracks if we are connected to the DMX controller so
  that we can log when we connect and when we disconnect. */
unsigned int timer = 0;
bool dmxIsConnected = false;

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
    how big our DMX packet is expected to be. We'll also pass to it our queue
    handle, some information about how many events we should store in our queue,
    and some interrupt priority information. Both the queue size and interrupt
    priority can be set to 1. */
  int queueSize = 1;
  int interruptPriority = 1;
  dmx_driver_install(dmxPort, DMX_MAX_PACKET_SIZE, queueSize, &queue,
                     interruptPriority);

  /* In this example, we are using the DMX sniffer. The sniffer uses an
    interrupt service routine (ISR) to create metadata about the DMX we
    receive. We will intall the ISR and then enable the sniffer on the hardware
    pin that we specified. */
  gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
  dmx_sniffer_enable(dmxPort, timingPin);
}


void loop() {
  /* We need a place to store information about the DMX packet we receive. We
    will use a dmx_event_t to store that packet information.  */
  dmx_event_t packet;

  /* And now we wait! The DMX standard defines the amount of time until DMX
    officially times out. That amount of time is converted into ESP32 clock ticks
    using the constant `DMX_PACKET_TIMEOUT_TICK`. If it takes longer than that
    amount of time to receive data, this if statement will evaluate to false. */
  if (xQueueReceive(queue, &packet, DMX_PACKET_TIMEOUT_TICK)) {

    /* If this code gets called, it means we've received DMX data! */

    /* We should check to make sure that there weren't any DMX errors. */
    if (packet.status == DMX_OK) {

      /* If this is the first DMX data we've received, lets log it! */
      if (!dmxIsConnected) {
        Serial.println("DMX connected!");
        dmxIsConnected = true;
      }

      /* We can read the DMX data into our buffer and increment our timer. */
      dmx_read_packet(dmxPort, data, packet.size);
      timer += packet.duration;

      /* Print a log message every 1 second (1,000,000 microseconds). */
      if (timer >= 1000000) {
        Serial.printf("Start Code: 0x%02X, Size: %i, Slot 1: 0x%02X, "
                      "Break: %ius, MAB: %ius Packet: %.2fms\n",
                      packet.start_code, packet.size, data[1], packet.timing.brk,
                      packet.timing.mab, packet.duration / 1000.0);
        timer -= 1000000;
      }

    } else {
      /* Uh-oh! Something went wrong receiving DMX. */
      Serial.println("DMX error!");
    }

  } else if (dmxIsConnected) {
    /* If the DMX timed out, but we were previously connected, we will assume
      that there is no more DMX to read. So we can uninstall the DMX driver. */
    Serial.println("DMX timed out! Uninstalling DMX driver...");
    dmx_driver_delete(dmxPort);

    /* Stop the program. */
    while (true) yield();
  }
}
