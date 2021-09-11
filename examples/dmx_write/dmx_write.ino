/*
 * Writes data to the DMX bus. Increments the value of each byte in the written
 * packet every 1 second.
 */

#include "esp_dmx.h"

#define TX_PIN 17  // the pin we are using to TX with
#define RX_PIN 16  // the pin we are using to RX with
#define EN_PIN 21  // the pin we are using to enable TX on the DMX transceiver

// declare the user buffer to write DMX data
static byte data[DMX_MAX_PACKET_SIZE] = {};

// use DMX port 2
static const dmx_port_t dmx_num = DMX_NUM_2;

// keeps track of how often we are logging messages to console
static unsigned int packet_counter = 0;
static byte inc_value = 0;

void setup() {

  Serial.begin(9600);
  
  // configure the UART hardware to the default DMX settings
  const dmx_config_t dmx_config = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmx_num, &dmx_config);

  // set communications pins
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  // initialize the DMX driver without an event queue
  dmx_driver_install(dmx_num, DMX_MAX_PACKET_SIZE, 0, NULL, 1);

  // set DMX to TX mode
  dmx_set_mode(dmx_num, DMX_MODE_TX);
}

void loop() {
  // block until the packet is done being sent
  dmx_wait_tx_done(dmx_num, DMX_TX_PACKET_TOUT_TICK);

  // transmit the packet on the DMX bus
  dmx_tx_packet(dmx_num);

  // increment the packet counter
  ++packet_counter;

  // increment every data slot in the frame by 1 every 1 second (44fps)
  if (packet_counter >= 44) {
    printf("incrementing data to 0x%02X\n", ++inc_value);
    
    // don't increment the start code
    for (int i = 1; i < DMX_MAX_PACKET_SIZE; ++i) data[i]++;

    // write the packet to the DMX driver
    dmx_write_packet(dmx_num, data, DMX_MAX_PACKET_SIZE);

    // decrement our packet counter timer
    packet_counter -= 44;
  }
}
