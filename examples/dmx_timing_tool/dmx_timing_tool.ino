/*
* This example demonstrates how to use the DMX timing tool to read packet
* metadata from incoming DMX data packets.
*/

#include "esp_dmx.h"

#define TX_PIN 17   // the pin we are using to TX with
#define RX_PIN 16   // the pin we are using to RX with
#define EN_PIN 21   // the pin we are using to enable TX on the DMX transceiver
#define TI_PIN  4   // the pin we are using as the timing tool interrupt

// declare the user buffer to read in DMX data
static uint8_t data[DMX_MAX_PACKET_SIZE] = {};
static const dmx_port_t dmx_num = DMX_NUM_2;
static QueueHandle_t queue;

// keeps track of how often we are logging messages to console
uint32_t timer = 0;
  
// allows us to know when the packet times out after it connects
bool timeout = true;

void setup() {
  Serial.begin(9600); 

  // configure the UART hardware to the default DMX settings
  const dmx_config_t dmx_config = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmx_num, &dmx_config);

  // set communications pins
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  // initialize the DMX driver with an event queue to read data
  dmx_driver_install(dmx_num, DMX_MAX_PACKET_SIZE, 1, &queue, 1);

  // install the default GPIO ISR
  gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);

  // enable the rx timing tool
  dmx_rx_timing_enable(dmx_num, TI_PIN);
}


void loop() {
  dmx_event_t packet;
  // wait until a packet is received or times out
  if (xQueueReceive(queue, &packet, DMX_RX_PACKET_TOUT_TICK)) {
    
    if (packet.status == DMX_OK) {
      // print a message upon initial DMX connection
      if (timeout) {
        printf("dmx connected\n");
        timeout = false; // establish connection!
      }

      // read the packet into the data buffer
      dmx_read_packet(dmx_num, data, packet.size);
      
      // increment the amount of time that has passed since the last packet
      timer += packet.duration;

      // print a log message every 1 second (1000000 us)
      if (timer >= 1000000) {
        printf(
            "Start Code: 0x%02X, Size: %i, Slot 1: 0x%02X, Break: %ius, MAB: "
            "%ius Packet: %.2fms\n",
            packet.start_code, packet.size, data[1], packet.timing.brk,
            packet.timing.mab, packet.duration / 1000.0);
        timer -= 1000000;
      }

    } else if (packet.status != DMX_OK) {
      // something went wrong receiving data
      printf("dmx error\n");
    }

  } else if (timeout == false) {
    // lost connection
    printf("lost dmx signal\n");
    // uninstall the DMX driver
    printf("uninstalling DMX driver\n");
    dmx_driver_delete(dmx_num);
    printf("terminating program\n");
    while (true) yield();
  }
}


