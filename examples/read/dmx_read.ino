#include "esp_dmx.h"

#define TX_PIN 17   // the pin we are using to TX with
#define RX_PIN 16   // the pin we are using to RX with
#define EN_PIN 21   // the pin we are using to enable TX on the DMX transceiver

static uint8_t data[DMX_MAX_PACKET_SIZE] = {}; // declare the user buffer to read in DMX data
static const dmx_port_t dmx_num = DMX_NUM_2;   // use DMX port 2
static uint32_t timer = 0;                     // keeps track of how often we are logging messages to console
static bool timeout = true;                    // allows us to know when the packet times out after it connects
static QueueHandle_t queue;                    // queue handle to know when we've received a packet of DMX

void setup() {
  // start serial communication back to the host computer
  Serial.begin(9600);
  
  // configure the UART hardware to the default DMX settings
  const dmx_config_t dmx_config = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmx_num, &dmx_config);

  // set communications pins
  dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN);

  // initialize the DMX driver with an event queue to read data
  dmx_driver_install(dmx_num, DMX_MAX_PACKET_SIZE, 1, &queue, 1);
}

void loop() {
  dmx_event_t packet;
  // wait until a packet is received or times out
  if (xQueueReceive(queue, &packet, DMX_RX_PACKET_TOUT_TICK)) {
    
    if (packet.status == DMX_OK) {
      // print a message upon initial DMX connection
      if (timeout) {
        ESP_LOGI(TAG, "dmx connected");
        timeout = false; // establish connection!
      }

      // read the packet into the data buffer
      dmx_read_packet(dmx_num, data, packet.size);
      
      // increment the amount of time that has passed since the last packet
      timer += packet.duration;

      // print a log message every 1 second (1000000 us)
      if (timer >= 1000000) {
        printf("start code is 0x%02X and address 1 is 0x%02X\n", data[0], data[1]);
        timer -= 1000000;
      }

    } else if (packet.status != DMX_OK) {
      // something went wrong receiving data
      printf("dmx error\n");
    }

  } else if (timeout == false) {
    // lost connection
    printf("DMX timed out! Uninstalling DMX driver...\n");
    dmx_driver_delete(dmx_num);
    while (true) yield(); // halts program without crashing
  }
}
