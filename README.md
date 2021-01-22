# esp_dmx

This library is still a work-in-progress! When it's ready to go, I'll remove this note.

This is a C library to handle sending and receiving ANSI-ESTA E1.11 DMX-512A using an Espressif ESP32. It differs from other existing DMX libraries in that it allows some control over the timing of the DMX frame that is transmitted. Furthermore, it monitors the DMX it receives and can alert the user if the data it is receiving is not within the DMX specification.

For more information on DMX, including timing and physical layer diagrams, see the [ANSI-ESTA DMX standards document](https://tsp.esta.org/tsp/documents/docs/ANSI-ESTA_E1-11_2008R2018.pdf). For a quick overview of the DMX standard, keep reading.

## Background

DMX is a serial, unidirectional, and differential communication protocol used primarily in the entertainment industry to control lighting and stage equipment. DMX is transmitted as a continuous stream of packets of up to 513 bytes long. The packet begins with a break (a zero, or "off"), followed by a mark after break (a one, or "on"), and then is followed by the packet. Each byte in the packet consists of a start bit, eight bits of data, and two stop bits for a total of 11 bits per byte or "slot." Each frame of DMX must contain a break, mark after break, and packet.

DMX can be transmitted from 1 frame per second up to ~44 frames per second. While it is technically possible to reduce a packet's length to allow for framerates higher than 44fps, it is not considered "to spec" to do so, and therefore it cannot be guaranteed that receiving devices will process commands properly. Framerates slower than 1fps are similarly not allowed.

DMX was originally invented in 1986 when hardware limitations were much more restrictive than they are today. While DMX has been revised slightly to keep up to date with equipment capabilities, it is still an extremely simple protocol and often struggles to keep up with the demands of the latest technology. However its simplicity and robustness often makes it the first choice for small scale projects.

## Installation

Clone this repo into your project's ```components``` folder. That's it!

## Usage

This library was written to look as similar to the default ESP-IDF UART implementation as possible. To get started, call the following code in your ```main.c``` file:

```C
#include "esp_dmx.h"

#define TX_GPIO_NUM  17
#define RX_GPIO_NUM  16
#define RTS_GPIO_NUM 21

void app_main(void) {
    // first, setup your input/output pins
    const dmx_port_t dmx_num = 2;
    dmx_set_pin(dmx_num, TX_GPIO_NUM, RX_GPIO_NUM, RTS_GPIO_NUM);

    // then configure the DMX timing how you like it...
    dmx_config_t config = {
        .baudrate = 250000,
        .break_num = 44,
        .idle_num = 3,
        .source_clk = 0
    };

    // ...or use the standard DMX timing spec
    config = DMX_DEFAULT_CONFIG;

    // now configure the UART
    dmx_param_config(dmx_num, &config);

    // and install the driver!
    QueueHandle_t dmx_queue;
    dmx_driver_install(dmx_num, DMX_MAX_PACKET_SIZE, 10, &dmx_queue, 
      ESP_INTR_FLAG_LEVEL3);
}
```

## Reading and Writing

TODO: More info to come!

## Error Handling

DMX has such restrictive timing requirements due in part to allow for backwards compatibility for older, slower lighting equipment. Because the ESP32 is faster than most lighting equipment in use today, it often can process DMX commands even if the received bytestream is faster than the DMX standard allows. A key design concept of this library is to allow for analysis of the incoming data to determine if it is within the DMX specification.

Data can be checked for validity using the FreeRTOS queue created in ```dmx_driver_install()```.

```C
static dmx_port_t dmx_num = 2;
static uint8_t data[196];

while (1) {
  dmx_event_t event;
  if (xQueueReceive(queue, &event, DMX_RX_PACKET_TOUT_TICK)) {
    if (event.type == DMX_OK) {
      // data is ok - read the frame to the user buffer
      dmx_read_frame(dmx_num, data, event.size);
    } else {
      switch (event.type) {
        case DMX_ERR_BRK_TO_BRK:
          printf("Break-to-break time is invalid\n");
          // packet was not sent to spec, but data may still be recoverable!
          if (event.size == sizeof(data)) {
            dmx_read_frame(dmx_num, data, event.size);
          }
          break;

        case DMX_ERR_IMPROPER_SLOT:
          printf("Received malformed byte at slot %i\n", event.size);
          // the slot at 'event.size' is malformed - possibly a glitch due to
          //  the physical XLR line, but certainly warrants more investigation
          // data can be recovered up until event.size
          dmx_read_frame(dmx_num, data, event.size);
          break;

        case DMX_ERR_PACKET_SIZE:
          printf("Packet size %i is invalid\n", event.size);
          // the host DMX device is sending a bigger packet than it should
          // data may be recoverable, but something is likely wrong with the
          //  host DMX device
          break;

        case DMX_ERR_BUFFER_SIZE:
          printf("User DMX buffer is too small - received %i slots\n", event.size);
          // whoops - our buffer isn't big enough
          // this condition will never occur if buffer_size == DMX_MAX_PACKET_SIZE
          break;

        case DMX_ERR_DATA_OVERFLOW:
          printf("Data could not be processed in time\n");
          // the UART FIFO overflowed
          // this could occur if the enabled interrupt mask is misconfigured
          //  or if the DMX interrupt service routine doesn't run enough
          break;
      }
    }
  } else {
    printf("Lost DMX signal\n");
  }
}
```

In error conditions, the ```dmx_event_t``` structure can be used to learn more information on what went wrong. Using ```event.type``` reveals the type of error. In data corruption events, ```event.size``` can be used to determine where the error occurred. If there is no error or if a DMX timing error or occurs, ```event.size``` can be used to determine the size of the received packet. 

To determine whether an event is a data corruption or invalid timing event, the macros ```DMX_ERR_CORRUPT_DATA``` and ```DMX_ERR_NOT_TO_SPEC``` are provided.

## To Do

- Bitwise error reporting
- detailed DMX rx timing analysis
- Enable C++ compilation/linking.
- Reset-sequence-first mode: allow for sending of DMX reset sequence first
- Remote Device Management.
- Art-Net?
