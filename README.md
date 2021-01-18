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
    dmx_driver_install(dmx_num, 513, 10, &dmx_queue, ESP_INTR_FLAG_LEVEL3);
}
```

## Reading and Writing

TODO: More info to come!

## Error Handling

DMX is an old protocol. It isn't clear why it has such restrictive timing requirements, but it could be a result of allowing backwards compatibility with older, slower equipment. The ESP32 is a fast chip - it can process DMX even if the data stream is faster than the standard allows. Therefore, this library specifies two error states: timing errors and data errors. A timing error occurs if the received data stream is either too fast or too slow. Timing errors are reported in the driver queue, but the data is not necessarily corrupted; data read from the frame buffer may still be accurate. Data errors, on the other hand, will always be corrupted. Data errors occur when the received data is malformed, or the rx UART is not read quickly enough and overflows. When a timing or data error occurs, it is wise to call ```dmx_get_valid_frame_len()``` to verify where in the frame the error occurred.

TODO: Example code coming soon...

## To Do

- Use RTS pin when tx'ing
- ESP32 uart RS485 mode
- DMX rx timing analysis
- Enable C++ compilation/linking.
- Remote Device Management.
- Art-Net?
