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

#define TX_GPIO_NUM   17
#define RX_GPIO_NUM   16
#define RTS_GPIO_NUM  21

void app_main(void) {
    // first, setup your input/output pins
    const dmx_port_t dmx_num = 1;
    dmx_set_pin(dmx_num, TX_GPIO_NUM, RX_GPIO_NUM, RTS_GPIO_NUM);

    // then setup the DMX timing how you like it...
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

The functions to configure the UART and install the driver should appear familiar to those who have experience using the UART in the ESP-IDF as they are almost identical to the IDF functions to assign GPIO, configure UART parameters, and install the UART driver.

```dmx_set_pin()``` is functionally the same as ```uart_set_pin()``` except that in this library only three pins are needed to operate a DMX bus: TX, RX, and RTS.

```dmx_param_config()``` configures the UART hardware for use on a DMX bus. The user passes a ```dmx_config_t``` structure as an argument to configure the UART ```baudrate```, ```break_num```, ```idle_num```, and ```source_clk```. In DMX the baudrate can range from 245kBaud to 255kBaud, but typically DMX is transmitted at 250kBaud. The ```break_num``` and ```idle_num``` refer to setting the DMX break and mark-after-break durations. In the ESP32 hardware, these are set in units of the time it takes to send 1 bit at the current baudrate. For example, if the baudrate is set at 250kBaud, then the time it takes to send 1 bit is ```1000000 / 250000 = 4us```. So setting ```break_num``` to ```44``` would result in a ```4 * 44 = 176us``` break time. The user should be aware that in the ESP32 hardware, ```break_num``` is represented as a 8-bit value and ```idle_num``` as a 10-bit value so they are limited to ```255``` decimal and ```1023``` decimal respectively. ```source_clk``` can be used to set the clock source for the UART hardware. A macro ```DMX_DEFAULT_CONFIG``` is included to configure the UART hardware to default DMX settings.

Optionally, there are individual getters and setters for DMX parameter configuration that can be called instead.

```C
const dmx_port_t dmx_num = 1;

// this is the same thing....
dmx_set_baudrate(dmx_num, 250000);
dmx_set_break_num(dmx_num, 44);
dmx_set_idle_num(dmx_num, 3);

// ... as calling this
dmx_config_t dmx_config = DMX_DEFAULT_CONFIG;
dmx_param_config(dmx_num, &dmx_config);
```

```dmx_driver_install()``` is called to install the DMX driver. In doing so, the user passes a buffer size, a FreeRTOS queue, and interrupt allocation flags as arguments. The DMX driver uses a double-buffering method to read and write DMX packets from the UART FIFO. As such, the user can expect that the amount of memory allocated for the driver buffer will be twice the size of the argument that is passed. The FreeRTOS queue is used to allow for synchronous reading and error-checking. If the user intends to use this library solely to transmit DMX, the user can pass ```NULL``` instead.

## Reading and Writing

DMX is a unidirectional protocol which means that on the DMX bus only one device writes commands whereas many devices (typically up to 32) listen for instructions from the host device. Because of this, this library permits either transmitting or receiving data but not both at once. Receive or transmit mode can be set using ```dmx_set_mode()``` and passing either ```DMX_MODE_RX``` to act as a client device or ```DMX_MODE_TX``` to act as a host device. Reading and writing to and from the DMX bus can be done using ```dmx_read_frame()``` and ```dmx_write_frame()``` respectively.

If both transmitting and receiving data is desired, the user can install two drivers - one on UART bus 1 and the other on UART bus 2 - to facilitate receiving and transmission. However, it should be noted that this is an unusual use case. This library is not meant to act as an optoisolator to split or retransmit DMX data.

### Reading from the DMX Bus

After installing the DMX driver with ```dmx_driver_install()``` the driver is automatically configured to receive DMX data. It is therefore not necessary to call ```dmx_set_mode()``` to set the driver to ```DMX_MODE_RX```.

Upon calling ```dmx_driver_install()```, a FreeRTOS queue is passed to the driver and instantiated. Events are posted to this queue whenever a DMX packet is received. Using the queuing system can facilitate synchronous reads from the data bus to ensure that data is not read from the bus before the packet is finished being transmitted. Using the macro ```DMX_RX_PACKET_TOUT_TICK``` can be used to determine a packet timeout if the data packet isn't received quickly enough.

```C
static const size_t BUF_SIZE = DMX_MAX_PACKET_SIZE;

static dmx_port_t dmx_num = 1;
static uint8_t data[BUF_SIZE];

// additional setup happens here...

// install the driver with a queue
QueueHandle_t queue;
dmx_driver_install(dmx_num, BUF_SIZE, 10, &queue, 0);

dmx_event_t event;

while (1) {
  if (xQueueReceive(queue, &event, DMX_RX_PACKET_TOUT_TICK) == pdTRUE) {
    dmx_read_frame(dmx_num, data, BUF_SIZE); // synchronous read
  } else {
    // packet timed out...
  }
}
```

Packet metadata is included in the queue messages and can be read to determine if the data should be processed or ignored. This library offers tools to assist with handling error conditions. See the section on Error Handling for more information.

```C
if (xQueueReceive(queue, &event, DMX_RX_PACKET_TOUT_TICK) == pdTRUE) {
  printf("Packet error code is %i, ", event.type); 
  // 'event.type' is used to determine error conditions such as a buffer
  //  overflow or improperly framed slot

  printf("start code is: %i, " event.start_code);
  // 'event.start_code' is a copy of slot 0 in the packet - typically non-zero
  //  start codes are handled differently than standard packets
  // in most error conditions, .start_code defaults to -1

  printf("and packet took %i microseconds.\n" event.duration);
  // 'event.duration' is simply the time it took between the start of the
  //  packet and the end of the packet
}
```

If synchronous reads aren't necessary or desired, the driver may be installed without passing a queue handle. Installing the driver without a messaging queue can be risky if the received data isn't guaranteed to be error-free.

```C
// install the driver without a queue
dmx_driver_install(dmx_num, BUF_SIZE, 0, NULL, 0);

while (1) {
  dmx_read_frame(dmx_num, data, BUF_SIZE); // asynchronous read
  // do other work here...
}
```

### RX Timing Analysis

This library contains an API to measure timings of the received DMX packet break and mark-after-break. This resource is relatively more CPU intensive than the default DMX receive driver, so it must be explicitly enabled by calling ```dmx_rx_timing_enable()``` after installing the driver.

The timing analysis tool installs an edge-triggered interrupt on the specified GPIO pin to detect and record the timestamps of when the DMX bus goes high or low. Rather than installing one interrupt that triggers on any GPIO, this library opts to use the default GPIO interrupt API, ```gpio_install_isr_service()```, for registering different event handlers on specific GPIO. This interrupt handler works by iterating through each GPIO to determine if an interrupt was triggered by the pin and if so, calls the callback function. A quirk of this ISR is that GPIO whose numbers are lower are checked first for interrupt conditions before higher numbered GPIO (e.g. 0, 1, 2, 3, ... 37, 38, 39). When using the timing analysis tool, it is recommended to short a lower number GPIO to the RX pin to ensure that there is as little latency as possible between when an edge transition occurs and the timing analysis tool has a chance to run.

It is important to note that the timing tool requires the fast clock speed of the ESP32 in order to properly function. In order to guarantee the accuracy of the timing tool, the ESP32 must be set to a CPU clock speed of at least 160MHz in ```sdkconfig```.

Before enabling the timing analysis tool, ```gpio_install_isr_service()``` should be called first.

```C
gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
// gpio #4 is the lowest available on the Adafruit Feather
dmx_rx_timing_enable(dmx_num, 4);
```

When the queue reports data has been receieved, the ```dmx_event_t``` structure can be used to read back additional metadata, ```event.timing.brk``` for the received break duration and ```event.timing.mab``` for the received mark-after-break duration, of the received frame.

```C
dmx_event_t event;
if (xQueueReceive(queue, &event, DMX_RX_PACKET_TOUT_TICK) == pdTRUE) {
  // read back break and mark-after-break
  printf("The received break was %ius, ", event.timing.brk);
  printf("and the mark-after-break was %ius.\n", event.timing.mab);
}
```

When the timing tool is disabled either because ```dmx_rx_timing_disable()``` was called or because it was not enabled in the first place, both ```event.timing.brk``` and ```event.timing.mab``` default to ```-1```.

### Writing to the DMX Bus

TODO: More info to come!

## Error Handling

DMX has restrictive timing requirements due in part to allow for backwards compatibility with older, slower lighting equipment. Because the ESP32 is faster than most lighting equipment in use today, it often can process DMX commands even if the received bytestream is faster than the DMX standard allows. A key design concept of this library is to allow for timing analysis of the incoming data to determine if it is within the DMX specification.

Data can be checked for validity using the FreeRTOS queue created in ```dmx_driver_install()```.

```C
// BUF_SIZE not set to DMX_MAX_PACKET_SIZE for demonstration purposes
static const size_t BUF_SIZE = 196;

static dmx_port_t dmx_num = 2;
static uint8_t data[BUF_SIZE];

// driver setup goes here...

QueueHandle_t queue;
dmx_driver_install(dmx_num, BUF_SIZE, 10, &queue, ESP_INTR_FLAG_LEVEL3);

dmx_event_t event;

while (1) {
  if (xQueueReceive(queue, &event, DMX_RX_PACKET_TOUT_TICK)) {
    switch (event.type) {
      case DMX_OK:
        printf("Received packet with start code: %02X and size: %i\n",
          event.start_code, event.size);
        // data is ok - read the packet into our buffer
        dmx_read_frame(dmx_num, data, event.size);
        break;

      case DMX_ERR_IMPROPER_SLOT:
        printf("Received malformed byte at slot %i\n", event.size);
        // the slot at 'event.size' is malformed - possibly a glitch due to
        //  the physical XLR, but certainly warrants more investigation
        // data can be recovered up until event.size
        break;

      case DMX_ERR_PACKET_SIZE:
        printf("Packet size %i is invalid\n", event.size);
        // the host DMX device is sending a bigger packet than it should
        // data may be recoverable, but something is likely wrong with the
        //  host DMX device
        break;

      case DMX_ERR_BUFFER_SIZE:
        printf("User DMX buffer is too small - received %i slots\n", 
          event.size);
        // whoops - our buffer isn't big enough
        // this condition will not occur if BUF_SIZE == DMX_MAX_PACKET_SIZE
        break;

      case DMX_ERR_DATA_OVERFLOW:
        printf("Data could not be processed in time\n");
        // the UART FIFO overflowed
        // this could occur if the enabled interrupt mask is misconfigured
        //  or if the DMX interrupt service routine doesn't run enough
        break;
    }
  } else {
    printf("Lost DMX signal\n");
    // haven't received a packet in DMX_RX_PACKET_TOUT_TICK ticks
    // handle signal loss here
  }
}
```

In error conditions, the ```dmx_event_t``` structure can be used to learn more information on what went wrong; ```event.type``` can be read to determine the source of the error. Error conditions are only reported if a likely data-corrupting error occurred. DMX timing errors will not be reported automatically to the user, but the user can use the ```dmx_event_t``` structure with the provided macros below to perform timing error-checking if desired.

```C
dmx_event_t event; // we've received an event from the queue

DMX_START_CODE_IS_VALID(event.start_code);
DMX_RX_PKT_DURATION_IS_VALID(event.duration);

// the following macros can be used if rx timing is enabled
// otherwise, they will evaluate to false!
DMX_RX_BRK_DURATION_IS_VALID(event.timing.brk);
DMX_RX_MAB_DURATION_IS_VALID(event.timing.mab);
```

Note that DMX has different timing requirements for transmitters and receivers. Only received parameters should be used with ```DMX_RX_??_DURATION_IS_VALID()``` macros. In situations where transmitted parameters need to be checked, ```DMX_TX_??_DURATION_IS_VALID()``` macros should be used.

```C
// parameters aren't from an event because they are transmitted, not received
int break_len_us = 176;    // is ok; 92us is the minimum
int mab_len_us = 8;        // not ok - should be at least 12us
int packet_len_us = 22756; // is ok even though 'mab_len_us' is too small

DMX_TX_BRK_DURATION_IS_VALID(break_len_us);  // evaluates true
DMX_TX_MAB_DURATION_IS_VALID(mab_len_us);    // evaluates false!
DMX_TX_PKT_DURATION_IS_VALID(packet_len_us); // evaluates true
```

## Additional Considerations

TODO: More info coming soon!

## To Do

- example code
- Testing of the DMX_ERR_IMPROPER_SLOT event type
- Reset-sequence-first mode: allow for sending of DMX reset sequence first
- Remote Device Management.
- Art-Net?
