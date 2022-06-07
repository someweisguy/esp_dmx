# esp_dmx

This library allows for transmitting and receiving ANSI-ESTA E1.11 DMX-512A using an Espressif ESP32. It provides control and analysis of the packet configuration and allows the user to read or write synchronously or asynchronously from the DMX bus using whichever hardware UART port that is desired. This library also includes tools for data error-checking to safely process DMX commands as well as DMX packet metadata extraction to assist with troubleshooting DMX errors.

## Contents

- [Library Installation](#library-installation)
  - [Arduino](#arduino)
  - [ESP-IDF](#esp-idf)
  - [PlatformIO](#platformio)
- [Quick-Start Guide](#quick-start-guide)
- [What is DMX?](#what-is-dmx)
- [Configuring the DMX Port](#configuring-the-dmx-port)
  - [Parameter Configuration](#parameter-configuration)
  - [Setting Communication Pins](#setting-communication-pins)
  - [Installing the Driver](#installing-the-driver)
- [Reading and Writing](#reading-and-writing)
  - [Device Mode](#device-mode)
  - [Reading](#reading)
  - [DMX Sniffer](#dmx-sniffer)
  - [Writing](#writing)
- [Error Handling](#error-handling)
  - [Packet Status](#packet-status)
  - [DMX Start Codes](#dmx-start-codes)
- [Additional Considerations](#additional-considerations)
  - [Wiring an RS-485 Circuit](#wiring-an-rs-485-circuit)
  - [Hardware Specifications](#hardware-specifications)
  - [Remote Device Management](#remote-device-management)
- [To Do](#to-do)

## Library Installation

### Arduino

This library requires the Arduino-ESP32 framework version 2.0.3 or newer. To install the correct framework, follow Espressif's instructions on the Arduino-ESP32 documentation page [here](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).

This library can be installed by cloning this repository into your your `Arduino/libaries` folder or by searching for `esp_dmx` in the Arduino IDE Library Manager. Then simply include the library by adding `#include "esp_dmx.h"` at the top of your Arduino sketch.

### ESP-IDF

This library requires ESP-IDF version 4.4.1 or newer. Clone this repository into your project's `components` folder. The library can be linked by writing `#include "esp_dmx.h"` at the top of your `main.c` file.

### PlatformIO

This library is compatible with the PlatformIO IDE. Search for this library in the PlatformIO library registry and add it to your project. The library can be included by writing `#include "esp_dmx.h"` at the top of your `main.c` or `main.cpp` file.

## Quick-Start Guide

To get started, call the following code in your `setup()` function if using Arduino, or `app_main()` in your `main.c` file if using ESP-IDF.

```cpp
const dmx_port_t dmx_num = DMX_NUM_2;

// first configure the UART...
const dmx_config_t config = DMX_DEFAULT_CONFIG;
dmx_param_config(dmx_num, &config);

// then set the communication pins...
const int tx_io_num = 17, rx_io_num = 16, rts_io_num = 21;
dmx_set_pin(dmx_num, tx_io_num, rx_io_num, rts_io_num);

// and install the driver!
QueueHandle_t dmx_queue;
dmx_driver_install(dmx_num, DMX_MAX_PACKET_SIZE, 10, &dmx_queue, 
      ESP_INTR_FLAG_IRAM);
```

Before the user is able to write to the DMX bus, the driver mode must be set. Call `dmx_set_mode()` and pass either `DMX_MODE_READ` or `DMX_MODE_WRITE`. After the driver is installed `DMX_MODE_READ` is the default.

```cpp
// configure for tx
dmx_set_mode(dmx_num, DMX_MODE_WRITE);
```

To write data to the DMX bus, two functions are provided. The function `dmx_write_packet()` writes data to the DMX buffer and `dmx_send_packet()` sends the data out onto the bus. The function `dmx_wait_send_done()` is used to block the task until the DMX bus is idle.

```cpp
uint8_t data[DMX_MAX_PACKET_SIZE] = {0};
while (1) {
    // write to the packet and send it
    dmx_write_packet(dmx_num, data, DMX_MAX_PACKET_SIZE);
    dmx_send_packet(dmx_num, DMX_MAX_PACKET_SIZE);
    
    // do work here...

    // block until the packet is finished sending
    dmx_wait_send_done(dmx_num, DMX_PACKET_TIMEOUT_TICK);
}
```

To read from the DMX bus, use the queue handle passed to `dmx_driver_install()`. The function `dmx_read_packet()` is provided to read from the driver buffer into an array.

```cpp
dmx_event_t event;
while (1) {
    if (xQueueReceive(dmx_queue, &event, DMX_PACKET_TIMEOUT_TICK)) {
        // read the packet from the driver buffer into 'data'
        dmx_read_packet(dmx_num, data, DMX_MAX_PACKET_SIZE);
    }

    // do other work here...

}
```

That's it! For more detailed information on how this library works, keep reading.

## What is DMX?

DMX is a unidirectional communication protocol used primarily in the entertainment industry to control lighting and stage equipment. DMX is transmitted as a continuous stream of packets using half-duplex RS-485 signalling with a standard UART port. DMX devices are typically connected using XLR5 in a daisy-chain configuration but other connectors such as XLR3 are common in consumer products.

Each DMX packet begins with a high-to-low transition called the break, followed by a low-to-high transition called the mark after break, followed by an eight-bit byte. This first byte is called the start code. The start-of-packet break, mark after break, and start code is called the reset sequence. After the reset sequence, a packet of up to 512 data bytes may be sent.

DMX imposes very strict timing requirements to allow for backwards compatibility with older lighting equipment. Frame rates may range from 1fps to up to approximately 830fps. A typical DMX controller transmits packets between approximately 25fps to 44fps. DMX receivers and transmitters have different timing requirements which must be adhered to carefully to ensure commands are processed.

Today, DMX often struggles to keep up with the demands of the latest hardware. Its low data rate and small packet size sees it losing market popularity over more capable protocols. However its simplicity and robustness often makes it the first choice for small scale projects.

For in-depth information on DMX, see the [E1.11 standards document](https://tsp.esta.org/tsp/documents/docs/ANSI-ESTA_E1-11_2008R2018.pdf).

## Configuring the DMX Port

The DMX driver’s functions identify each of the UART controllers using `dmx_port_t`. This identification is needed for all the following function calls.

### Parameter Configuration

#### Single Step

Call the function `dmx_param_config()` and pass it a `dmx_config_t` structure. It contains all the parameters needed to configure the DMX packet settings. In most situations, custom packet configuration isn't necessary. The macro `DMX_DEFAULT_CONFIG` is provided to simplify this process.

```cpp
const dmx_config_t dmx_config = DMX_DEFAULT_CONFIG;
dmx_param_config(DMX_NUM_2, &dmx_config);
```

If using a custom DMX configuration is desired, the `dmx_config_t` parameters can be set manually.

```cpp
const dmx_config_t dmx_config = {
    .baud_rate = 250000, // typical baud rate   
    .break_num = 45,     // 180us 
    .idle_num = 5        // 20us
};
dmx_param_config(DMX_NUM_2, &dmx_config);
```

The `break_num` corresponds to the duration of the packet break and `idle_num` corresponds to the duration of the mark after break. Both values are set in units of time that it takes to send one bit at the current baud rate. If the current baud rate is 250k, it takes 4μs to send one bit. Setting `break_num` to 45 and `idle_num` to 5 in this example sets the break and mark after break to 180μs and 20μs respectively.

#### Multiple Steps

Parameters may be configured individually by calling the below dedicated functions. These functions are also useful if re-configuring a single parameter.

```cpp
dmx_set_baud_rate(DMX_NUM_2, 250000);
dmx_set_break_num(DMX_NUM_2, 44);
dmx_set_idle_num(DMX_NUM_2, 3);
```

Each of the above functions has a `_get_` counterpart to check the currently set value. For example, to check the current baud rate, call `dmx_get_baud_rate()`.

### Setting Communication Pins

Configure the physical GPIO pins to which the DMX port will be connected. To do this, call the function `dmx_set_pin()` and specify which GPIO should be connected to the TX, RX, and RTS signals. If you want to keep a currently allocated pin to a specific signal, pass the macro `DMX_PIN_NO_CHANGE`. This macro should also be used if a pin isn't used.

```cpp
// set TX: IO16 (port 2 default), RX: IO17 (port 2 default), RTS: IO21
dmx_set_pin(DMX_NUM_2, DMX_PIN_NO_CHANGE, DMX_PIN_NO_CHANGE, 21);
```

### Installing the Driver

After the communication pins are set, install the driver by calling `dmx_driver_install()`. The following parameters are passed to this function:

- Size of the driver double-buffer
- Size of the event queue
- Handle to the queue
- Flags to allocate interrupts

This function will allocate the necessary resources for the DMX driver. Note that the driver uses a double-buffer system. The driver will allocate twice the size of the passed buffer size argument.

```cpp
QueueHandle_t dmx_queue;
const int buffer_size = DMX_MAX_PACKET_SIZE; // 513 bytes
// install DMX driver using an event queue
dmx_driver_install(DMX_NUM_2, buffer_size, 10, &dmx_queue, 0);
```

Once this step is complete, DMX devices can be connected to check for communication.

## Reading and Writing

### Device Mode

DMX is a unidirectional protocol. This means that on the DMX bus only one device can transmit commands and many devices (typically up to 32) listen for commands. Therefore, this library permits either reading or writing to the bus but not both at once.

To set the driver mode call `dmx_set_mode()` and pass to it either `DMX_MODE_READ` or `DMX_MODE_WRITE`. After the driver is installed `DMX_MODE_READ` is the default.

```cpp
// set the DMX driver to transmit mode
dmx_set_mode(DMX_NUM_2, DMX_MODE_WRITE);
// dmx_set_mode(DMX_NUM_2, DMX_MODE_READ); // don't need to read now
```

If transmitting and receiving data simultaneously is desired, the user can install two drivers on two UART ports. It should be noted that this is an unusual use case. This library is not meant to act as a DMX optoisolator or splitter.

### Reading

To read from the DMX bus, the event queue handle passed to `dmx_driver_install()` can be used to determine when a packet has been received. A `dmx_event_t` message will be posted to the event queue. Then the packet can be read from the DMX driver double-buffer into a user buffer using `dmx_read_packet()`.

The macro `DMX_PACKET_TIMEOUT_TICK` can be used to block the task until a packet is received or a DMX timeout occurs.

```cpp
// allocate a buffer that is the max size of a DMX packet
uint8_t data[DMX_MAX_PACKET_SIZE];

dmx_event_t event;
while (1) {
    if (xQueueReceive(dmx_queue, &event, DMX_PACKET_TIMEOUT_TICK) == pdTRUE) {
        // read back the size of the packet into our buffer
        dmx_read_packet(DMX_NUM_2, data, event.size);
    } else {
        // handle packet timeout...
    }
}
```

The `dmx_event_t` structure contains some helpful information about the packet that was received. Some of the information includes:

- Packet errors
- Start code
- Size in bytes
- Duration in microseconds

These values can be used to determine if the received data should be processed or ignored.

```cpp
// if there are no errors and the start code is correct, read the packet
if (event.status == DMX_OK && event.start_code == DMX_SC) {
    dmx_read_packet(DMX_NUM_2, data, event.size);

    printf("Packet took %i microseconds!", event.duration);
}
```

Individual DMX slots can be read using `dmx_read_slot()`. To verify that the DMX slot exists, the size of the packet should be verified.

```cpp
const int slot_idx = 5;
if (event.status == DMX_OK && event.size >= slot_idx) {
  uint8_t slot_data;
  dmx_read_slot(DMX_NUM_2, slot_idx, &slot_data);

  printf("Slot %i == %i", slot_idx, slot_data);
}
```

This library offers tools to perform robust error-checking. For more information on errors, see the [Error Handling](#error-handling) section.

### DMX Sniffer

This library offers an option to measure break and mark after break timings of received data packets. This tool is much more resource intensive than the default DMX receive driver, so it must be explicitly enabled by calling `dmx_sniffer_enable()`.

The DMX sniffer installs an edge-triggered interrupt on the specified GPIO pin. This library uses the ESP-IDF provided GPIO ISR which allows the use of individual interrupt handlers for specific GPIO interrupts. The interrupt handler works by iterating through each GPIO to determine if it triggered an interrupt and if so, it calls the appropriate handler.

A quirk of the default ESP-IDF GPIO ISR is that lower GPIO numbers are processed earlier than higher GPIO numbers. It is recommended that the DMX RX pin be shorted to a lower GPIO number in order to ensure that the DMX sniffer can run with low latency.

It is important to note that the sniffer requires a fast clock speed in order to maintain low latency. In order to guarantee accuracy of the sniffer, the ESP32 must be set to a CPU clock speed of at least 160MHz. This setting can be configured in `sdkconfig` if the ESP-IDF is used.

Before enabling the sniffer tool, `gpio_install_isr_service()` must be called.

```cpp
gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
const int sniffer_io_num = 4; // lowest exposed pin on the Feather breakout board
dmx_sniffer_enable(DMX_NUM_2, sniffer_io_num);
```

Break and mark after break timings are reported to the event queue when the DMX sniffer is enabled. If the sniffer is disabled, either because `dmx_sniffer_disable()` was called or because `dmx_sniffer_enable()` was not called, the reported break and mark after break durations will default to -1.

```cpp
dmx_event_t event;
if (xQueueReceive(queue, &event, DMX_PACKET_TIMEOUT_TICK) == pdTRUE) {
  // read back break and mark after break
  printf("The break was %ius, ", event.timing.brk);
  printf("and the mark after break was %ius.\n", event.timing.mab);
}
```

### Writing

Writing to the DMX bus does not require the use of an event queue. To write to the DMX bus, `dmx_write_packet()` can be called. This writes data to the DMX driver but it does not transmit a packet onto the bus. In order to transmit the data that was written, `dmx_send_packet()` can be called.

```cpp
uint8_t data[DMX_MAX_PACKET_SIZE] = { 0, 1, 2, 3 };

dmx_set_mode(DMX_NUM_2, DMX_MODE_WRITE); // enable tx mode

// write the packet and send it out on the DMX bus
dmx_write_packet(DMX_NUM_2, data, DMX_MAX_PACKET_SIZE);
dmx_send_packet(DMX_NUM_2, DMX_MAX_PACKET_SIZE);
```

Calling `dmx_send_packet()` will fail if the DMX driver is currently transmitting a packet of DMX data. To ensure that packets are continuously sent, `dmx_wait_send_done()` can be used.

```cpp
uint8_t data[DMX_MAX_PACKET_SIZE] = { 0, 1, 2, 3 };

dmx_set_mode(DMX_NUM_2, DMX_MODE_WRITE); // enable tx mode

while (1) {
    // write and send the packet
    const int num_slots = 100;
    dmx_write_packet(DMX_NUM_2, data, num_slots);
    dmx_send_packet(DMX_NUM_2, num_slots);

    // do other work here...

    // block until we are ready to send another packet
    dmx_wait_send_done(DMX_NUM_2, DMX_PACKET_TIMEOUT_TICK);
}
```

The DMX driver will automatically check if the DMX transmission has timed out between sending the last packet and the current packet. If it has, it will simulate a DMX reset sequence in software before sending a new packet. Simulating the reset sequence uses inefficient busy-waiting to recreate a break and mark after break. ESP32 busy-waiting is imprecise at the microsecond resolution that is needed for the reset sequence. If the DMX task is not preempted it is usually precise within 30μs. Because this should only happen after sending the first packet and because 30μs is well within DMX timing requirements, this behavior is acceptable for this library.

Individual DMX slots can be written using `dmx_write_slot()`.

```cpp
// set slot 5 to 127
const int slot_idx = 5;
uint8_t slot_val = 127;
dmx_write_slot(DMX_NUM_2, slot_idx, slot_val);

// don't forget to call dmx_send_packet()!
```

## Error Handling

### Packet Status

On rare occasions, DMX packets can become corrupted. Errors can be checked by reading the status from the `dmx_event_t` structure. The error types are as follows:

- `DMX_OK` occurs when the packet is received successfully.
- `DMX_ERR_IMPROPER_SLOT` occurs when a slot is missing a start or stop bit.
- `DMX_ERR_PACKET_SIZE` occurs when the number of data bytes received exceeds `DMX_MAX_PACKET_SIZE`
- `DMX_ERR_BUFFER_SIZE` occurs when the driver buffer size is smaller than the number of packets received. This error will not occur if the driver buffer size is set to `DMX_MAX_PACKET_SIZE`.
- `DMX_ERR_DATA_OVERFLOW` occurs when the UART hardware is not able to process data quickly enough and it overflows.

In most errors, the event size can be read to determine at which byte the error occurred. In every error condition except for `DMX_ERR_BUFFER_SIZE` the event start code will default to -1.

```cpp
dmx_event_t event;
while (1) {
  if (xQueueReceive(queue, &event, DMX_PACKET_TIMEOUT_TICK)) {
    switch (event.status) {
      case DMX_OK:
        printf("Received packet with start code: %02X and size: %i\n",
          event.start_code, event.size);
        // data is ok - read the packet into our buffer
        dmx_read_packet(DMX_NUM_2, data, event.size);
        break;

      case DMX_ERR_IMPROPER_SLOT:
        printf("Received malformed byte at slot %i\n", event.size);
        // a slot in the packet is malformed - possibly a glitch due to the
        //  XLR connector? will need some more investigation
        // data can be recovered up until event.size
        break;

      case DMX_ERR_PACKET_SIZE:
        printf("Packet size %i is invalid\n", event.size);
        // the host DMX device is sending a bigger packet than it should
        // data may be recoverable but something went very wrong to get here
        break;

      case DMX_ERR_BUFFER_SIZE:
        printf("User DMX buffer is too small - received %i slots\n", 
          event.size);
        // whoops - our buffer isn't big enough
        // this code will not run if buffer size is set to DMX_MAX_PACKET_SIZE
        break;

      case DMX_ERR_DATA_OVERFLOW:
        printf("Data could not be processed in time\n");
        // the UART FIFO overflowed
        // this could occur if the interrupt mask is misconfigured or if the
        //  DMX ISR is constantly preempted
        break;
    }
  } else {
    printf("Lost DMX signal\n");
    // haven't received a packet in DMX_PACKET_TIMEOUT_TICK ticks
    // handle packet timeout...
  }
}
```

It should be noted that this library does not automatically check for DMX timing errors. This library does provide macros to assist with timing error checking, but it is left to the user to implement such measures. The following macros can be used to assist with timing error checking.

- `DMX_RX_PKT_DURATION_IS_VALID()` evaluates to true if the packet duration is valid.
- `DMX_RX_BRK_DURATION_IS_VALID()` evaluates to true if the break duration is valid.
- `DMX_RX_MAB_DURATION_IS_VALID()` evaluates to true if the mark after break duration is valid.

DMX specifies different timing requirements for receivers and transmitters. In situations where it is necessary to check if transmitted timing values are valid, this library provides `_TX_` versions of the above macros.

Finally, the following macros can be used in both transmit and receive scenarios.

- `DMX_BAUD_RATE_IS_VALID()` evaluates to true if the baud rate is valid.
- `DMX_START_CODE_IS_VALID()` evaluates to true if the start code is permitted in the DMX standard.

### DMX Start Codes

This library offers the following macro constants for use as DMX start codes. More information about each start code can be found in the DMX standards document or in `dmx_caps.h`.

- `DMX_SC` is the standard DMX null start code.
- `RDM_SC` is the standard Remote Device Management start code.
- `DMX_TEXT_ASC` is the ASCII text alternate start code.
- `DMX_TEST_ASC` is the test packet alternate start code.
- `DMX_UTF8_ASC` is the UTF-8 text packet alternate start code.
- `DMX_ORG_ID_ASC` is the organization/manufacturer ID alternate start code.
- `DMX_SIP_ASC` is the System Information Packet alternate start code.

Some start codes are considered invalid and should not be used in a DMX packet. The validity of the start code can be checked using the macro `DMX_START_CODE_IS_VALID()`. If the start code is valid, this macro will evaluate to true. This library does not automatically check for valid start codes. Such error checking is left to the user to implement.

## Additional Considerations

### Wiring an RS-485 Circuit

DMX is transmitted over RS-485. RS-485 uses twisted-pair, half-duplex, differential signalling to ensure that data packets can be transmitted over large distances. DMX starts as a UART signal which is then driven using an RS-485 transceiver. Because the ESP32 does not have a built-in RS-485 transceiver, it is required for the ESP32 to be wired to a transceiver in most cases.

RS-485 transceivers typically have four data input pins: `RO`, `DI`, `DE`, and `/RE`. `RO` is receiver output. It is the pin that the UART RX pin is connected to so that data may be read from other devices to the ESP32. `DI` is driver input. It is connected to the UART TX pin so that data may be written to other devices from the ESP32. `DE` is driver input enable. Bringing this pin high enables the input on the `DI` pin. `/RE` is receiver output enable. The overline on this pin name indicates that it is active when driven low, and inactive when driven high. Driving this pin low enables the input on the `DI` pin.

Because `DE` and `/RE` enable writing and reading respectively, and because `DE` is active high and `/RE` is active low, these pins are often shorted together. In this example, these pins are wired together and are controlled with one pin on the ESP32. This pin is called the enable pin. It can also be referred to as the RTS pin. The example schematic can be seen below.

![An example RS-485 circuit](/media/rs485-ckt.png)

In this example circuit, R1 and R3 are 680 ohms each. Many RS-485 breakout boards set these resistor values to 20k ohm or higher. Such high resistance values are acceptable and should still allow DMX to be written and read.

R2, the 120 ohm resistor, is a terminating resistor. It is not required to include this resistor but it can ensure system stability when connecting long lines of DMX consisting of multiple devices. If it is decided not to include this resistor, DMX-A and DMX-B should not be shorted together.

Many RS-485 chips, such as the [Maxim MAX485](https://datasheets.maximintegrated.com/en/ds/MAX1487-MAX491.pdf) are 3.3v tolerant. This means that it can be controlled with the ESP32 without any additional electrical components. Other RS-485 chips may require 5v data to transmit DMX. In this case, it is required to convert the output of the ESP32 to 5v using a logic level converter.

### Hardware Specifications

ANSI-ESTA E1.11 DMX512-A specifies that DMX devices be electrically isolated from other devices on the DMX bus. In the event of a power surge, the likely worse-case scenario would mean the failure of the RS-485 circuitry and not the entire DMX device. Some DMX devices may function without isolation, but using non-isolated equipment is not recommended.

### Remote Device Management

Support for RDM is currently in progress.

## To Do

- Allow user to place ISR in IRAM optionally
- Reset-Sequence-First Mode. Allow for reset sequences to be sent first rather than using the UART hardware break circuitry.
- RDM Support
