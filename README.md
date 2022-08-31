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
  - [Setting Communication Pins](#setting-communication-pins)
  - [Installing the Driver](#installing-the-driver)
  - [Parameter Configuration](#parameter-configuration)
- [Reading and Writing](#reading-and-writing)
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

```c
const dmx_port_t dmx_num = DMX_NUM_2;

// First set the communication pins...
const int tx_pin = 17;
const int rx_pin = 16;
const int rts_pin = 21;
dmx_set_pin(dmx_num, tx_pin, rx_pin, rts_pin);

// ...and then install the driver!
dmx_driver_install(dmx_num, DMX_DEFAULT_CONFIG);
```

To write data to the DMX bus, two functions are provided. The function `dmx_write()` writes data to the DMX buffer and `dmx_send()` sends the data out onto the bus. The function `dmx_wait_sent()` is used to block the task until the DMX bus is idle.

```c
uint8_t data[DMX_PACKET_SIZE] = {0};

while (true) {
  // Write to the packet and send it.
  dmx_write(dmx_num, data, DMX_PACKET_SIZE);
  dmx_send(dmx_num, DMX_PACKET_SIZE);
  
  // Do work here...

  // Block until the packet is finished sending.
  dmx_wait_sent(dmx_num, DMX_TIMEOUT_TICK);
}
```

To read from the DMX bus, two additional functions are provided. The function `dmx_receive()` waits until a new packet has been received. The function `dmx_read()` reads the data from the driver buffer into an array so that it can be processed.

```c
dmx_event_t event;  // Read more about the dmx_event_t in Reading and Writing
while (true) {
  const int size = dmx_receive(dmx_num, &event, DMX_TIMEOUT_TICK);
  if (size > 0) {
    dmx_read(dmx_num, data, size);
    // Process data here...
  }

  // Do other work here...

}
```

That's it! For more detailed information on how this library works including details on RDM tools, keep reading.

## What is DMX?

DMX is a unidirectional communication protocol used primarily in the entertainment industry to control lighting and stage equipment. DMX is transmitted as a continuous stream of packets using half-duplex RS-485 signalling with a standard UART port. DMX devices are typically connected using XLR5 in a daisy-chain configuration but other connectors such as XLR3 are common in consumer products.

Each DMX packet begins with a high-to-low transition called the break, followed by a low-to-high transition called the mark after break, followed by an eight-bit byte. This first byte is called the start code. The start-of-packet break, mark after break, and start code is called the reset sequence. After the reset sequence, a packet of up to 512 data bytes may be sent.

DMX imposes very strict timing requirements to allow for backwards compatibility with older lighting equipment. Frame rates may range from 1fps to up to approximately 830fps. A typical DMX controller transmits packets between approximately 25fps to 44fps. DMX receivers and transmitters have different timing requirements which must be adhered to carefully to ensure commands are processed.

Today, DMX often struggles to keep up with the demands of the latest hardware. Its low data rate and small packet size sees it losing market popularity over more capable protocols. However its simplicity and robustness often makes it the first choice for small scale projects.

For in-depth information on DMX, see the [E1.11 standards document](https://tsp.esta.org/tsp/documents/docs/ANSI-ESTA_E1-11_2008R2018.pdf).

## Configuring the DMX Port

The DMX driver’s functions identify each of the UART controllers using `dmx_port_t`. This identification is needed for all the following function calls.

### Setting Communication Pins

Configure the physical GPIO pins to which the DMX port will be connected. To do this, call the function `dmx_set_pin()` and specify which GPIO should be connected to the TX, RX, and RTS signals. If you want to keep a currently allocated pin to a specific signal, pass the macro `DMX_PIN_NO_CHANGE`. This macro should also be used if a pin isn't used.

```c
// Set TX: GPIO16 (port 2 default), RX: GPIO17 (port 2 default), RTS: GPIO21.
dmx_set_pin(DMX_NUM_2, DMX_PIN_NO_CHANGE, DMX_PIN_NO_CHANGE, 21);
```

### Installing the Driver

After the communication pins are set, install the driver by calling `dmx_driver_install()`. This function will allocate the necessary resources for the DMX driver. It instantiates the driver to default DMX settings. The following parameters are passed to this function:

- The hardware timer group to use.
- The hardware timer number to use.
- Flags to allocate interrupts.

```c
const int timer_group = 0;
const int timer_num = 0;
const int interrupt_flags = ESP_INTR_FLAG_IRAM;  // Place interrupt in IRAM.
dmx_driver_install(DMX_NUM_2, timer_group, timer_num, interrupt_flags);
```

Optionally, the macro `DMX_DEFAULT_CONFIG` may be used to simplify installation of the DMX driver.

```c
dmx_driver_install(DMX_NUM_2, DMX_DEFAULT_CONFIG);
```

Hardware timers are used for several purposes including the generation of the DMX reset sequence. It is not possible for individual DMX drivers to share hardware timers. In cases where multiple DMX ports are in use, each DMX port must use different hardware timers.

### Parameter Configuration

In most situations it is not necessary to adjust the default parameters of the DMX driver. Nonetheless, this library allows for individual configuration of the DMX baud rate, break, and mark-after-break. After the DMX driver has been installed, the following functions may be called.

```c
dmx_set_baud_rate(DMX_NUM_2, DMX_BAUD_RATE);     // Set DMX baud rate.
dmx_set_break_len(DMX_NUM_2, DMX_BREAK_LEN_US);  // Set DMX break length.
dmx_set_mab_len(DMX_NUM_2, DMX_MAB_LEN_US);      // Set DMX MAB length.
```

If parameters values that are not within the DMX specification are passed to these functions, the values will be clamped so that they are within DMX specification. Note that it is possible to set driver parameters to be within DMX specification but not within RDM specification. Care must be used when using these functions to ensure that RDM capabilities are maintained.

The above functions each have `_get_` counterparts to retrieve the currently set DMX parameters.

## Reading and Writing

DMX is a unidirectional protocol. This means that on the DMX bus only one device can transmit commands and many devices (typically up to 32) listen for commands. Therefore, this library permits either reading or writing to the bus but not both at once. If transmitting and receiving data simultaneously is desired, the user can install two drivers on two UART ports.

### Reading

Reading may be performed synchronously or asynchronously from the DMX bus. It is typically desired to perform reads synchronously. This means that reads are only performed when a new DMX packet is received. This is ideal because it is not commonly desired to perform reads on the same data multiple times.

To read synchronously from the DMX bus the DMX driver must wait for a new packet. The blocking function `dmx_receive()` can be used for this purpose.

```c
dmx_event_t event;
// Wait for a new packet. Returns the size of the received packet or 0 on error.
size_t packet_size = dmx_receive(DMX_NUM_2, &event, DMX_TIMEOUT_TICK);
```

The function `dmx_receive()` takes three arguments. The first argument is the `dmx_port_t` which identifies which DMX port to use. The second argument is a pointer to a `dmx_event_t` struct. Data about the received packet is copied into the `dmx_event_t` struct when a packet is received. This data includes:

- `err` reports any errors that occurred while receiving the packet (see: [Error Handling](#error-handling)).
- `sc` is the DMX start-code of the packet.
- `size` is the size of the packet in bytes, including the DMX start-code.
- `is_rdm` evaluates to true if the packet is an RDM packet.

The `dmx_event_t` struct also contains detailed information about received RDM packets. If `is_rdm` is true, RDM information can be read from the `dmx_event_t` struct. More information about parsing RDM data can be found in //TODO add link.

Using the `dmx_event_t` struct is optional. If processing DMX or RDM packet data is not desired, users can pass `NULL` in place of a pointer to a `dmx_event_t` struct.

The final argument to `dmx_receive()` is the amount of FreeRTOS ticks to block until the function times out. This library defines a constant, `DMX_TIMEOUT_TICK`, which is the length of time that must be waited until the DMX signal is considered lost according to DMX specification. According to DMX specification this constant is equivalent to 1250 milliseconds.

After a packet is received, `dmx_read()` can be called to read the packet into a user buffer. It is recommended to check for DMX errors before reading data but it is not required.

```c
uint8_t data[DMX_PACKET_SIZE];

dmx_event_t event;
if (dmx_receive(DMX_NUM_2, &event, DMX_TIMEOUT_TICK)) {
  // Check that no errors occurred.
  if (event.err == DMX_OK) {
    dmx_read(DMX_NUM_2, data, event.size);
  } else {
    printf("An error occurred receiving DMX!");
  }
} else {
  printf("Timed out waiting for DMX.");
}
```

// TODO: documentation for `dmx_read_slot()`.

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
if (xQueueReceive(queue, &event, DMX_RX_PACKET_TOUT_TICK) == pdTRUE) {
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
    dmx_wait_send_done(DMX_NUM_2, DMX_TX_PACKET_TOUT_TICK);
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
  if (xQueueReceive(queue, &event, DMX_RX_PACKET_TOUT_TICK)) {
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
    // haven't received a packet in DMX_RX_PACKET_TOUT_TICK ticks
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

For a list of planned features, see the [esp_dmx GitHub Projects](https://github.com/users/someweisguy/projects/5) page.
