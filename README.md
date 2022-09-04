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
- [RDM Tools](#rdm-tools)
- [Error Handling](#error-handling)
  - [Packet Errors](#packet-errors)
  - [DMX Start Codes](#dmx-start-codes)
- [Additional Considerations](#additional-considerations)
  - [Wiring an RS-485 Circuit](#wiring-an-rs-485-circuit)
  - [Hardware Specifications](#hardware-specifications)
  - [Remote Device Management](#remote-device-management)
- [API Reference](#api-reference)
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
const bool use_timer = true;
dmx_driver_install(dmx_num, use_timer, DMX_DEFAULT_INTR_FLAGS);
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

Each DMX packet begins with a high-to-low transition called the break, followed by a low-to-high transition called the mark-after-break, followed by an eight-bit byte. This first byte is called the start code. The start-of-packet break, mark-after-break, and start code is called the reset sequence. After the reset sequence, a packet of up to 512 data bytes may be sent.

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

- The DMX port to use.
- Whether to use the ESP32 hardware timer with the DMX driver.
- Flags to allocate interrupts. The macro `DMX_DEFAULT_INTR_FLAGS` can be used to allocate the interrupts using the default interrupt flags.

```c
const bool use_timer = true;
dmx_driver_install(DMX_NUM_2, use_timer, DMX_DEFAULT_INTR_FLAGS);
```

Hardware timers are used for several purposes including the generation of the DMX reset sequence. Users may opt to use busy-waits instead of the hardware timer. If this is desired, the DMX driver will still be able to send and receive proper DMX but some RDM functions may take longer than normal.

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
// Wait for a packet. Returns the size of the received packet or 0 on timeout.
int packet_size = dmx_receive(DMX_NUM_2, &event, DMX_TIMEOUT_TICK);
```

The function `dmx_receive()` takes three arguments. The first argument is the `dmx_port_t` which identifies which DMX port to use. The second argument is a pointer to a `dmx_event_t` struct. Data about the received packet is copied into the `dmx_event_t` struct when a packet is received. This data includes:

- `err` reports any errors that occurred while receiving the packet (see: [Error Handling](#error-handling)).
- `sc` is the DMX start code of the packet.
- `size` is the size of the packet in bytes, including the DMX start code.
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

There are two variations to the `dmx_read()` function. The function `dmx_read_offset()` is similar to `dmx_read()` but allows a small footprint of the entire DMX packet to be read.

```c
const int size = 12;   // The size of this device's DMX footprint.
const int offset = 5;  // The start address of this device.
uint8_t data[size];

// Read slots 5 through 17. Returns the number of slots that were read.
int slots_read = dmx_read_offset(DMX_NUM_2, offset, data, size);
```

Lastly, `dmx_read_slot()` can be used to read a single slot of DMX data.

```c
const int slot_num = 0;  // The slot to read. Slot 0 is the DMX start code!

// Read slot 0. Returns the value of the desired slot or -1 on error.
int value = dmx_read_slot(DMX_NUM_2, slot_num);
```

### DMX Sniffer

This library offers an option to measure DMX break and mark-after-break timings of received data packets. This tool is much more resource intensive than the default DMX driver, so it must be explicitly enabled by calling `dmx_sniffer_enable()`.

The DMX sniffer installs an edge-triggered interrupt on the specified GPIO pin. This library uses the ESP-IDF provided GPIO ISR which allows the use of individual interrupt handlers for specific GPIO interrupts. The interrupt handler works by iterating through each GPIO to determine if it triggered an interrupt and if so, it calls the appropriate handler.

A quirk of the default ESP-IDF GPIO ISR is that lower GPIO numbers are processed earlier than higher GPIO numbers. It is recommended that the DMX RX pin be shorted to a lower GPIO number in order to ensure that the DMX sniffer can run with low latency.

It is important to note that the sniffer requires a fast clock speed in order to maintain low latency. In order to guarantee accuracy of the sniffer, the ESP32 must be set to a CPU clock speed of at least 160MHz. This setting can be configured in `sdkconfig` if the ESP-IDF is used.

Before enabling the sniffer tool, `gpio_install_isr_service()` must be called with the required DMX sniffer interrupt flags. The macro `DMX_SNIFFER_INTR_FLAGS` can be used to provide the proper interrupt flags.

```c
gpio_install_isr_service(DMX_SNIFFER_INTR_FLAGS);

const int sniffer_pin = 4; // Lowest exposed pin on the Feather breakout board.
dmx_sniffer_enable(DMX_NUM_2, sniffer_pin);
```

Break and mark-after-break timings are reported to the DMX sniffer when it is enabled. To read data from the DMX sniffer call `dmx_sniffer_get_data()`. This will wait until the sniffer receives a packet and copy the sniffer data so that it may be processed by the user. If data is copied, this function will return `true`.

```c
dmx_sniffer_data_t sniffer_data;
if (dmx_sniffer_get_data(DMX_NUM_2, &sniffer_data, DMX_TIMEOUT_TICK)) {
  printf("The DMX break length was: %i\n", sniffer_data.break_len);
  printf("The DMX mark-after-break length was: %i\n", sniffer_data.mab_len);
}
```

### Writing

To write to the DMX bus, `dmx_write()` can be called. This writes data to the DMX driver but it does not transmit a packet onto the bus. In order to transmit the data that was written, `dmx_send()` must be called.

```c
uint8_t data[DMX_PACKET_SIZE] = { 0, 1, 2, 3 };

// Write the packet and send it out on the DMX bus.
const int num_bytes_to_send = DMX_PACKET_SIZE;
dmx_write_packet(DMX_NUM_2, data, num_bytes_to_send);
dmx_send_packet(DMX_NUM_2);
```

The size of the packet that is sent when calling `dmx_send()` is equal to either the size of the last call to `dmx_write()` or the slot number used in the last call to `dmx_write_slot()` - whichever is higher.

It takes a typical DMX packet approximately 22 milliseconds to send. During this time, it is possible to write new data to the DMX driver with `dmx_write()` if non-RDM data is being sent. To do so would result in an asynchronous write which may not be desired. To write data synchronously it is required to wait until the DMX packet is finished being sent. The function `dmx_wait_sent()` is used for this purpose.

```c
uint8_t data[DMX_PACKET_SIZE] = { 0, 1, 2, 3 };

while (true) {
  // Send the DMX packet.
  dmx_send(DMX_NUM_2);

  // Process the next DMX packet (while the previous is being sent) here...
  // For example, increment the value of each slot excluding the start code.
  for (int i = 1; i < DMX_PACKET_SIZE; ++i) {
    ++data[i];
  }

  // Wait until the packet is finished being sent before proceeding.
  dmx_wait_sent(DMX_NUM_2, DMX_TIMEOUT_TICK);

  // Now write the packet synchronously!
  dmx_write(DMX_NUM_2, data, DMX_PACKET_SIZE);
}
```

Individual DMX slots can be written using `dmx_write_slot()` similarly to reading individual slots with `dmx_read_slot()`.

```c
// Set slot number 5 to value 127.
const int slot_num = 5;
const uint8_t value = 127;
dmx_write_slot(DMX_NUM_2, slot_num, value);

// Don't forget to call dmx_send()!
```

## RDM Tools

Using only the functions listed above it is possible to send and receive RDM packets. When an RDM packet is written using `dmx_write()` the DMX driver will respond accordingly and ensure that RDM timing requirements are met. For example, calls to `dmx_send()` typically send a DMX break and mark-after-break when sending a DMX packet with a null start code. When sending an RDM discovery response packet the DMX driver automatically removes the DMX break and mark-after-break which is required per the RDM standard. Sending RDM responses with `dmx_send()` may also fail when the DMX driver has detected that the RDM response timeout has already elapsed. This is done to reduce the number of data collisions on the RDM bus and keeps the RDM bus operating properly.

Likewise, the function `dmx_receive()` behaves contextually when receiving DMX or RDM packets. When receiving DMX, calls to `dmx_receive()` may timeout according to the timeout value provided, such as `DMX_TIMEOUT_TICK`. When receiving RDM packets, the DMX driver may timeout much more quickly than the provided timeout value as the timing requirements for receiving RDM packets are much smaller. This feature of the DMX driver is only enabled when the DMX driver is configured to use a hardware timer.

// TODO

## Error Handling

### Packet Errors

On rare occasions, DMX packets can become corrupted. Errors are typically detected upon initially connecting to an active DMX bus but are resolved on receiving the next packet. Errors can be checked by reading the error code from the `dmx_event_t` structure. The error types are as follows:

- `DMX_OK` indicates data was read successfully.
- `DMX_ERR_IMPROPERLY_FRAMED_SLOT` occurs when the DMX driver detects missing stop bits. If a missing stop bit is detected the driver shall discard the improperly framed slot data and all following slots in the packet.
- `DMX_ERR_DATA_COLLISION` occurs when a data collision is detected. This typically occurs during RDM discovery.
- `DMX_ERR_HARDWARE_OVERFLOW` occurs when the ESP32 hardware overflows resulting in loss of data.

In most errors the `dmx_event_t` size can be read to determine at which slot the error occurred.

```c
uint8_t data[DMX_PACKET_SIZE];

dmx_event_t event;
while (true) {
  if (dmx_receive(DMX_NUM_2, &event, DMX_TIMEOUT_TICK)) {
    switch (event.err) {
      case DMX_OK:
        printf("Received packet with start code: %02X and size: %i.\n",
          event.sc, event.size);
        // Data is OK. Now read the packet into the buffer.
        dmx_read_packet(DMX_NUM_2, data, event.size);
        break;

      case DMX_ERR_IMPROPER_SLOT:
        printf("Received malformed byte at slot %i.\n", event.size);
        // A slot in the packet is malformed. Data can be recovered up until 
        //  event.size
        break;

      case DMX_ERR_DATA_COLLISION:
        printf("A data collision was detected.\n");
        // A data collision was detected. This typically happens using RDM.
        break;

      case DMX_ERR_DATA_OVERFLOW:
        printf("Data could not be processed in time.\n");
        // The ESP32 UART overflowed. This could occur if the DMX ISR is being
        //  constantly preempted.
        break;
    }
  } else {
    printf("Lost DMX signal\n");
    // A packet hasn't been received in DMX_TIMEOUT_TICK ticks.
    // Handle packet timeout here...
  }
}
```

It should be noted that this library does not automatically check for DMX timing errors. This library does provide macros to assist with timing error checking, but it is left to the user to implement such measures. DMX and RDM each have their own timing requirements so macros for checking DMX and RDM are both provided. The following macros can be used to assist with timing error checking.

- `DMX_BAUD_RATE_IS_VALID()` evaluates to true if the baud rate is valid for DMX.
- `DMX_BREAK_LEN_IS_VALID()` evaluates to true if the DMX break duration is valid.
- `DMX_MAB_LEN_IS_VALID()` evaluates to true if the DMX mark-after-break duration is valid.
- `RDM_BAUD_RATE_IS_VALID()` evaluates to true if the baud rate is valid for RDM.
- `RDM_BREAK_LEN_IS_VALID()` evaluates to true if the RDM break duration is valid.
- `RDM_MAB_LEN_IS_VALID()` evaluates to true if the RDM mark-after-break duration is valid.

DMX and RDM specify different timing requirements for receivers and transmitters. This library attempts to simplify error checking by combining timing requirements for receiving and transmitting. Therefore there are only the above six timing error checking macros instead of six macros each for receiving and transmitting.

### DMX Start Codes

This library offers the following macro constants for use as DMX start codes. More information about each start code can be found in the DMX standards document or in `dmx_constants.h`.

- `DMX_SC` is the standard DMX null start code.
- `RDM_SC` is the standard Remote Device Management start code.
- `DMX_TEXT_SC` is the ASCII text start code.
- `DMX_TEST_SC` is the test packet start code.
- `DMX_UTF8_SC` is the UTF-8 text packet start code.
- `DMX_ORG_ID_SC` is the organization/manufacturer ID start code.
- `DMX_SIP_SC` is the System Information Packet start code.

Additional macros constants include the following:

- `RDM_SUB_SC` is the sub-start code for Remote Device Management. It is the first byte received after the RDM start code.
- `RDM_PREAMBLE` is not considered a start code but is often the first byte received in an RDM discovery response packet.
- `RDM_DELIMITER` is not considered a start code but is the delimiter byte received at the end of an RDM discovery response preamble.

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

## API Reference

// TODO

## To Do

For a list of planned features, see the [esp_dmx GitHub Projects](https://github.com/users/someweisguy/projects/5) page.
