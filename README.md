# esp_dmx

This library is still a work-in-progress! When it's ready to go, I'll remove this note.

This is a C library to handle sending and receiving ANSI-ESTA E1.11 DMX-512A using an Espressif ESP32. It differs from other existing DMX libraries in that it allows some control over the timing of the DMX frame that is transmitted. Furthermore, it monitors the DMX it receives and can alert the user if the data it is receiving is not within the DMX specification.

For more information on DMX, including timing and physical layer diagrams, see the [ANSI-ESTA DMX standards document](https://tsp.esta.org/tsp/documents/docs/ANSI-ESTA_E1-11_2008R2018.pdf).

## Installation

---

Clone this repo into your project's ```components``` folder. That's it!

## Usage

---

This library was written to look as similar to the default ESP-IDF UART implementation as possible. To get started, call the following code in your ```main.c``` file:

```C
#include "esp_dmx.h"

void app_main(void) {
    // first, setup your input/output pins
    const dmx_port_t dmx_num = 2;
    dmx_set_pin(dmx_num, TX_GPIO_NUM, RX_GPIO_NUM);

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
    dmx_driver_install(dmx_num, DMX_FRAME_SIZE_MAX, DMX_FRAME_SIZE_MAX, 10, 
        &dmx_queue, DMX_INTR_ALLOC_FLAGS);
}
```

## Reading and Writing

---

TODO: More info to come!

## To Do

---

- Enable C++ compilation/linking.
- Remote Device Management.
- DMX rx timing analysis for _breaks_ and _mark after breaks_.
- Art-Net?
