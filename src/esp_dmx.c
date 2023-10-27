#include "esp_dmx.h"

#include "dmx/struct.h"
#include "dmx/hal/uart.h"
#include "dmx/hal/timer.h"
#include "dmx/hal/gpio.h"
#include "dmx/hal/nvs.h"
#include "endian.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_mac.h"
#endif

const char *TAG = "dmx";  // The log tagline for the library

dmx_port_t rdm_binding_port;
rdm_uid_t rdm_device_uid = {};
dmx_driver_t *dmx_driver[DMX_NUM_MAX] = {};
