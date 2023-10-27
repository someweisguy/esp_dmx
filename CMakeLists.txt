idf_component_register(
  SRCS "src/esp_dmx.c" "src/rdm_utils.c" "src/dmx/hal/uart.c"
       "src/dmx/hal/timer.c" "src/dmx/hal/nvs.c" "src/dmx/hal/gpio.c"
       "src/rdm/controller.c" "src/rdm/responder.c" "src/rdm/parameters.c"

       "src/dmx/device.c" "src/dmx/driver.c"
       "src/dmx/io.c" "src/dmx/rw.c" "src/dmx/sniffer.c"
  INCLUDE_DIRS "src" 
  REQUIRES driver esp_timer esp_common esp_hw_support nvs_flash
)   