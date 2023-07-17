idf_component_register(
  SRCS "src/esp_dmx.c" "src/dmx/uart.c" "src/dmx/timer.c" "src/dmx/hal.c"
       "src/rdm/controller.c" "src/rdm/responder.c"
       "src/rdm/parameters.c" "src/rdm/utils.c"
  INCLUDE_DIRS "src" 
  REQUIRES driver esp_timer esp_common esp_hw_support nvs_flash
)   