idf_component_register(
  SRCS "src/dmx/hal/uart.c" "src/dmx/hal/timer.c" "src/dmx/hal/nvs.c"
       "src/dmx/hal/gpio.c" "src/dmx/driver.c" "src/dmx/bus_ctl.c" 
       "src/dmx/device.c" "src/dmx/sniffer.c" "src/rdm/utils.c" 
       
       "src/rdm/controller.c" "src/rdm/responder.c" "src/rdm/parameters.c"
       "src/rdm/uid.c" "src/rdm/pd.c" "src/rdm/bus_ctl.c"
  INCLUDE_DIRS "src" 
  REQUIRES driver esp_timer esp_common esp_hw_support nvs_flash
)   