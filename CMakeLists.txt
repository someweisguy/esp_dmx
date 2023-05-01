idf_component_register(
  SRCS "src/esp_dmx.c" "src/rdm/utils.c" 
       "src/rdm/controller.c"
       "src/rdm/parameters.c" "src/rdm/responder.c" "src/rdm/read_write.c"
       "src/dmx/hal.c"
  INCLUDE_DIRS "src" 
  REQUIRES driver esp_timer esp_common esp_hw_support
)   