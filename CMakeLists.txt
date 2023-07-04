idf_component_register(
  SRCS "src/esp_dmx.c" "src/dmx/hal.c" "src/dmx/agent.c" "src/dmx/sniffer.c"
       "src/rdm/controller.c" "src/rdm/responder.c" "src/rdm/pids.c" 
       "src/rdm/utils.c"
  INCLUDE_DIRS "src" 
  REQUIRES driver esp_timer esp_common esp_hw_support nvs_flash
)   