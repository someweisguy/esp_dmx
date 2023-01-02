idf_component_register(
  SRCS "src/esp_dmx.c" "src/esp_rdm.c" "src/private/rdm_encode/functions.c"
  INCLUDE_DIRS "src" 
  REQUIRES driver esp_timer esp_common esp_hw_support
)   