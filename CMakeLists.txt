idf_build_get_property(target IDF_TARGET)

idf_component_register(
  SRCS "src/esp_dmx.c" 
  INCLUDE_DIRS "src/include" 
  PRIV_INCLUDE_DIRS "src/driver" "src/${target}"
  REQUIRED_IDF_TARGETS esp32
)   