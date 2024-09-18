# If the ESP_DMX_ROOT variable is not set, set it to the current directory
if(NOT DEFINED ESP_DMX_ROOT)
  set(ESP_DMX_ROOT ".")
endif()

idf_component_register(
  SRCS 
       # DMX driver HAL
       "${ESP_DMX_ROOT}/src/dmx/hal/uart.c" "${ESP_DMX_ROOT}/src/dmx/hal/timer.c" "${ESP_DMX_ROOT}/src/dmx/hal/nvs.c"
       "${ESP_DMX_ROOT}/src/dmx/hal/gpio.c"
       
       # DMX driver and sniffer
       "${ESP_DMX_ROOT}/src/dmx/service.c" "${ESP_DMX_ROOT}/src/dmx/driver.c"
       "${ESP_DMX_ROOT}/src/dmx/io.c" "${ESP_DMX_ROOT}/src/dmx/device.c" "${ESP_DMX_ROOT}/src/dmx/parameter.c"
       "${ESP_DMX_ROOT}/src/dmx/sniffer.c" 

       # RDM driver
       "${ESP_DMX_ROOT}/src/rdm/driver.c"
       
       # RDM controller
       "${ESP_DMX_ROOT}/src/rdm/controller/discovery.c" "${ESP_DMX_ROOT}/src/rdm/controller/product_info.c"
       "${ESP_DMX_ROOT}/src/rdm/controller/device_control.c" "${ESP_DMX_ROOT}/src/rdm/controller/dmx_setup.c"
       "${ESP_DMX_ROOT}/src/rdm/controller/utils.c"
       
       # RDM responder
       "${ESP_DMX_ROOT}/src/rdm/responder.c" "${ESP_DMX_ROOT}/src/rdm/responder/discovery.c"
       "${ESP_DMX_ROOT}/src/rdm/responder/product_info.c" "${ESP_DMX_ROOT}/src/rdm/responder/rdm_info.c"
       "${ESP_DMX_ROOT}/src/rdm/responder/device_control.c" "${ESP_DMX_ROOT}/src/rdm/responder/queue_status.c"
       "${ESP_DMX_ROOT}/src/rdm/responder/dmx_setup.c" "${ESP_DMX_ROOT}/src/rdm/responder/sensor_parameter.c"
       "${ESP_DMX_ROOT}/src/rdm/responder/power_lamp.c" "${ESP_DMX_ROOT}/src/rdm/responder/utils.c"
  INCLUDE_DIRS "${ESP_DMX_ROOT}/src"
  REQUIRES driver esp_timer esp_common esp_hw_support nvs_flash
)   