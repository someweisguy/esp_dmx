idf_component_register(
  SRCS 
       # DMX driver and HAL
       "src/dmx/hal/uart.c" "src/dmx/hal/timer.c" "src/dmx/hal/nvs.c"
       "src/dmx/hal/gpio.c" "src/dmx/driver.c" "src/dmx/io.c" "src/dmx/device.c"
       "src/dmx/parameter.c" "src/dmx/sniffer.c" 

       # RDM Controller
       "src/rdm/controller/discovery.c" "src/rdm/controller/product_info.c"
       "src/rdm/controller/device_control.c" "src/rdm/controller/dmx_setup.c"
       "src/rdm/controller/utils.c"
       
       # RDM Responder
       "src/rdm/responder/discovery.c" "src/rdm/responder/product_info.c"
       "src/rdm/responder/rdm_info.c" "src/rdm/responder/device_control.c"
       "src/rdm/responder/queue_status.c" "src/rdm/responder/dmx_setup.c"
       "src/rdm/responder/sensor_parameter.c" "src/rdm/responder/utils.c"
  INCLUDE_DIRS "src" 
  REQUIRES driver esp_timer esp_common esp_hw_support nvs_flash
)   