/*

  ESP-IDF RDM Responder

  Configure this device to act as an RDM responder. This example sets the
  device's device info, which defines many of the device's basic capabilities.
  This example also registers a custom callback for
  RDM_PID_SOFTWARE_VERSION_LABEL GET requests. This PID is already registered
  when installing the DMX driver, but is overwritten for demonstration purposes.
  Afterwards, this example loops continuously in a simple DMX receive loop which
  allows for processing and responding to RDM requests.

  Note: this example is for use with the ESP-IDF. It will not work on Arduino!

  Created 20 May 2023
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "rdm/mdb.h"

#define TX_PIN 17  // The DMX transmit pin.
#define RX_PIN 16  // The DMX receive pin.
#define EN_PIN 21  // The DMX transmit enable pin.

static const char *TAG = "main";

// Define a custom response to RDM_PID_SOFTWARE_VERSION_LABEL
rdm_response_type_t rdm_software_version_label(dmx_port_t dmx_num,
                                               const rdm_header_t *header,
                                               rdm_mdb_t *mdb, void *context) {
  // Log the request for testing purposes only
  ESP_LOGI(TAG, "Received RDM_PID_SOFTWARE_VERSION_LABEL request");

  // Ensure that the parameter data is the expected length
  if (mdb->pdl != 0) {
    rdm_encode_nack_reason(mdb, RDM_NR_FORMAT_ERROR);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Ensure that the CC is correct
  if (header->cc != RDM_CC_GET_COMMAND) {
    // RDM_PID_SOFTWARE_VERSION_LABEL only supports GET requests
    rdm_encode_nack_reason(mdb, RDM_NR_UNSUPPORTED_COMMAND_CLASS);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }

  // Encode the response
  const char *sw_version_label = (const char *)context;
  rdm_encode_string(mdb, sw_version_label, strlen(sw_version_label));
  return RDM_RESPONSE_TYPE_ACK;
}

void app_main() {
  const dmx_port_t dmx_num = DMX_NUM_2;
  ESP_ERROR_CHECK(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));
  ESP_ERROR_CHECK(dmx_driver_install(dmx_num, DMX_DEFAULT_INTR_FLAGS));

  // Set the device info for this device driver
  const rdm_device_info_t device_info = {
      .model_id = 1,  // An arbitrary value defined by the user
      .product_category = RDM_PRODUCT_CATEGORY_FIXTURE,
      .software_version_id = 1,  // An arbitrary value defined by the user
      .footprint = 1,
      .current_personality = 1,  // Begins at 1, not 0
      .personality_count = 1,
      .start_address = 1,
      .sub_device_count = 0,
      .sensor_count = 0};
  rdm_driver_set_device_info(dmx_num, &device_info);

  /* Register the custom callback. This overwrites the default
    RDM_PID_SOFTWARE_VERSION_LABEL response. */
  const char *sw_version_label = "My Custom RDM Software";
  rdm_register_callback(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL,
                        rdm_software_version_label, (void *)sw_version_label);

  /* Care should be taken to ensure that sw_version_label (the user context)
    never goes out of scope. Allowing this to happen can lead to undesired
    behavior. User contexts are not copied into the driver. */

  ESP_LOGI(TAG, "Waiting for RDM packets...");

  dmx_packet_t packet;
  while (true) {
    // Block until a packet is received
    dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK);

    // Handle packets here...
  }
}
