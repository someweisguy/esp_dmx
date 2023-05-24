/*

  RDM Responder

  Configure this device to act as an RDM responder. This example sets the
  device's device info, which defines many of the device's basic capabilities.
  This example also registers a custom callback for RDM_PID_DMX_START_ADDRESS
  requests. This PID is already registered when installing the DMX driver, but
  is overwritten for demonstration purposes. Afterwards, this example loops
  continuously in a simple DMX receive loop which allows for processing and
  responding to RDM requests.

  Created 23 May 2023
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include <Arduino.h>
#include <esp_dmx.h>

/* First, lets define the hardware pins that we are using with our ESP32. We
  need to define which pin is transmitting data and which pin is receiving data.
  DMX circuits also often need to be told when we are transmitting and when we
  are receiving data. We can do this by defining an enable pin. */
int transmitPin = 17;
int receivePin = 16;
int enablePin = 21;
/* Make sure to double-check that these pins are compatible with your ESP32!
  Some ESP32s, such as the ESP32-WROVER series, do not allow you to read or
  write data on pins 16 or 17, so it's always good to read the manuals. */

/* Next, lets decide which DMX port to use. The ESP32 has either 2 or 3 ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 1! */
dmx_port_t dmxPort = 1;

/* Now we will define a callback to handle requests for
  RDM_PID_DMX_START_ADDRESS. This callback function should be able to
  handle GET and SET requests. Some PIDs only support either GET or SET, but
  this PID supports both. This particular PID is already defined when installing
  the DMX driver. We will overwrite it for demonstration purposes. */
rdm_respnose_type_t rdm_dmx_start_address(dmx_port_t dmx_num,
                                          const rdm_header_t *header,
                                          rdm_mdb_t *mdb, void *context) {
  /* The arguments in this function include dmx_num: the port number to the
    DMX driver calling this function; header: a struct which contains
    information about the RDM device sending this request; mdb: a struct which
    contains information about the message data block (MDB) or main payload of
    the request; and context: which points to a user-provided context for the
    callback. */

  /* We'll log the request for testing purposes only. It is good practice to
    avoid printing to the Serial Monitor too much in these callbacks as it may
    cause the DMX driver to send an RDM response too late. Printing to the
    Serial Monitor once or twice should be fine. */
  Serial.println("Received RDM_PID_DMX_START_ADDRESS request.");

  /* Now we will parse the request. RDM_PID_DMX_START_ADDRESS supports both the
    GET and SET command class so we must check to see which command class was
    used. */
  if (header->cc == RDM_CC_GET_COMMAND) {
    /* The request is an RDM_CC_GET_COMMAND. This means that the RDM controller
     is attempting to GET this device's DMX start address. We will get the DMX
     start address stored on the DMX driver and encode it onto the message data
     block. We can use rdm_encode_16bit() to encode a single 16-bit number into
     the message data block. */
    uint16_t start_address = rdm_driver_get_dmx_start_address(dmx_num);
    rdm_encode_16bit(mdb, &start_address, 1);

    /* Different types of request require different encoding functions. In this
      example, we will need to encode a 16-bit value. Other PIDs require
      different encoding functions. */

    /* Because the response was handled successfully, we should return an ACK.
      We can do this with RDM_RESPONSE_TYPE_ACK. */
    return RDM_RESPONSE_TYPE_ACK;

  } else if (header->cc == RDM_CC_SET_COMMAND) {
    /* This request is an RDM_CC_SET_COMMAND. In this case, the RDM controller
      is attempting to set this device's DMX start address to a new value. The
      DMX start address may be between the values of 1 and 512, inclusive. If
      this device is not DMX controllable, the DMX start address must be set to
      DMX_START_ADDRESS_NONE. In this case, we will assume that this device uses
      at least 1 DMX slot. */

    /* The RDM controller encoded a new DMX start address for this device in the
      message data block. We must decode it

    To get the value to which the RDM controller is requesting that this device
      set its DMX start address, */
    uint16_t start_address;
    rdm_decode_16bit(mdb, &start_address, 1);
    if (start_address >= 1 && start_address <= 512) {
      /* The received DMX start address is valid. We will set the driver DMX
        start address to its new value and return RDM_RESPONSE_TYPE_ACK. */
      rdm_driver_set_dmx_start_address(dmx_num, start_address);
      return RDM_RESPONSE_TYPE_ACK;
    } else {
      /* The received DMX start address is invalid so we must encode the
        appropriate NACK reason (in this case RDM_NR_DATA_OUT_OF_RANGE) and
        return RDM_RESPONSE_TYPE_NACK_REASON. */
      rdm_encode_nack_reason(mdb, RDM_NR_DATA_OUT_OF_RANGE);
      return RDM_RESPONSE_TYPE_NACK_REASON;
    }
  } else {
    /* If the CC is neither RDM_CC_GET_COMMAND nor RDM_CC_SET_COMMAND, something
      has gone wrong. We should encode RDM_NR_FORMAT_ERROR and return a NACK. */
    rdm_encode_nack_reason(mdb, RDM_NR_FORMAT_ERROR);
    return RDM_RESPONSE_TYPE_NACK_REASON;
  }
}

void setup() {
  /* Start the serial connection back to the computer so that we can log
   messages to the Serial Monitor. Lets set the baud rate to 115200. */
  Serial.begin(115200);

  /* Set the DMX hardware pins to the pins that we want to use. */
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  /* Now we can install the DMX driver! We'll tell it which DMX port to use and
    which interrupt priority it should have. If you aren't sure which interrupt
    priority to use, you can use the macro `DMX_DEFAULT_INTR_FLAG` to set the
    interrupt to its default settings.*/
  dmx_driver_install(dmxPort, DMX_DEFAULT_INTR_FLAGS);

  // Set the device info for this device driver
  const rdm_device_info_t deviceInfo = {
      .model_id = 1,  // An arbitrary value defined by the user.
      .product_category = RDM_PRODUCT_CATEGORY_FIXTURE,
      .software_version_id = 1,  // An arbitrary value defined by the user.
      .footprint = 1,
      .current_personality = 1,  // Begins at 1, not 0.
      .personality_count = 1,
      .start_address = 1,
      .sub_device_count = 0,
      .sensor_count = 0};
  rdm_driver_set_device_info(dmx_num, &deviceInfo);

  /* Register the custom callback. This overwrites the default
    RDM_PID_DMX_START_ADDRESS response. */
  rdm_register_callback(dmx_num, RDM_PID_DMX_START_ADDRESS,
                        rdm_dmx_start_address, NULL);

  /* Care should be taken to ensure that the user context never goes out of
    scope. Allowing this to happen can lead to undesired behavior. User contexts
    are not copied into the driver. In this example, we won't need to worry
    about this because the user context is NULL. */
}

void loop() {
  /* We need a place to store information about the DMX packets we receive. We
    will use a dmx_packet_t to store that packet information.  */
  dmx_packet_t packet;

  /* Now we will block until data is received. If an RDM request for this device
    is received, the dmx_receive() function will automatically respond to the
    requesting RDM device with the appropriate callback. */
  dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK);

  /* Typically, you would handle your packet information here. Since this is
    just an example, this section has been left blank. */
}
