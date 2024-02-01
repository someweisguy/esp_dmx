/*

  RDM Sensor

  This sketch creates an RDM sensor which keeps track of the amount of time in
  seconds that the ESP32 has been on. The sensor value will be accurate for
  approximately 18 hours (65535 seconds) before the value overflows.

  Created 31 January 2024
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include <Arduino.h>
#include <esp_dmx.h>
#include <rdm/responder.h>
#include <time.h>

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

/* In this example, we are going to make a basic RDM sensor that tracks the
  current time in seconds. We will need two variables; one to get the current
  time and one to check to see if the current time has changed since the last
  time the RDM sensor has been set. */
time_t currentTime;
time_t lastTime;

void setup() {
  /* Start the serial connection back to the computer so that we can log
   messages to the Serial Monitor. Lets set the baud rate to 115200. */
  Serial.begin(115200);

  /* Now we will install the DMX driver! We'll tell it which DMX port to use,
    what device configuration to use, and what DMX personalities it should have.
    If you aren't sure which configuration to use, you can use the macros
    `DMX_CONFIG_DEFAULT` to set the configuration to its default settings.
    This device is being setup as an RDM responder so it is likely that it
    should respond to DMX commands. It will need at least one DMX personality.
    Since this is an example, we will use a default personality which only uses
    1 DMX slot in its footprint. */
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {1, "Default Personality"}
  };
  int personality_count = 1;
  dmx_driver_install(dmxPort, &config, personalities, personality_count);

  /* Now set the DMX hardware pins to the pins that we want to use. */
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  /* Now we should be ready to register some sensors! */

  /* Let's decide how many sensors we want to register. To keep things simple, 
    we'll start by registering only one. */
  int sensorCount = 1;

  /* Now we register! We do so with our `rdm_register_` functions. We first
    register RDM_PID_SENSOR_VALUE. This will instantiate the RDM sensors so that
    we can start using them. Since our `rdm_register_` functions allow us to
    define a callback and user context, we should define those variables as
    well. For now we can leave them as NULL. */
  rdm_callback_t callback = NULL;
  void *context = NULL;
  if (!rdm_register_sensor_value(dmxPort, sensorCount, callback, context)) {
    Serial.println("Unable to register RDM_PID_SENSOR_VALUE!");
  }

  /* We should also make sure that RDM controllers can send an
    RDM_PID_RECORD_SENSORS command to this device. This way, if there are any
    issues with this device, the RDM controller can capture the sensor data at a
    specific time in order to read the data back later. */
  if (!rdm_register_record_sensors(dmxPort, callback, context)) {
    Serial.println("Unable to register RDM_PID_RECORD_SENSORS!");
  }

  /* Care should be taken to ensure that the parameters registered for callbacks
    never go out of scope. The variables passed as parameter data for responses
    must be valid throughout the lifetime of the DMX driver. Allowing parameter
    variables to go out of scope can result in undesired behavior during RDM
    response callbacks. */

  /* And that is it! We now have an RDM sensor on our device! We should set the
    initial value of our sensor before going any further. */
  currentTime = time(NULL);
  rdm_sub_device_t deviceNum = RDM_SUB_DEVICE_ROOT;
  int sensorNum = 0;
  rdm_sensor_set(dmxPort, deviceNum, sensorNum, currentTime);

  /* And now update the lastTime variable so that we only update the sensor
    value when the value changes. */
  lastTime = currentTime;
}

void loop() {
  rdm_sub_device_t deviceNum = RDM_SUB_DEVICE_ROOT;
  int sensorNum = 0;
  
  /* Use a basic DMX loop with RDM response handling. */
  if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
    if (packet.is_rdm) {
      rdm_send_response(dmxPort);
    }
  }

  /* Now check the time in every loop. If the time is different from the last
    time the RDM sensor was set, we can update the RDM sensor! */
  currentTime = time(NULL);
  if (currentTime != lastTime) {
    rdm_sensor_set(dmxPort, deviceNum, sensorNum, currentTime);
    lastTime = currentTime;
  }
}
