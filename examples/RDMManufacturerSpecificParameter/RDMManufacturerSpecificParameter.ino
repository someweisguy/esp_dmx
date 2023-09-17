#include <Arduino.h>
#include <esp_dmx.h>
#include <rdm/responder.h>

uint8_t myBool = 0;
void myBoolChanged(dmx_port_t dmxPort, const rdm_header_t *header,
                   void *context)
{
  if (header->cc == RDM_CC_SET_COMMAND_RESPONSE)
  {
    Serial.printf("My bool changed to %d\n", myBool);
  }
}

void setup()
{
  Serial.begin(115200);

  const auto config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(DMX_NUM_2, &config, DMX_INTR_FLAGS_DEFAULT);
  dmx_set_pin(DMX_NUM_2, 17, 16, 21);

  rdm_pid_description_t desc;
  desc.pid = RDM_PID_MANUFACTURER_SPECIFIC_BEGIN;
  desc.pdl_size = 0x1;
  desc.data_type = RDM_DS_UNSIGNED_BYTE;
  desc.cc = RDM_CC_GET_SET;
  desc.unit = RDM_UNITS_NONE;
  desc.prefix = RDM_PREFIX_NONE;
  desc.min_value = 0;
  desc.max_value = 1;
  desc.default_value = 1;
  strlcpy(desc.description, "my bool", strlen("my bool") + 1);

  const char *param_str = "b$";

  rdm_register_manufacturer_specific_simple(DMX_NUM_2, desc, &myBool, param_str, myBoolChanged, NULL);
}

void loop()
{
  dmx_packet_t packet;
  unsigned long now = millis();
  dmx_receive(DMX_NUM_2, &packet, DMX_TIMEOUT_TICK);
}
