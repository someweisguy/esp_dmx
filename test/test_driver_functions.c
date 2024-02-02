#include "esp_dmx.h"
#include "unity.h"

#define TX_PIN 17  // The DMX transmit pin.
#define RX_PIN 16  // The DMX receive pin.
#define EN_PIN 21  // The DMX transmit enable pin.

TEST_CASE("Install and delete the default driver", "[driver]") {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {1, "Default Personality"}
  };
  const int personality_count = 1;
  TEST_ASSERT(
      dmx_driver_install(dmx_num, &config, personalities, personality_count));
  TEST_ASSERT(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));
  TEST_ASSERT(dmx_driver_delete(dmx_num));
}

TEST_CASE("Install and delete the default driver with no DMX personalities", "[driver]") {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  const int personality_count = 0;
  TEST_ASSERT(
      dmx_driver_install(dmx_num, &config, NULL, personality_count));
  TEST_ASSERT(dmx_set_pin(dmx_num, TX_PIN, RX_PIN, EN_PIN));
  TEST_ASSERT(dmx_driver_delete(dmx_num));
}

TEST_CASE("Install the DMX driver twice", "[driver]") {
  const dmx_port_t dmx_num = DMX_NUM_1;
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  const int personality_count = 0;
  TEST_ASSERT(
      dmx_driver_install(dmx_num, &config, NULL, personality_count));
  TEST_ASSERT_FALSE(
      dmx_driver_install(dmx_num, &config, NULL, personality_count));
  TEST_ASSERT(dmx_driver_delete(dmx_num));
}