#include "dmx/include/device.h"

#include "dmx/hal/include/nvs.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/responder/include/dmx_setup.h"
#include "rdm/responder/include/utils.h"

uint16_t dmx_get_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint16_t dmx_start_address;
  if (!rdm_get_dmx_start_address(dmx_num, &dmx_start_address)) {
    // This device does not use a DMX address
    dmx_start_address = DMX_START_ADDRESS_NONE;
  }

  return dmx_start_address;
}

bool dmx_set_start_address(dmx_port_t dmx_num, uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_start_address > 0 && dmx_start_address < DMX_PACKET_SIZE_MAX,
            false, "dmx_start_address error");
  DMX_CHECK(dmx_get_start_address(dmx_num) != DMX_START_ADDRESS_NONE, false,
            "cannot set DMX start address");

  if (!rdm_set_dmx_start_address(dmx_num, dmx_start_address)) {
    // An unusual error occurred
    DMX_ERR("unable to set DMX start address");
    return false;
  }

  return true;
}

uint8_t dmx_get_current_personality(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  rdm_dmx_personality_t personality;
  if (!rdm_get_dmx_personality(dmx_num, &personality)) {
    // This device does not use a DMX address
    personality.current = 0;
  }

  return personality.current;
}

bool dmx_set_current_personality(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num error");

  if (!rdm_set_dmx_personality(dmx_num, personality_num)) {
    // An unusual error occurred
    DMX_ERR("unable to set DMX personality");
    return false;
  }

  return true;
}

uint8_t dmx_get_personality_count(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  rdm_dmx_personality_t personality;
  if (!rdm_get_dmx_personality(dmx_num, &personality)) {
    // This device does not use a DMX address
    personality.count = 0;
  }

  return personality.count;
}

size_t dmx_get_footprint(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            0, "personality_num is invalid");

  rdm_dmx_personality_description_t personality;
  if (!rdm_get_dmx_personality_description(dmx_num, personality_num,
                                           &personality)) {
    return 0;
  }

  return personality.footprint;
}

const char *dmx_get_personality_description(dmx_port_t dmx_num,
                                            uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num is invalid");

  // Get a pointer to the stored personality
  const rdm_dmx_personality_description_t *personalities =
      dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT,
                             RDM_PID_DMX_PERSONALITY_DESCRIPTION);
  if (personalities == NULL) {
    return NULL;
  }

  --personality_num;  // Personalities are indexed beginning at 1
  return personalities[personality_num].description;
}
