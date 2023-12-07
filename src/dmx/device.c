#include "dmx/device.h"

#include "dmx/driver.h"
#include "dmx/hal/include/nvs.h"
#include "dmx/struct.h"

uint16_t dmx_get_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint16_t dmx_start_address;

  if (!rdm_is_enabled(dmx_num)) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    const dmx_driver_personality_t *device = dmx_driver[dmx_num]->rdm.heap_ptr;
    dmx_start_address = device->dmx_start_address;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else if (!rdm_get_dmx_start_address(dmx_num, &dmx_start_address)) {
    // An unusual error occurred
    DMX_ERR("unable to get DMX start address");
    dmx_start_address = 0;
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

  bool ret = false;

  if (!rdm_is_enabled(dmx_num)) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->rdm.heap_ptr;
    personality->dmx_start_address = dmx_start_address;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    ret = true;

    // Explicitly record the value to NVS
    if (ret) {
      // FIXME: deferred NVS
      dmx_nvs_set(dmx_num, RDM_PID_DMX_START_ADDRESS, RDM_SUB_DEVICE_ROOT,
                  RDM_DS_UNSIGNED_WORD, &dmx_start_address, sizeof(uint16_t));
    }
  } else if (!rdm_set_dmx_start_address(dmx_num, dmx_start_address)) {
    // An unusual error occurred
    DMX_ERR("unable to set DMX start address");
  }

  return ret;
}

uint8_t dmx_get_current_personality(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint8_t current_personality;

  if (!rdm_is_enabled(dmx_num)) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    const dmx_driver_personality_t *device = dmx_driver[dmx_num]->rdm.heap_ptr;
    current_personality = device->current_personality;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    rdm_dmx_personality_t pers;
    rdm_get_dmx_personality(dmx_num, &pers);
    current_personality = pers.current_personality;
  }

  return current_personality;
}

bool dmx_set_current_personality(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num error");

  bool ret;

  if (!rdm_is_enabled(dmx_num)) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    dmx_driver_personality_t *personality = dmx_driver[dmx_num]->rdm.heap_ptr;
    personality->current_personality = personality_num;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    ret = true;
  } else {
    ret = rdm_set_dmx_personality(dmx_num, personality_num);
  }

  // Explicitly record the value to NVS
  if (ret) {
    // FIXME: deferred NVS
    ret = dmx_nvs_set(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT,
                      RDM_DS_UNSIGNED_BYTE, &personality_num, sizeof(uint8_t));
  }

  return ret;
}

uint8_t dmx_get_personality_count(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint8_t personality_count;

  // Check if RDM is enabled on the driver
  const bool rdm_is_enabled = true;  // FIXME

  if (rdm_is_enabled) {
    // Get the personality count from the RDM device info
    const rdm_device_info_t *device_info =
        rdm_pd_get_ptr(dmx_num, RDM_SUB_DEVICE_ROOT, RDM_PID_DEVICE_INFO);
    if (device_info != NULL) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      personality_count = device_info->current_personality;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    } else {
      DMX_ERR("RDM_PID_DEVICE_INFO must be registered");
      personality_count = 0;
    }
  } else {
    dmx_driver_personality_t *personality = dmx_driver[dmx_num]->rdm.heap_ptr;
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    personality_count = personality->personality_count;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  return personality_count;
}

size_t dmx_get_footprint(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            0, "personality_num is invalid");

  --personality_num;  // Personalities are indexed starting at 1
  return dmx_driver[dmx_num]->personalities[personality_num].footprint;
}

const char *dmx_get_personality_description(dmx_port_t dmx_num,
                                            uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num is invalid");

  --personality_num;  // Personalities are indexed starting at 1
  return dmx_driver[dmx_num]->personalities[personality_num].description;
}