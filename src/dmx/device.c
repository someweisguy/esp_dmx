#include "dmx/device.h"

#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"

uint16_t dmx_get_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint16_t dmx_start_address;

  if (!rdm_is_enabled(dmx_num)) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    const dmx_driver_personality_t *device = dmx_driver[dmx_num]->pd;
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
    dmx_driver_personality_t *personality = (void *)dmx_driver[dmx_num]->pd;
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

  // Check if RDM is enabled on the driver
  const bool rdm_is_enabled = (dmx_driver[dmx_num]->pd_size >= 53);

  /* If RDM is enabled, attempt to read the current personality from
    RDM_PID_DEVICE_INFO. If RDM_PID_DEVICE_INFO doesn't exist, throw an error
    and return 0.
    If RDM is not enabled, the current personality can be read from
    the device personality struct.*/

  if (rdm_is_enabled) {
    const rdm_device_info_t *device_info =
        rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    if (device_info != NULL) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      current_personality = device_info->current_personality;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    } else {
      DMX_ERR("RDM_PID_DEVICE_INFO must be registered");
      current_personality = 0;
    }
  } else {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    const dmx_driver_personality_t *device = dmx_driver[dmx_num]->pd;
    current_personality = device->current_personality;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
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

  // Check if RDM is enabled on the driver
  const bool rdm_is_enabled = (dmx_driver[dmx_num]->pd_size >= 53);

  /* If RDM is enabled, check if RDM_PID_DMX_PERSONALITY is registered on the
    driver. If it is, simply call rdm_pd_set() to set the personality. If
    it isn't, check if RDM_PID_DMX_DEVICE_INFO is registered. If
    RDM_PID_DEVICE_INFO is registered, make a deep copy of the device info,
    update the personality, and call rdm_pd_set() on the new device info. Then
    explicitly call dmx_nvs_set() to update the current personality in NVS. If
    neither RDM_PID_DMX_PERSONALITY nor RDM_PID_DEVICE_INFO are registered,
    throw an error.
    If RDM is not enabled, the personality can be set to
    the device personality struct.*/

  if (rdm_is_enabled) {
    const uint8_t *current_personality_ptr =
        rdm_pd_get(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT);
    const rdm_device_info_t *device_info_ptr =
        rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    if (current_personality_ptr != NULL) {
      ret = rdm_pd_set(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT,
                       &personality_num, sizeof(uint8_t));
    } else if (device_info_ptr != NULL) {
      rdm_device_info_t device_info;
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      memcpy(&device_info, device_info_ptr, sizeof(rdm_device_info_t));
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      device_info.current_personality = personality_num;
      device_info.footprint = dmx_get_footprint(dmx_num, personality_num);
      ret = rdm_pd_set(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT,
                       &device_info, sizeof(rdm_device_info_t));
    } else {
      DMX_ERR("RDM_PID_DEVICE_INFO must be registered");
      ret = false;
    }
  } else {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    dmx_driver_personality_t *personality = dmx_driver[dmx_num]->pd;
    personality->current_personality = personality_num;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    ret = true;
  }

  // Explicitly record the value to NVS
  if (ret) {
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
  const bool rdm_is_enabled = (dmx_driver[dmx_num]->pd_size >= 53);

  if (rdm_is_enabled) {
    // Get the personality count from the RDM device info
    const rdm_device_info_t *device_info =
        rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    if (device_info != NULL) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      personality_count = device_info->current_personality;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    } else {
      DMX_ERR("RDM_PID_DEVICE_INFO must be registered");
      personality_count = 0;
    }
  } else {
    dmx_driver_personality_t *personality = dmx_driver[dmx_num]->pd;
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    personality_count = personality->personality_count;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  return personality_count;
}

bool dmx_get_personality_description(
    dmx_port_t dmx_num, uint8_t personality_num,
    dmx_personality_description_t *description) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(description != NULL, false, "description is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num is invalid");

  // Copy the values to the personality description struct
  description->personality_num = personality_num;
  description->footprint = dmx_get_footprint(dmx_num, personality_num);
  --personality_num;  // Personalities are indexed starting at 1
  strncpy(description->description,
          dmx_driver[dmx_num]->personalities[personality_num].description, 32);

  return true;
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