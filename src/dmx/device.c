#include "dmx/device.h"

#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"

uint16_t dmx_get_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint16_t dmx_start_address;

  // Check if RDM is enabled on the driver
  const bool rdm_is_enabled = (dmx_driver[dmx_num]->pd_size >= 53);

  /* If RDM is enabled, attempt to read the DMX start address from
    RDM_PID_DEVICE_INFO. If RDM_PID_DEVICE_INFO doesn't exist, throw an error
    and return 0.
    If RDM is not enabled, the DMX start address can be read from
    the device personality struct.*/

  if (rdm_is_enabled) {
    const rdm_device_info_t *device_info =
        rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    if (device_info != NULL) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      dmx_start_address = device_info->dmx_start_address;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    } else {
      DMX_ERR("RDM_PID_DEVICE_INFO must be registered");
      dmx_start_address = 0;
    }
  } else {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    const dmx_driver_personality_t *device = dmx_driver[dmx_num]->pd;
    dmx_start_address = device->dmx_start_address;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
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

  bool ret;

  // Check if RDM is enabled on the driver
  const bool rdm_is_enabled = (dmx_driver[dmx_num]->pd_size >= 53);

  /* If RDM is enabled, check if RDM_PID_DMX_START address is registered on the
    driver. If it is, simply call rdm_pd_set() to set the DMX start address. If
    it isn't, check if RDM_PID_DMX_DEVICE_INFO is registered. If
    RDM_PID_DEVICE_INFO is registered, make a deep copy of the device info,
    update the DMX start address, and write the DMX start address. Then
    explicitly call dmx_nvs_set() to update the start address in NVS. If neither
    RDM_PID_DMX_START_ADDRESS nor RDM_PID_DEVICE_INFO are registered, throw an
    error.
    If RDM is not enabled, the DMX start address can be set to the device 
    personality struct.*/

  if (rdm_is_enabled) {
    const uint16_t *dmx_start_address_ptr =
        rdm_pd_get(dmx_num, RDM_PID_DMX_START_ADDRESS, RDM_SUB_DEVICE_ROOT);
    const rdm_device_info_t *device_info_ptr =
        rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    if (dmx_start_address_ptr != NULL) {
      ret = rdm_pd_set(dmx_num, RDM_PID_DMX_START_ADDRESS, RDM_SUB_DEVICE_ROOT,
                       &dmx_start_address, sizeof(uint16_t), true);
    } else if (device_info_ptr != NULL) {
      rdm_device_info_t device_info;
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      memcpy(&device_info, device_info_ptr, sizeof(rdm_device_info_t));
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      device_info.dmx_start_address = dmx_start_address;
      ret = rdm_pd_set(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT,
                       &device_info, sizeof(rdm_device_info_t), false);
      if (ret) {
        ret = dmx_nvs_set(dmx_num, RDM_PID_DMX_START_ADDRESS,
                          RDM_DS_UNSIGNED_WORD, &dmx_start_address,
                          sizeof(uint16_t));
      }
    } else {
      DMX_ERR("RDM_PID_DEVICE_INFO must be registered");
      ret = false;
    }
  } else {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    dmx_driver_personality_t *personality = (void *)dmx_driver[dmx_num]->pd;
    personality->dmx_start_address = dmx_start_address;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    ret = true;
  }

  return ret;
}

uint8_t dmx_get_current_personality(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint8_t current_personality;

  const rdm_device_info_t *device_info =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (device_info == NULL) {
    const dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->pd;
    current_personality = personality->current_personality;
  } else {
    current_personality = device_info->current_personality;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return current_personality;
}

bool dmx_set_current_personality(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num error");

  // Get the required personality values from RDM device info or DMX driver
  uint8_t *current_personality;
  uint16_t *footprint;
  const rdm_device_info_t *device_info =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  if (device_info == NULL) {
    dmx_driver_personality_t *personality = (void *)dmx_driver[dmx_num]->pd;
    current_personality = &personality->current_personality; // FIXME: const cast
    footprint = NULL;
  } else {
    current_personality = &device_info->current_personality;
    // FIXME: correctly set current personality
    footprint = (void *)device_info + offsetof(rdm_device_info_t, footprint);
  }

  // Set the new personality
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  *current_personality = personality_num;
  if (footprint != NULL) {
    *footprint = dmx_get_footprint(dmx_num, personality_num);
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  if (device_info != NULL) {
    // TODO: send message to RDM queue
  }

  return true;
}

uint8_t dmx_get_personality_count(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const rdm_device_info_t *device_info =
      rdm_pd_get(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
  if (device_info == NULL) {
    const dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->pd;
    return personality->personality_count;
  } else {
    return device_info->personality_count;
  }
}

const char *dmx_get_personality_description(dmx_port_t dmx_num,
                                            uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, NULL, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), NULL, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            NULL, "personality_num is invalid");

  --personality_num;  // Personalities are indexed starting at 1
  return dmx_driver[dmx_num]->personalities[personality_num].description;
}

size_t dmx_get_footprint(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            0, "personality_num is invalid");

  --personality_num;  // Personalities are indexed starting at 1

  size_t fp;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  fp = dmx_driver[dmx_num]->personalities[personality_num].footprint;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return fp;
}