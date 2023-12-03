#include "parameters.h"

#include "dmx/device.h"
#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"

bool rdm_get_device_info(dmx_port_t dmx_num, rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(device_info != NULL, 0, "device_info is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Get the parameter and copy it to the user's pointer
  const rdm_device_info_t *di =
      rdm_pd_get_pointer(dmx_num, RDM_PID_DEVICE_INFO, 0);
  DMX_CHECK(di != NULL, false, "device_info not registered");
  memcpy(device_info, di, sizeof(rdm_device_info_t));

  return true;
}

bool rdm_get_software_version_label(dmx_port_t dmx_num,
                                    char *software_version_label,
                                    size_t *size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(software_version_label != NULL, 0,
            "software_version_label is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Get the parameter and copy it to the user's pointer
  const char *svl =
      rdm_pd_get_pointer(dmx_num, RDM_PID_SOFTWARE_VERSION_LABEL, 0);
  DMX_CHECK(svl != NULL, false, "software_version_label not registered");
  *size = strnlen(svl, *size);  // Prevent buffer overflows
  strncpy(software_version_label, svl, *size);

  return true;
}

bool rdm_get_identify_device(dmx_port_t dmx_num, uint8_t *identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify != NULL, 0, "identify is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Get the parameter and copy it to the user's pointer
  const uint8_t *id = rdm_pd_get_pointer(dmx_num, RDM_PID_IDENTIFY_DEVICE, 0);
  DMX_CHECK(id != NULL, false, "identify_device not registered");
  *identify = *id;

  return true;
}

bool rdm_set_identify_device(dmx_port_t dmx_num, const uint8_t identify) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(identify == 0 || identify == 1, 0, "identify error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return rdm_pd_set(dmx_num, RDM_PID_IDENTIFY_DEVICE, 0, &identify,
                    sizeof(uint8_t));
}

bool rdm_get_dmx_start_address(dmx_port_t dmx_num,
                               uint16_t *dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address != NULL, 0, "dmx_start_address is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const bool rdm_is_enabled = true;  // FIXME

  if (rdm_is_enabled) {
    const void *pd = rdm_pd_get_pointer(dmx_num, RDM_PID_DMX_START_ADDRESS,
                                        RDM_SUB_DEVICE_ROOT);
    memcpy(dmx_start_address, pd, sizeof(uint16_t));
  } else {
    *dmx_start_address = dmx_get_start_address(dmx_num);
  }

  return (*dmx_start_address != 0);
}

bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_start_address > 0 && dmx_start_address < DMX_PACKET_SIZE_MAX, 0,
            "dmx_start_address error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  bool ret;

  if (rdm_is_enabled(dmx_num)) {
    ret = rdm_pd_set_and_queue(dmx_num, RDM_PID_DMX_START_ADDRESS,
                               RDM_SUB_DEVICE_ROOT, &dmx_start_address,
                               sizeof(uint16_t));

    // Explicitly record the value to NVS
    if (ret) {
      // FIXME: deferred NVS
      dmx_nvs_set(dmx_num, RDM_PID_DMX_START_ADDRESS, RDM_SUB_DEVICE_ROOT,
                  RDM_DS_UNSIGNED_WORD, &dmx_start_address, sizeof(uint16_t));
    }
  } else {
    ret = dmx_set_start_address(dmx_num, dmx_start_address);
  }

  return ret;
}

bool rdm_get_current_personality(dmx_port_t dmx_num,
                                 rdm_dmx_personality_t *personality) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(personality != NULL, 0, "personality is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  if (rdm_is_enabled(dmx_num)) {
    // if (rdm_pd_exists(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT)) {
    //   const rdm_dmx_personality_t *pd = rdm_pd_get_pointer(
    //       dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT);
    //   taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    //   memcpy(personality, pd, sizeof(rdm_dmx_personality_t));
    //   taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    // } else if (rdm_pd_exists(dmx_num, RDM_PID_DEVICE_INFO,
    //                          RDM_SUB_DEVICE_ROOT)) {
    //   const rdm_device_info_t *pd =
    //       rdm_pd_get_pointer(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    //   taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    //   personality->current_personality = pd->current_personality;
    //   personality->personality_count = pd->personality_count;
    //   taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    // } else {
    //   // An unusual error occurred
    //   DMX_ERR("unable to get current DMX personality");
    //   return false;
    // }
  } else {
    personality->current_personality = dmx_get_current_personality(dmx_num);
    personality->personality_count = dmx_get_personality_count(dmx_num);
  }

  return true;
}

bool rdm_set_current_personality(dmx_port_t dmx_num, uint8_t personality_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK((personality_num > 0 &&
             personality_num <= dmx_get_personality_count(dmx_num)),
            false, "personality_num error");

  bool ret = false;

  if (rdm_is_enabled(dmx_num)) {
    // if (rdm_pd_exists(dmx_num, RDM_PID_DMX_PERSONALITY, RDM_SUB_DEVICE_ROOT)) {
    //   ret = rdm_pd_set_and_queue(dmx_num, RDM_PID_DMX_PERSONALITY,
    //                              RDM_SUB_DEVICE_ROOT, &personality_num,
    //                              sizeof(uint8_t));
    // } else if (rdm_pd_exists(dmx_num, RDM_PID_DEVICE_INFO,
    //                          RDM_SUB_DEVICE_ROOT)) {
    //   rdm_device_info_t device_info;
    //   const void *pd =
    //       rdm_pd_get_pointer(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT);
    //   taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    //   memcpy(&device_info, pd, sizeof(rdm_device_info_t));
    //   taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    //   device_info.current_personality = personality_num;
    //   ret = rdm_pd_set(dmx_num, RDM_PID_DEVICE_INFO, RDM_SUB_DEVICE_ROOT,
    //                    &device_info, sizeof(rdm_device_info_t));
    // } else {
    //   // An unusual error occurred
    //   DMX_ERR("unable to set current DMX personality");
    //   return false;
    // }
  } else {
    ret = dmx_set_current_personality(dmx_num, personality_num);
  }

  return ret;
}

size_t rdm_get_device_label(dmx_port_t dmx_num, char *label, size_t label_len) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  const char *rdm_label = rdm_pd_get_pointer(dmx_num, RDM_PID_DEVICE_LABEL, 0);
  DMX_CHECK(rdm_label != NULL, 0, "RDM_PID_DEVICE_LABEL not found");

  const size_t rdm_label_len = strnlen(rdm_label, 32);  // length without '\0'
  const size_t size = label_len < rdm_label_len ? label_len : rdm_label_len;
  strncpy(label, rdm_label, size);

  return size;
}