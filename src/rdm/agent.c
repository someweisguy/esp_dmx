#include "agent.h"

#include <string.h>

#include "dmx/driver.h"
#include "dmx/hal.h"
#include "dmx/types.h"
#include "endian.h"
#include "esp_log.h"
#include "rdm/utils.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_mac.h"
#endif

static rdm_uid_t binding_uid = {};

/**
 * @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID (as
 * long as it is used responsibly) or may choose to register their own
 * manufacturer ID.
 */
#define RDM_MAN_ID_DEFAULT (0x05e0)

static const char *TAG = "rdm_agent";

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

void rdm_driver_get_uid(dmx_port_t dmx_num, rdm_uid_t *uid) {
  // Initialize the binding UID if it isn't initialized
  if (uid_is_null(&binding_uid)) {
    uint16_t man_id;
    uint32_t dev_id;
#if CONFIG_RDM_DEVICE_UID_MAN_ID == 0
    man_id = RDM_MAN_ID_DEFAULT;
#else
    man_id = CONFIG_RDM_DEVICE_UID_MAN_ID;
#endif
#if CONFIG_RDM_DEVICE_UID_DEV_ID == 0
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    dev_id = bswap32(*(uint32_t *)(mac + 2));
#else
    dev_id = CONFIG_RDM_DEVICE_UID_DEV_ID;
#endif
    binding_uid.man_id = man_id;
    binding_uid.dev_id = dev_id;
  }

  // Return early if there is an argument error
  if (dmx_num >= DMX_NUM_MAX || uid == NULL) {
    return;
  }

  // Copy the binding UID and increment the final octet by dmx_num
  uid->man_id = binding_uid.man_id;
  uid->dev_id = binding_uid.dev_id;
  uint8_t last_octet = (uint8_t)binding_uid.dev_id;
  last_octet += dmx_num;
  uid->dev_id &= 0x00ffffff;
  uid->dev_id |= last_octet;
}

bool rdm_driver_is_muted(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  bool is_muted;
  taskENTER_CRITICAL(spinlock);
  is_muted = driver->rdm.discovery_is_muted;
  taskEXIT_CRITICAL(spinlock);

  return is_muted;
}

bool rdm_driver_get_device_info(dmx_port_t dmx_num,
                                rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  DMX_CHECK(device_info != NULL, false, "device_info is null");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);
  *device_info = driver->rdm.device_info;
  taskEXIT_CRITICAL(spinlock);

  return true;
}

void rdm_driver_set_device_info(dmx_port_t dmx_num,
                                const rdm_device_info_t *device_info) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  DMX_CHECK(device_info != NULL, , "device_info is null");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);
  driver->rdm.device_info = *device_info;
  taskEXIT_CRITICAL(spinlock);
}

int rdm_driver_get_dmx_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);
  int start_address = driver->rdm.device_info.start_address;
  taskEXIT_CRITICAL(spinlock);

  return start_address;
}

void rdm_driver_set_dmx_start_address(dmx_port_t dmx_num, int start_address) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  DMX_CHECK(start_address >= 1 && start_address <= 512, ,
            "start_address is invalid");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);
  driver->rdm.device_info.start_address = start_address;
  taskEXIT_CRITICAL(spinlock);
}

static int rdm_default_discovery_cb(dmx_port_t dmx_num,
                                    const rdm_header_t *header, void *pd,
                                    uint8_t *pdl_out, void *param,
                                    unsigned int num, void *context) {
  // Ignore this message if discovery is muted
  if (rdm_driver_is_muted(dmx_num)) {
    return RDM_RESPONSE_TYPE_NONE;
  }

  rdm_response_type_t response_type;
  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // Get the discovery branch parameters
    rdm_disc_unique_branch_t branch;
    pd_emplace(&branch, "uu$", pd, sizeof(branch), true);

    // Respond if lower_bound <= my_uid <= upper_bound
    rdm_uid_t my_uid;
    rdm_driver_get_uid(dmx_num, &my_uid);
    if (uid_is_ge(&my_uid, &branch.lower_bound) &&
        uid_is_le(&my_uid, &branch.upper_bound)) {
      *pdl_out = pd_emplace(pd, "u$", &my_uid, sizeof(my_uid), false);
      response_type = RDM_RESPONSE_TYPE_ACK;
    } else {
      response_type = RDM_RESPONSE_TYPE_NONE;
    }
  } else {
    // Mute or un-mute the discovery responses
    dmx_driver[dmx_num]->rdm.discovery_is_muted =
        (header->pid == RDM_PID_DISC_MUTE);

    // Get the binding UID of this device
    int num_ports = 0;
    rdm_uid_t binding_uid;
    for (int i = 0; i < DMX_NUM_MAX; ++i) {
      if (dmx_driver_is_installed(i)) {
        if (num_ports == 0) {
          rdm_driver_get_uid(i, &binding_uid);
        }
        ++num_ports;
      }
    }
    if (num_ports == 1) {
      // Only report binding UID if there are multiple ports
      binding_uid = RDM_UID_NULL;
    }

    // Respond with this device's mute parameters
    const rdm_disc_mute_t mute = {
        .control_field = 0,  // TODO: get the control_field of the device
        .binding_uid = binding_uid,
    };
    *pdl_out = pd_emplace(pd, "wv$", &mute, sizeof(mute), false);
    response_type = RDM_RESPONSE_TYPE_ACK;
  }

  return response_type;
}

bool rdm_register_disc_unique_branch(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UNIQUE_BRANCH,
                                .pdl_size = sizeof(rdm_uid_t),
                                .data_type = RDM_DS_BIT_FIELD,
                                .cc = RDM_CC_DISC,
                                .unit = RDM_UNITS_NONE,
                                .prefix = RDM_PREFIX_NONE,
                                .min_value = 0,
                                .max_value = 0,
                                .default_value = 0,
                                .description = "Discovery Unique Branch"};

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_default_discovery_cb, NULL, 0, NULL);
}

bool rdm_register_disc_mute(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_pid_description_t desc = {.pid = RDM_PID_DISC_MUTE,
                                .pdl_size = sizeof(rdm_disc_mute_t),
                                .data_type = RDM_DS_BIT_FIELD,
                                .cc = RDM_CC_DISC,
                                .unit = RDM_UNITS_NONE,
                                .prefix = RDM_PREFIX_NONE,
                                .min_value = 0,
                                .max_value = 0,
                                .default_value = 0,
                                .description = "Discovery Mute"};

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_default_discovery_cb, NULL, 0, NULL);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UN_MUTE,
                                .pdl_size = sizeof(rdm_disc_mute_t),
                                .data_type = RDM_DS_BIT_FIELD,
                                .cc = RDM_CC_DISC,
                                .unit = RDM_UNITS_NONE,
                                .prefix = RDM_PREFIX_NONE,
                                .min_value = 0,
                                .max_value = 0,
                                .default_value = 0,
                                .description = "Discovery Un-Mute"};

  return rdm_register_response(dmx_num, RDM_SUB_DEVICE_ROOT, &desc,
                               rdm_default_discovery_cb, NULL, 0, NULL);
}

// static int rdm_simple_param_cb(dmx_port_t dmx_num, const rdm_header_t
// *header,
//                                rdm_encode_decode_t *functions, rdm_mdb_t
//                                *mdb, void *param, int num, void *context) {
//   // if (functions->decode != NULL) {
//   //   functions->decode(mdb, param, num);
//   // }
//   // if (functions->encode != NULL) {
//   //   functions->encode(mdb, param, num);
//   // }
//   return RDM_RESPONSE_TYPE_ACK;
// }

bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info) {
  // TODO: arg check

  rdm_pid_description_t desc = {.pid = RDM_PID_DEVICE_INFO,
                                .pdl_size = sizeof(rdm_device_info_t),
                                .data_type = RDM_DS_BIT_FIELD,
                                .cc = RDM_CC_GET,
                                .unit = RDM_UNITS_NONE,
                                .prefix = RDM_PREFIX_NONE,
                                .min_value = 0,
                                .max_value = 0,
                                .default_value = 0,
                                .description = "Device Info"};

  // return rdm_register_callback(dmx_num, &desc, &get, NULL,
  // rdm_simple_param_cb,
  //                              device_info, 1, NULL);

  return false;
}

bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         const char *software_version_label) {
  // TODO: arg check

  rdm_pid_description_t desc = {.pid = RDM_PID_SOFTWARE_VERSION_LABEL,
                                .pdl_size = 32,
                                .data_type = RDM_DS_ASCII,
                                .cc = RDM_CC_GET,
                                .unit = RDM_UNITS_NONE,
                                .prefix = RDM_PREFIX_NONE,
                                .min_value = 0,
                                .max_value = 0,
                                .default_value = 0,
                                .description = "Software Version Label"};

  // size_t len = strlen(software_version_label);
  // if (len > 32) {
  //   len = 32;
  // }

  // return rdm_register_callback(dmx_num, &desc, &get, NULL,
  //                              rdm_simple_param_cb,
  //                              (void *)software_version_label, len, NULL);
  return false;
}

bool rdm_register_identify_device(dmx_port_t dmx_num) {
  // TODO

  rdm_pid_description_t desc = {.pid = RDM_PID_IDENTIFY_DEVICE,
                                .pdl_size = sizeof(uint8_t),
                                .data_type = RDM_DS_UNSIGNED_BYTE,
                                .cc = RDM_CC_GET_SET,
                                .unit = RDM_UNITS_NONE,
                                .prefix = RDM_PREFIX_NONE,
                                .min_value = 0,
                                .max_value = 1,
                                .default_value = 0,
                                .description = "Identify Device"};

  return false;
}

bool rdm_register_dmx_start_address(dmx_port_t dmx_num,
                                    uint16_t *dmx_start_address) {
  // TODO: arg check

  rdm_pid_description_t desc = {.pid = RDM_PID_DMX_START_ADDRESS,
                                .pdl_size = sizeof(uint16_t),
                                .data_type = RDM_DS_UNSIGNED_WORD,
                                .cc = RDM_CC_GET_SET,
                                .unit = RDM_UNITS_NONE,
                                .prefix = RDM_PREFIX_NONE,
                                .min_value = 1,
                                .max_value = 512,
                                .default_value = 1,
                                .description = "DMX Start Address"};

  // return rdm_register_callback(dmx_num, &desc, &get, &set,
  //                              rdm_simple_param_cb, dmx_start_address, 1,
  //                              NULL);
  return false;
}