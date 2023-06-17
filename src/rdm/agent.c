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

/**
 * @brief A packed struct which can be used to help process raw RDM packets
 * instead of reading slots by index alone. RDM sends data in most-significant
 * byte first - endianness must be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_raw_t {
  uint8_t sc;           // The RDM start code.
  uint8_t sub_sc;       // The RDM sub-start code.
  uint8_t message_len;  // Number of slots in the RDM packet excluding checksum.
  uint8_t dest_uid[6];  // The UID of the target device(s).
  uint8_t src_uid[6];   // The UID of the device originating this packet.
  uint8_t tn;           // The transaction number.
  union {
    uint8_t port_id;        // The requesting device's port ID.
    uint8_t response_type;  // The responding device's response type.
  };
  uint8_t message_count;  // The number of packets in the device's packet queue.
  uint16_t sub_device;    // The destination sub-device.
  uint8_t cc;             // The command class.
  uint16_t pid;           // The parameter ID.
  uint8_t pdl;            // The parameter data length.
  struct {
  } pd;  // The RDM parameter data. It can be variable length.
} rdm_raw_t;

void rdm_driver_get_uid(dmx_port_t dmx_num, rdm_uid_t *uid) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Initialize the RDM UID
  taskENTER_CRITICAL(spinlock);
  if (uid_is_null(&driver->rdm.uid)) {
    struct __attribute__((__packed__)) {
      uint16_t manufacturer;
      uint64_t device;
    } mac;
    esp_efuse_mac_get_default((void *)&mac);
    driver->rdm.uid.dev_id = bswap32(mac.device) + dmx_num;
    driver->rdm.uid.man_id = RDM_MAN_ID_DEFAULT;
  }
  *uid = driver->rdm.uid;
  taskEXIT_CRITICAL(spinlock);
}


void rdm_driver_set_uid(dmx_port_t dmx_num, rdm_uid_t uid) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, , "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), , "driver is not installed");
  DMX_CHECK(!uid_is_broadcast(&uid), , "uid error");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);
  driver->rdm.uid = uid;
  taskEXIT_CRITICAL(spinlock);
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
                                    uint8_t *pdl, void *param, unsigned int num,
                                    void *context) {
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
      *pdl = pd_emplace(pd, "u$", &my_uid, sizeof(my_uid), false);
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
    *pdl = pd_emplace(pdl, "wv$", &mute, sizeof(mute), false);
    response_type = RDM_RESPONSE_TYPE_ACK;
  }


  return response_type;
}

bool rdm_register_disc_unique_branch(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UNIQUE_BRANCH,
                                .pdl_size = 6,
                                .data_type = 0,
                                .cc = RDM_CC_DISC,
                                .unit = 0,
                                .prefix = 0,
                                .min_value = 0,
                                .max_value = 0,
                                .default_value = 0,
                                .description = "Discovery Unique Branch"};

  return rdm_register_response(dmx_num, &desc, rdm_default_discovery_cb, NULL,
                               0, NULL);
}

bool rdm_register_disc_mute(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UNIQUE_BRANCH,
                                .pdl_size = 8,
                                .data_type = 0,
                                .cc = RDM_CC_DISC,
                                .unit = 0,
                                .prefix = 0,
                                .min_value = 0,
                                .max_value = 0,
                                .default_value = 0,
                                .description = "Discovery Mute"};

  return rdm_register_response(dmx_num, &desc, rdm_default_discovery_cb, NULL,
                               0, NULL);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_pid_description_t desc = {.pid = RDM_PID_DISC_UNIQUE_BRANCH,
                                .pdl_size = 8,
                                .data_type = 0,
                                .cc = RDM_CC_DISC,
                                .unit = 0,
                                .prefix = 0,
                                .min_value = 0,
                                .max_value = 0,
                                .default_value = 0,
                                .description = "Discovery Un-Mute"};

  return rdm_register_response(dmx_num, &desc, rdm_default_discovery_cb, NULL,
                               0, NULL);
}

// static int rdm_simple_param_cb(dmx_port_t dmx_num, const rdm_header_t *header,
//                                rdm_encode_decode_t *functions, rdm_mdb_t *mdb,
//                                void *param, int num, void *context) {
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

  // const rdm_pid_description_t desc = {
  //     .pid = RDM_PID_DEVICE_INFO, .pdl_size = 0, .cc = RDM_CC_GET};

  // return rdm_register_callback(dmx_num, &desc, &get, NULL, rdm_simple_param_cb,
  //                              device_info, 1, NULL);

  return false;
}

bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         const char *software_version_label) {
  // TODO: arg check

  // const rdm_pid_description_t desc = {.pid = RDM_PID_SOFTWARE_VERSION_LABEL,
  //                                     .pdl_size = 0,
  //                                     .cc = RDM_CC_GET};

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

  return false;
}

bool rdm_register_dmx_start_address(dmx_port_t dmx_num,
                                    uint16_t *dmx_start_address) {
  // TODO: arg check

  // const rdm_pid_description_t desc = {.pid = RDM_PID_DMX_START_ADDRESS,
  //                                     .pdl_size = 2,
  //                                     .cc = RDM_CC_GET_SET,
  //                                     .max_value = 512,
  //                                     .min_value = 1};

  // return rdm_register_callback(dmx_num, &desc, &get, &set,
  //                              rdm_simple_param_cb, dmx_start_address, 1,
  //                              NULL);
  return false;
}