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

bool rdm_register_callback(dmx_port_t dmx_num,
                           const rdm_pid_description_t *desc,
                           const rdm_encode_decode_t *get,
                           const rdm_encode_decode_t *set,
                           rdm_response_cb_t callback, 
                           void *param, const int num,
                           void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");
  // TODO: desc and callback is required
  
  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);

  // Iterate the callback list to see if a callback with this PID exists
  int i = 0;
  for (; i < driver->rdm.num_callbacks; ++i) {
    if (driver->rdm.cbs[i].desc.pid == desc->pid) break;
  }

  // Check if there is space for callbacks
  if (i >= CONFIG_RDM_RESPONDER_MAX_PARAMETERS) {
    taskEXIT_CRITICAL(spinlock);
    ESP_LOGE(TAG, "No more space for RDM callbacks");
    return false;
  }
  
  // Add the requested callback to the callback list
  if (get == NULL) {
    driver->rdm.cbs[i].get.encode = NULL;
    driver->rdm.cbs[i].get.decode = NULL;
  } else {
    driver->rdm.cbs[i].get = *get;
  }
  if (set == NULL) {
    driver->rdm.cbs[i].set.encode = NULL;
    driver->rdm.cbs[i].set.decode = NULL;
  } else {
    driver->rdm.cbs[i].set = *set;
  }
  driver->rdm.cbs[i].desc = *desc;
  driver->rdm.cbs[i].cb = callback;
  driver->rdm.cbs[i].param = param;
  driver->rdm.cbs[i].num = num;
  driver->rdm.cbs[i].context = context;
  ++driver->rdm.num_callbacks;

  taskEXIT_CRITICAL(spinlock);
  return true;
}

size_t rdm_read(dmx_port_t dmx_num, rdm_header_t *header, rdm_mdb_t *mdb) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(header, false, "header is null");
  DMX_CHECK(mdb, false, "mdb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(spinlock);

  // Verify that the checksum is correct
  uint16_t sum = 0;
  uint16_t checksum;
  size_t preamble_len;
  size_t bytes_read;
  const uint8_t sc = driver->data.buffer[0];
  if (sc == RDM_SC) {
    // Calculate sum and decode checksum normally
    const size_t message_len = ((rdm_raw_t *)driver->data.buffer)->message_len;
    for (int i = 0; i < message_len; ++i) {
      sum += driver->data.buffer[i];
    }
    checksum = bswap16(*(uint16_t *)(&driver->data.buffer[message_len]));
    bytes_read = message_len + 2;
  } else {
    // Decode checksum from encoded DISC_UNIQUE_BRANCH response
    preamble_len = get_preamble_len(driver->data.buffer);
    const uint8_t *d = &driver->data.buffer[preamble_len + 1];
    for (int i = 0; i < 12; ++i) {
      sum += d[i];
    }
    checksum = (d[12] & d[13]) << 8;
    checksum += (d[14] & d[15]);
    bytes_read = preamble_len + 12;
  }
  if (sum != checksum) {
    taskEXIT_CRITICAL(spinlock);
    return 0;
  }

  // Decode the packet
  if (sc == RDM_SC) {
    const rdm_raw_t *const rdm = (void *)driver->data.buffer;

    // Assign or ignore the parameter data
    const size_t pdl = rdm->pdl;
    if (pdl > 231) {
      return false;  // PDL must be <= 231
    } else if (pdl > 0 && mdb) {
      memcpy(mdb->pd, &rdm->pd, pdl);
    }
    if (mdb) {
      mdb->pdl = pdl;
    }

    // Copy the remaining header data
    uidcpy(&header->dest_uid, &rdm->dest_uid);
    uidcpy(&header->src_uid, &rdm->src_uid);
    header->tn = rdm->tn;
    header->port_id = rdm->port_id;  // Also copies response_type
    header->message_count = rdm->message_count;
    header->sub_device = bswap16(rdm->sub_device);
    header->cc = rdm->cc;
    header->pid = bswap16(rdm->pid);

  } else {
    // Decode the EUID
    uint8_t buf[6];
    const uint8_t *d = &driver->data.buffer[preamble_len + 1];
    for (int i = 0, j = 0; i < 6; ++i, j += 2) {
      buf[i] = d[j] & d[j + 1];
    }
    uidcpy(&header->src_uid, buf);

    // Fill out the remaining header and MDB data
    header->dest_uid = RDM_UID_NULL;
    header->tn = -1;
    header->response_type = RDM_RESPONSE_TYPE_ACK;  // Also copies port_id
    header->message_count = -1;
    header->sub_device = -1;
    header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
    header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    if (mdb) {
      mdb->pdl = 0;
      mdb->preamble_len = preamble_len;
    }
  }

  taskEXIT_CRITICAL(spinlock);
  return bytes_read;
}

size_t rdm_write2(dmx_port_t dmx_num, rdm_header_t *header, uint8_t pdl,
                 const void *pd) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(pdl <= 231 && !(pd == NULL && pdl > 0), 0, "pdl is invalid");
  DMX_CHECK((header != NULL) || (pd != NULL && pdl > 0), 0,
            "header and pd are null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // if (header != NULL) {
  //   DMX_CHECK(!uid_is_null(&header->dest_uid), 0, "dest_uid is invalid");
  //   DMX_CHECK(!uid_is_broadcast(&header->src_uid), 0, "src_uid is invalid");
  //   DMX_CHECK(
  //       (header->sub_device < 513 || header->sub_device == RDM_SUB_DEVICE_ALL),
  //       0, "sub_device is invalid");
  //   DMX_CHECK((header->cc == RDM_CC_DISC_COMMAND ||
  //              header->cc == RDM_CC_DISC_COMMAND_RESPONSE ||
  //              header->cc == RDM_CC_GET_COMMAND ||
  //              header->cc == RDM_CC_GET_COMMAND_RESPONSE ||
  //              header->cc == RDM_CC_SET_COMMAND ||
  //              header->cc == RDM_CC_SET_COMMAND_RESPONSE),
  //             0, "cc is invalid");

  //   // if port_id is 0, set it to this rdm port
  //   // if src_uid is null, set it to this UID
  // }

  size_t written = 0;

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  uart_dev_t *const restrict uart = driver->uart;

  // Get pointers to driver data buffer locations and declare checksum
  uint16_t checksum = 0xcc + 0x01;
  uint8_t *header_ptr = driver->data.buffer;
  uint8_t *message_len_ptr = header_ptr + 2;
  uint8_t *pdl_ptr = header_ptr + 24;
  void *pd_ptr = header_ptr + 25;

  taskENTER_CRITICAL(spinlock);

  // Ensure driver isn't sending - RDM writes must be synchronous
  if (driver->is_sending) {
    taskEXIT_CRITICAL(spinlock);
    return written;
  } else if (dmx_uart_get_rts(uart) == 1) {
    dmx_uart_set_rts(uart, 0);  // Stop writes from being overwritten by DMX
  }

  // Copy the header, pd, message_len and pdl into the driver
  pdcpy(header_ptr, 513, "#cc01#18huubbbwbw", header, sizeof(*header), false);
  memcpy(pd_ptr, pd, pdl);
  *message_len_ptr += pdl;
  *pdl_ptr = pdl;

  // Calculate and copy the checksum
  for (int i = 2; i < *message_len_ptr; ++i) {
    checksum += header_ptr[i];
  }
  *(uint16_t *)(header_ptr + *message_len_ptr) = bswap16(checksum);

  taskEXIT_CRITICAL(spinlock);

  return written;
}

size_t rdm_write(dmx_port_t dmx_num, const rdm_header_t *header,
                 const rdm_mdb_t *mdb) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  if (mdb != NULL && !(header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
                       header->cc == RDM_CC_DISC_COMMAND_RESPONSE)) {
    if (mdb->pdl > 231) {
      ESP_LOGE(TAG, "pdl is invalid");
      return 0;
    } else if (mdb->pdl > 0 && mdb->pd == NULL) {
      ESP_LOGE(TAG, "pd is null");
      return 0;
    }
  }

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];
  uart_dev_t *const restrict uart = driver->uart;

  taskENTER_CRITICAL(spinlock);
  if (driver->is_sending) {
    // Do not allow asynchronous writes when sending an any packet
    taskEXIT_CRITICAL(spinlock);
    return 0;
  } else if (dmx_uart_get_rts(uart) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_uart_set_rts(uart, 0);
  }

  size_t encoded;
  if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
      header->cc == RDM_CC_DISC_COMMAND_RESPONSE) {
    // Encode the preamble
    const size_t preamble_len = mdb->preamble_len > 7 ? 7 : mdb->preamble_len;
    for (int i = 0; i < preamble_len; ++i) {
      driver->data.buffer[i] = RDM_PREAMBLE;
    }
    driver->data.buffer[preamble_len] = RDM_DELIMITER;

    // Encode the EUID and calculate the checksum
    // FIXME: loop?
    uint8_t *d = &(driver->data.buffer[mdb->preamble_len + 1]);
    d[0] = ((uint8_t *)&(header->src_uid.man_id))[1] | 0xaa;
    d[1] = ((uint8_t *)&(header->src_uid.man_id))[1] | 0x55;
    d[2] = ((uint8_t *)&(header->src_uid.man_id))[0] | 0xaa;
    d[3] = ((uint8_t *)&(header->src_uid.man_id))[0] | 0x55;
    d[4] = ((uint8_t *)&(header->src_uid.dev_id))[3] | 0xaa;
    d[5] = ((uint8_t *)&(header->src_uid.dev_id))[3] | 0x55;
    d[6] = ((uint8_t *)&(header->src_uid.dev_id))[2] | 0xaa;
    d[7] = ((uint8_t *)&(header->src_uid.dev_id))[2] | 0x55;
    d[8] = ((uint8_t *)&(header->src_uid.dev_id))[1] | 0xaa;
    d[9] = ((uint8_t *)&(header->src_uid.dev_id))[1] | 0x55;
    d[10] = ((uint8_t *)&(header->src_uid.dev_id))[0] | 0xaa;
    d[11] = ((uint8_t *)&(header->src_uid.dev_id))[0] | 0x55;
    
    uint16_t checksum = 0;
    for (int i = 0; i < 12; ++i) {
      checksum += d[i];
    }

    // Encode the checksum
    d[12] = (checksum >> 8) | 0xaa;
    d[13] = (checksum >> 8) | 0x55;
    d[14] = (checksum & 0xff) | 0xaa;
    d[15] = (checksum & 0xff) | 0x55;

    encoded = mdb->preamble_len + 1 + 16;
  } else {
    rdm_raw_t *const rdm = (void *)driver->data.buffer;

    // Encode the parameter data
    if (mdb != NULL) {
      if (mdb->pdl > 0) {
        memcpy(&rdm->pd, mdb->pd, mdb->pdl);
      }
      rdm->pdl = mdb->pdl;
    } else {
      rdm->pdl = 0;
    }

    // Encode the packet header
    const size_t message_len = 24 + mdb->pdl;
    rdm->sc = RDM_SC;
    rdm->sub_sc = RDM_SUB_SC;
    rdm->message_len = message_len;
    uidcpy(rdm->dest_uid, &header->dest_uid);
    uidcpy(rdm->src_uid, &header->src_uid);
    rdm->tn = header->tn;
    rdm->port_id = header->port_id;  // Also copies response_type
    rdm->message_count = header->message_count;
    rdm->sub_device = bswap16(header->sub_device);
    rdm->cc = header->cc;
    rdm->pid = bswap16(header->pid);

    // Encode the checksum
    uint16_t checksum = 0;
    for (int i = 0; i < message_len; ++i) {
      checksum += driver->data.buffer[i];
    }
    *(uint16_t *)&(driver->data.buffer)[message_len] = bswap16(checksum);

    encoded = message_len + 2;
  }

  driver->data.tx_size = encoded;  // Update driver transmit size
  
  taskEXIT_CRITICAL(spinlock);
  return encoded;
}

size_t rdm_send(dmx_port_t dmx_num, rdm_header_t *header,
                const rdm_encode_t *encode, rdm_decode_t *decode,
                rdm_ack_t *ack) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL, 0, "header is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Validate required header information
  if (uid_is_null(&header->dest_uid)) {
    ESP_LOGE(TAG, "dest_uid is invalid");
    return 0;
  }
  if (header->cc != RDM_CC_DISC_COMMAND && header->cc != RDM_CC_GET_COMMAND &&
      header->cc != RDM_CC_SET_COMMAND) {
    ESP_LOGE(TAG, "cc is invalid");
    return 0;
  }
  if (header->pid == 0 || header->pid > 0xffff) {
    ESP_LOGE(TAG, "pid is invalid");
    return 0;
  }
  if (header->sub_device > 512 && header->sub_device != RDM_SUB_DEVICE_ALL) {
    ESP_LOGE(TAG, "sub_device is invalid");
    return 0;
  } else if (header->sub_device == RDM_SUB_DEVICE_ALL &&
             header->cc == RDM_CC_GET_COMMAND) {
    ESP_LOGE(TAG, "cannot send RDM_CC_GET_COMMAND to RDM_SUB_DEVICE_ALL");
    return 0;
  }

  // Validate header values that the user doesn't need to include
  if (uid_is_broadcast(&header->src_uid)) {
    ESP_LOGE(TAG, "src_uid is invalid");
    return 0;
  } else if (uid_is_null(&header->src_uid)) {
    rdm_driver_get_uid(dmx_num, &header->src_uid);
  }
  if (header->port_id < 0 || header->port_id > 255) {
    ESP_LOGE(TAG, "port_id is invalid");
    return 0;
  } else if (header->port_id == 0) {
    header->port_id = dmx_num + 1;
  }

  spinlock_t *const restrict spinlock = &dmx_spinlock[dmx_num];
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Set header values that the user cannot set themselves
  taskENTER_CRITICAL(spinlock);
  header->tn = driver->rdm.tn;
  taskEXIT_CRITICAL(spinlock);
  header->message_count = 0;

  // Encode parameter data
  rdm_mdb_t mdb;
  if (encode && encode->function && encode->params && encode->num) {
    encode->function(&mdb, encode->params, encode->num);
  } else {
    mdb.pdl = 0;
  }

  // Take mutex so driver values may be accessed
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);
  dmx_wait_sent(dmx_num, portMAX_DELAY);

  // Send the request and await the response
  const size_t write_size = rdm_write(dmx_num, header, &mdb);
  dmx_send(dmx_num, write_size);
  dmx_packet_t packet = {};  // Initialize values to 0
  if (!uid_is_broadcast(&header->dest_uid) ||
      (header->pid == RDM_PID_DISC_UNIQUE_BRANCH &&
       header->cc == RDM_CC_DISC_COMMAND)) {
    dmx_receive(dmx_num, &packet, 2);
  }

  // Process the response data
  if (packet.size > 0) {
    uint32_t decoded = 0;
    const rdm_header_t req = *header;
    rdm_response_type_t response_type;
    if (packet.err || !rdm_read(dmx_num, header, &mdb)) {
      response_type = RDM_RESPONSE_TYPE_INVALID;  // Data or checksum error
    } else {
      // Decode the response type
      response_type = header->response_type;
      if (response_type < RDM_RESPONSE_TYPE_ACK ||
          response_type > RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
        response_type = RDM_RESPONSE_TYPE_INVALID;  // Invalid response type
      } else if (!(req.cc == RDM_CC_DISC_COMMAND &&
                   req.pid == RDM_PID_DISC_UNIQUE_BRANCH) &&
                 (req.cc != (header->cc - 1) || req.pid != header->pid ||
                  req.tn != header->tn ||
                  !uid_is_eq(&req.src_uid, &header->dest_uid) ||
                  !uid_is_eq(&req.dest_uid, &header->src_uid))) {
        response_type = RDM_RESPONSE_TYPE_INVALID;  // Invalid packet format
      }

      // Handle the response based on the response type
      if (response_type == RDM_RESPONSE_TYPE_ACK) {
        // Decode the MDB if there is parameter data
        if (mdb.pdl > 0) {
          if (decode && decode->function && decode->params && decode->num) {
            decoded = decode->function(&mdb, decode->params, decode->num);
          } else {
            ESP_LOGW(TAG, "received parameter data but decoder is null");
          }
        }
      } else if (response_type == RDM_RESPONSE_TYPE_ACK_TIMER) {
        // Get the estimated response time and convert it to FreeRTOS ticks
        rdm_decode_16bit(&mdb, &decoded, 1);
        decoded = pdMS_TO_TICKS(decoded * 10);
      } else if (response_type == RDM_RESPONSE_TYPE_NACK_REASON) {
        // Get the reported NACK reason
        rdm_decode_16bit(&mdb, &decoded, 1);
      } else if (response_type == RDM_RESPONSE_TYPE_ACK_OVERFLOW) {
        // Received RDM_RESPONSE_TYPE_ACK_OVERFLOW
        packet.err = ESP_ERR_NOT_SUPPORTED;  // TODO: implement overflow support
      }
    }

    // Report the ACK back to the user
    if (ack != NULL) {
      ack->err = packet.err;
      ack->type = response_type;
      ack->num = decoded;
    }
  } else {
    // Wait for request to finish sending if no response is expected
    if (ack != NULL) {
      ack->err = packet.err;
      ack->type = RDM_RESPONSE_TYPE_NONE;
      ack->num = 0;
    }
    dmx_wait_sent(dmx_num, 2);
  }

  xSemaphoreGiveRecursive(driver->mux);
  return packet.size;
}

static int rdm_disc_unique_branch_cb(dmx_port_t dmx_num,
                                     const rdm_header_t *header,
                                     rdm_encode_decode_t *functions,
                                     rdm_mdb_t *mdb, void *param, int num,
                                     void *context) {
  // Ignore this message if discovery is muted
  if (rdm_driver_is_muted(dmx_num)) {
    return RDM_RESPONSE_TYPE_NONE;
  }

  // Decode the two UIDs
  // TODO: decode directly into the mdb array and return a pointer to the
  // decoded MDB
  rdm_disc_unique_branch_t branch;
  functions->decode(mdb, &branch, 2);

  // Respond if the device UID is between the branch bounds
  rdm_uid_t my_uid;
  rdm_driver_get_uid(dmx_num, &my_uid);
  if (uid_is_ge(&my_uid, &branch.lower_bound) &&
      uid_is_le(&my_uid, &branch.upper_bound)) {
    mdb->preamble_len = 7;
    return RDM_RESPONSE_TYPE_ACK;
  } else {
    return RDM_RESPONSE_TYPE_NONE;
  }
}

bool rdm_register_disc_unique_branch(dmx_port_t dmx_num, void *context) {
  // TODO: arg check

  const rdm_pid_description_t desc = {
      .pid = RDM_PID_DISC_UNIQUE_BRANCH, .pdl_size = 12, .cc = RDM_CC_DISC};
  const rdm_encode_decode_t disc = {.decode = rdm_decode_uids};

  return rdm_register_callback(dmx_num, &desc, &disc, NULL,
                               rdm_disc_unique_branch_cb, NULL, 0, context);
}

static int rdm_disc_mute_cb(dmx_port_t dmx_num, const rdm_header_t *header,
                            rdm_encode_decode_t *functions, rdm_mdb_t *mdb,
                            void *param, int num, void *context) {
  // Mute or un-mute the discovery
  dmx_driver[dmx_num]->rdm.discovery_is_muted =
      (header->pid == RDM_PID_DISC_MUTE);

  // Encode the response
  const rdm_disc_mute_t mute = {
      // TODO: get control field
      // TODO: get binding UID
  };
  functions->encode(mdb, &mute, 1);

  // Return an ACK
  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_disc_mute(dmx_port_t dmx_num, void *context) {
  // TODO: arg check

  const rdm_pid_description_t desc = {
      .pid = RDM_PID_DISC_MUTE, .pdl_size = 0, .cc = RDM_CC_DISC};
  const rdm_encode_decode_t disc = {.encode = rdm_encode_mute};

  return rdm_register_callback(dmx_num, &desc, &disc, NULL, rdm_disc_mute_cb,
                               NULL, 0, context);
}

bool rdm_register_disc_un_mute(dmx_port_t dmx_num, void *context) {
  // TODO: arg check

  const rdm_pid_description_t desc = {
      .pid = RDM_PID_DISC_UN_MUTE, .pdl_size = 0, .cc = RDM_CC_DISC};
  const rdm_encode_decode_t disc = {.encode = rdm_encode_mute};

  return rdm_register_callback(dmx_num, &desc, &disc, NULL, rdm_disc_mute_cb,
                               NULL, 0, context);
}

static int rdm_simple_param_cb(dmx_port_t dmx_num, const rdm_header_t *header,
                               rdm_encode_decode_t *functions, rdm_mdb_t *mdb,
                               void *param, int num, void *context) {
  if (functions->decode != NULL) {
    functions->decode(mdb, param, num);
  }
  if (functions->encode != NULL) {
    functions->encode(mdb, param, num);
  }
  return RDM_RESPONSE_TYPE_ACK;
}

bool rdm_register_device_info(dmx_port_t dmx_num,
                              rdm_device_info_t *device_info) {
  // TODO: arg check

  const rdm_pid_description_t desc = {
      .pid = RDM_PID_DEVICE_INFO, .pdl_size = 0, .cc = RDM_CC_GET};
  const rdm_encode_decode_t get = {.encode = rdm_encode_device_info};

  return rdm_register_callback(dmx_num, &desc, &get, NULL, rdm_simple_param_cb,
                               device_info, 1, NULL);
}

bool rdm_register_software_version_label(dmx_port_t dmx_num,
                                         const char *software_version_label) {
  // TODO: arg check

  const rdm_pid_description_t desc = {.pid = RDM_PID_SOFTWARE_VERSION_LABEL,
                                      .pdl_size = 0,
                                      .cc = RDM_CC_GET};
  const rdm_encode_decode_t get = {.encode = rdm_encode_string};

  size_t len = strlen(software_version_label);
  if (len > 32) {
    len = 32;
  }

  return rdm_register_callback(dmx_num, &desc, &get, NULL,
                               rdm_simple_param_cb,
                               (void *)software_version_label, len, NULL);
}

bool rdm_register_identify_device(dmx_port_t dmx_num) {
  // TODO

  return false;
}

bool rdm_register_dmx_start_address(dmx_port_t dmx_num,
                                    uint16_t *dmx_start_address) {
  // TODO: arg check

  const rdm_pid_description_t desc = {.pid = RDM_PID_DMX_START_ADDRESS,
                                      .pdl_size = 2,
                                      .cc = RDM_CC_GET_SET,
                                      .max_value = 512,
                                      .min_value = 1};
  const rdm_encode_decode_t get = {.encode = rdm_encode_16bit};
  const rdm_encode_decode_t set = {.decode = rdm_decode_16bit,
                                   .encode = rdm_encode_null};

  return rdm_register_callback(dmx_num, &desc, &get, &set,
                               rdm_simple_param_cb, dmx_start_address, 1,
                               NULL);
}