#include "read_write.h"

#include <stdint.h>

#include "dmx/driver.h"
#include "dmx/hal.h"
#include "esp_dmx.h"
#include "rdm/utils.h"

static const char *TAG = "rdm_read_write";

extern dmx_driver_t *dmx_driver[DMX_NUM_MAX];
extern spinlock_t dmx_spinlock[DMX_NUM_MAX];

/**
 * @brief A packed struct which can be used to help process raw RDM packets
 * instead of reading slots by index alone. RDM sends data in most-significant
 * byte first, so endianness must be swapped when using values larger than 8
 * bits.
 */
typedef struct __attribute__((__packed__)) rdm_data_t {
  uint8_t sc;                  // This field shall contain the defined RDM start code. Controllers and responders shall always send RDM_SC in this slot.
  uint8_t sub_sc;              // This field shall contain the sub-start code within RDM that defines this packet structure. Unless specified in future version, the sub-start code shall be equal to RDM_SUB_SC.
  uint8_t message_len;         // The message length value is defined as the number of slots int he RDM packet including the start code and excluding the checksum.
  uint8_t destination_uid[6];  // The UID of the target device(s).
  uint8_t source_uid[6];       // The UID of the device originating this packet.
  uint8_t tn;                  // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
  union {
    uint8_t port_id;           // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
    uint8_t response_type;     // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
  };
  uint8_t message_count;       // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
  uint16_t sub_device;         // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
  uint8_t cc;                  // The command class (CC) specifies the action of the message. 
  uint16_t pid;                // The parameter ID (PID) identifies a specific type of parameter data.
  uint8_t pdl;                 // The parameter data length (PDL) is the number of slots included in the parameter data area that it precedes.
  struct {
  } pd;                        // The parameter data (PD) is of variable length. The content format is PID dependent.
} rdm_data_t;

/**
 * @brief A packed struct which can be used to help process RDM discovery mute
 * parameters. RDM sends data in most-significant byte first, so endianness must
 * be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_disc_mute_data_t {
  union {
    struct {
      uint8_t managed_proxy : 1;
      uint8_t sub_device : 1;
      uint8_t boot_loader : 1;
      uint8_t proxied_device : 1;
    };
    uint16_t control_field;
  };
  uint8_t binding_uid[6];
} rdm_disc_mute_data_t;

/**
 * @brief A packed struct which can be used to help process RDM device info
 * parameters. RDM sends data in most-significant byte first, so endianness must
 * be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_device_info_data_t {
  uint8_t major_rdm_version;
  uint8_t minor_rdm_version;
  uint16_t model_id;
  uint8_t coarse_product_category;
  uint8_t fine_product_category;
  uint32_t software_version_id;
  uint16_t footprint;
  uint8_t current_personality;
  uint8_t personality_count;
  uint16_t start_address;
  uint16_t sub_device_count;
  uint8_t sensor_count;
} rdm_device_info_data_t;

bool rdm_read(dmx_port_t dmx_num, rdm_header_t *header, rdm_mdb_t *mdb) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(header, false, "header is null");
  DMX_CHECK(mdb, false, "mdb is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // FIXME: make this function thread-safe

  // Verify that the checksum is correct
  uint16_t sum = 0;
  uint16_t checksum;
  size_t preamble_len;
  const uint8_t sc = driver->data.buffer[0];
  if (sc == RDM_SC) {
    // Calculate sum and decode checksum normally
    const size_t message_len = ((rdm_data_t *)driver->data.buffer)->message_len;
    for (int i = 0; i < message_len; ++i) {
      sum += driver->data.buffer[i];
    }
    checksum = bswap16(*(uint16_t *)(&driver->data.buffer[message_len]));
  } else {
    // Decode checksum from encoded DISC_UNIQUE_BRANCH response
    preamble_len = get_preamble_len(driver->data.buffer);
    // FIXME: rdm is invalid if preamble_len is >7
    const uint8_t *d = &driver->data.buffer[preamble_len + 1];
    for (int i = 0; i < 12; ++i) {
      sum += d[i];
    }
    checksum = (d[14] & 0x55) | (d[15] & 0xaa);
    checksum |= ((d[12] & 0x55) | (d[13] & 0xaa)) << 8;
  }
  bool checksum_is_valid = (sum == checksum);
  if (!checksum_is_valid) {
    return checksum_is_valid;
  }

  // Decode the packet
  if (sc == RDM_SC) {
    const rdm_data_t *const rdm = (void *)driver->data.buffer;

    // Assign or ignore the parameter data
    const size_t pdl = rdm->pdl;
    if (pdl > 231) {
      return false;  // PDL must be <= 231
    } else if (pdl > 0) {
      memcpy(mdb->pd, &rdm->pd, pdl);
    }
    mdb->pdl = pdl;

    // Copy the remaining header data
    header->dest_uid = bswap48(rdm->destination_uid);
    header->src_uid = bswap48(rdm->source_uid);
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
      buf[i] = (d[j] & 0x55) | (d[j + 1] & 0xaa);  // TODO: & each byte
    }
    header->src_uid = bswap48(buf);

    // Fill out the remaining header and MDB data
    header->dest_uid = 0;
    header->tn = -1;
    header->response_type = RDM_RESPONSE_TYPE_ACK;
    header->port_id = -1;
    header->message_count = -1;
    header->sub_device = -1;
    header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
    header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    mdb->pdl = 0;
    mdb->preamble_len = preamble_len;
  }

  return checksum_is_valid;
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
  taskEXIT_CRITICAL(spinlock);

  // TODO: make this function and the other `_write_` functions thread-safe

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
    uint16_t checksum = 0;
    uint8_t *d = &(driver->data.buffer[mdb->preamble_len + 1]);
    for (int i = 0, j = 5; i < 12; i += 2, --j) {
      d[i] = ((uint8_t *)&(header->src_uid))[j] | 0xaa;
      d[i + 1] = ((uint8_t *)&(header->src_uid))[j] | 0x55;
      checksum += ((uint8_t *)&(header->src_uid))[j] + 0xaa + 0x55;
    }

    // Encode the checksum
    d[12] = (checksum >> 8) | 0xaa;
    d[13] = (checksum >> 8) | 0x55;
    d[14] = (checksum & 0xff) | 0xaa;
    d[15] = (checksum & 0xff) | 0x55;

    encoded = mdb->preamble_len + 1 + 16;
  } else {
    rdm_data_t *const rdm = (void *)driver->data.buffer;

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
    uidcpy(rdm->destination_uid, &(header->dest_uid));
    uidcpy(rdm->source_uid, &(header->src_uid));
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

  taskENTER_CRITICAL(spinlock);
  driver->data.tx_size = encoded;  // Update driver transmit size
  taskEXIT_CRITICAL(spinlock);

  return encoded;
}

size_t rdm_encode_16bit(void *pd, const void *data, int size) {
  size_t pdl = 0;
  if (data != NULL) {
    uint16_t *const restrict ptr = pd;
    const uint32_t *const restrict params = data;
    for (int i = 0; i < size; ++i) {
      ptr[i] = bswap16(params[i]);
    }
    pdl = size * sizeof(uint16_t);
  }
  return pdl;
}

int rdm_decode_string(const void *pd, void *data, int size) {
  int decoded = 0;
  if (data != NULL) {
    char *restrict string = data;
    memcpy(string, pd, size);
    string[size] = 0;
    decoded = size + 1;
  }
  return decoded;
}

int rdm_decode_device_info(const void *pd, void *data, int size) {
  int decoded = 0;
  if (data != NULL) {
    const rdm_device_info_data_t *restrict ptr = pd;
    rdm_device_info_t *const restrict device_info = data;
    device_info->model_id = bswap16(ptr->model_id);
    device_info->coarse_product_category = ptr->coarse_product_category;
    device_info->fine_product_category = ptr->fine_product_category;
    device_info->software_version_id = bswap32(ptr->software_version_id);
    device_info->footprint = bswap16(ptr->footprint);
    device_info->current_personality = ptr->current_personality;
    device_info->personality_count = ptr->personality_count;
    device_info->start_address =
        ptr->start_address != 0xffff ? bswap16(ptr->start_address) : -1;
    device_info->sub_device_count = bswap16(ptr->sub_device_count);
    device_info->sensor_count = ptr->sensor_count;
    decoded = 1;
  }
  return decoded;
}

size_t rdm_encode_nack_reason(rdm_mdb_t *mdb, rdm_nr_t nack_reason) {
  const size_t encoded = rdm_encode_16bit(mdb->pd, &nack_reason, 1);
  mdb->pdl = encoded;
  return encoded;
}

int rdm_decode_uids(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb != NULL && mdb->pd != NULL && data != NULL) {
    for (int i = 0; decoded < num && i < mdb->pdl; i += 6) {
        ((rdm_uid_t *)data)[decoded] = bswap48(mdb->pd + i);
        ++decoded;
    }
  }
  return decoded;
}

size_t rdm_encode_mute(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data && num) {
    struct rdm_disc_mute_data_t *pd = (void *)mdb->pd;
    const rdm_disc_mute_t *param = data;
    bzero(mdb->pd, 2);  // FIXME: make the bit field more efficient?
    pd->managed_proxy = param->managed_proxy;
    pd->sub_device = param->sub_device;
    pd->boot_loader = param->boot_loader;
    pd->proxied_device = param->proxied_device;
    if (param->binding_uid != 0) {
      uidcpy(pd->binding_uid, &(param->binding_uid));
      encoded += 6;
    }
    encoded += 2;
  }
  mdb->pdl = encoded;
  return encoded;
}

size_t rdm_encode_device_info(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data) {
    rdm_device_info_data_t *const pd = (void *)mdb->pd;
    const rdm_device_info_t *param = data;
    pd->major_rdm_version = 1;
    pd->minor_rdm_version = 0;
    pd->model_id = bswap16(param->model_id);
    pd->coarse_product_category = param->coarse_product_category;
    pd->fine_product_category = param->fine_product_category;
    pd->software_version_id = bswap32(param->software_version_id);
    pd->footprint = bswap16(param->footprint);
    pd->current_personality = param->current_personality;
    pd->start_address =
        param->start_address != -1 ? bswap16(param->start_address) : 0xffff;
    pd->sub_device_count = bswap16(param->sub_device_count);
    pd->sensor_count = param->sensor_count;
    encoded = sizeof(rdm_device_info_data_t);
  }
  mdb->pdl = encoded;
  return encoded;
}

size_t rdm_encode_string(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data) {
    char *dest = (void *)mdb->pd;
    const char *src = data;
    while (encoded < num && encoded < 32) {
      if (*src) {
        *dest = *src;
        ++encoded;
        ++dest;
        ++src;
      } else {
        break;  // Don't encode null terminators
      }
    }
  }
  mdb->pdl = encoded;
  return encoded;
}

size_t rdm_encode_8bit(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data) {
    uint8_t *pd = mdb->pd;
    const uint8_t *param = data;
    for (int i = 0; i < num; ++i) {
      // FIXME: ensure that the number of encoded bytes never exceeds 231
      pd[i] = param[i];
    }
    encoded = num;
  }
  mdb->pdl = encoded;
  return encoded;
}

int rdm_decode_8bit(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const uint8_t *pd = mdb->pd;
    uint8_t *param = data;
    for (int i = 0; i < num; ++i) {
      param[i] = pd[i];
    }
    decoded = num;
  }
  return decoded;
}

int rdm_decode_16bit(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const uint16_t *pd = (void *)mdb->pd;
    uint16_t *params = data;
    if (num > 231 / sizeof(uint16_t)) {
      num = 231 / sizeof(uint16_t);
    }
    for (int i = 0, j = 0; i < num && j < mdb->pdl;
         ++i, j += sizeof(uint16_t)) {
      params[i] = bswap16(pd[i]);
    }
  }
  return decoded;
}

int rdm_decode_mute(const rdm_mdb_t *mdb, void *data, int num) {
  int decoded = 0;
  if (mdb && mdb->pdl && data) {
    const struct rdm_disc_mute_data_t *const pd = (void *)mdb->pd;
    rdm_disc_mute_t *param = data;
    param->managed_proxy = pd->managed_proxy;
    param->sub_device = pd->sub_device;
    param->boot_loader = pd->boot_loader;
    param->proxied_device = pd->proxied_device;
    param->binding_uid = mdb->pdl > 2 ? bswap48(pd->binding_uid) : 0;
    decoded = 1;
  }
  return decoded;
}

size_t rdm_encode_uids(rdm_mdb_t *mdb, const void *data, int num) {
  size_t encoded = 0;
  if (mdb && data && num) {
    for (int i = 0; i < num; ++i, encoded += 6) {
      uidcpy(mdb->pd + encoded, &(((rdm_uid_t *)data)[i]));
    }
  }
  mdb->pdl = encoded;
  return encoded;
}