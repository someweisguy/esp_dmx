#include "rdm_tools.h"

#include <stdint.h>
#include <string.h>

#include "dmx_constants.h"
#include "endian.h"
#include "esp_log.h"
#include "esp_system.h"
#include "impl/driver.h"
#include "rdm_constants.h"

static const char *TAG = "rdm";

static uint64_t rdm_uid = 0;  // The 48-bit unique ID of this device.

IRAM_ATTR uint64_t buf_to_uid(const void *buf) {
  uint64_t val = 0;
  ((uint8_t *)&val)[5] = ((uint8_t *)buf)[0];
  ((uint8_t *)&val)[4] = ((uint8_t *)buf)[1];
  ((uint8_t *)&val)[3] = ((uint8_t *)buf)[2];
  ((uint8_t *)&val)[2] = ((uint8_t *)buf)[3];
  ((uint8_t *)&val)[1] = ((uint8_t *)buf)[4];
  ((uint8_t *)&val)[0] = ((uint8_t *)buf)[5];
  return val;
}

void *uid_to_buf(void *buf, uint64_t uid) {
  uid = bswap64(uid) >> 16;
  return memcpy(buf, &uid, 6);
}

uint64_t rdm_get_uid() { 
  // Initialize the RDM UID
  if (rdm_uid == 0) {
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    ((rdm_uid_t *)(&rdm_uid))->manufacturer_id = RDM_DEFAULT_MANUFACTURER_ID;
    ((rdm_uid_t *)(&rdm_uid))->device_id = bswap32(*(uint32_t *)(mac + 2));
  }

  return rdm_uid;
}

void rdm_set_uid(uint64_t uid) { 
  rdm_uid = uid;
}

bool rdm_parse(void *data, size_t size, rdm_event_t *event) {
  // TODO: check args

  const rdm_data_t *const rdm = (rdm_data_t *)data;
  
  if ((rdm->sc == RDM_PREAMBLE || rdm->sc == RDM_DELIMITER) && size > 17) {
    // Find the length of the discovery response preamble (0-7 bytes)
    int preamble_len = 0;
    const uint8_t *response = data;
    for (; preamble_len < 7; ++preamble_len) {
      if (response[preamble_len] == RDM_DELIMITER) {
        break;
      }
    }
    if (response[preamble_len] != RDM_DELIMITER || size < preamble_len + 17) {
      return false;  // Not a valid discovery response
    }

    // Decode the 6-byte UID and get the packet sum
    uint64_t uid = 0;
    uint16_t sum = 0;
    response = &((uint8_t *)data)[preamble_len + 1];
    for (int i = 5, j = 0; i >= 0; --i, j += 2) {
      ((uint8_t *)&uid)[i] = response[j] & 0x55;
      ((uint8_t *)&uid)[i] |= response[j + 1] & 0xaa;
      sum += ((uint8_t *)&uid)[i] + 0xff;
    }

    // Decode the checksum received in the response
    uint16_t checksum;
    for (int i = 1, j = 12; i >= 0; --i, j += 2) {
      ((uint8_t *)&checksum)[i] = response[j] & 0x55;
      ((uint8_t *)&checksum)[i] |= response[j + 1] & 0xaa;
    }

    // Return DMX data to the caller
    event->cc = RDM_DISCOVERY_COMMAND_RESPONSE;
    event->pid = RDM_PID_DISC_UNIQUE_BRANCH;
    event->source_uid = uid;
    event->checksum_is_valid = (sum == checksum);
    return true;

  } else if (rdm->sc == RDM_SC && rdm->sub_sc == RDM_SUB_SC) {
    // Verify the packet checksum
    uint16_t sum = 0;
    for (int i = 0; i < rdm->message_len; ++i) {
      sum += ((uint8_t *)data)[i];
    }
    const uint16_t checksum = bswap16(*(uint16_t *)(data + rdm->message_len));
    event->checksum_is_valid = (sum == checksum);

    // Copy the packet data to the event if the checksum is valid
    event->destination_uid = buf_to_uid(rdm->destination_uid);
    event->source_uid = buf_to_uid(rdm->source_uid);
    event->tn = rdm->tn;
    event->port_id = rdm->port_id;  // Also copies response_type
    event->message_count = rdm->message_count;
    event->sub_device = bswap16(rdm->sub_device);
    event->cc = rdm->cc;
    event->pid = bswap16(rdm->pid);
    event->pdl = rdm->pdl;
    return true;
  }

  return false;
}

bool rdm_write_discovery_response(dmx_port_t dmx_num) {
  // TODO: check args

  // TODO: return true if a response is sent, false if the driver is muted

  // Build the discovery response packet
  uint8_t response[24] = {RDM_PREAMBLE, RDM_PREAMBLE, RDM_PREAMBLE,
                          RDM_PREAMBLE, RDM_PREAMBLE, RDM_PREAMBLE,
                          RDM_PREAMBLE, RDM_DELIMITER};
  const uint64_t uid = rdm_get_uid();
  uint16_t checksum = 0;
  for (int i = 8, j = 5; i < 20; i += 2, --j) {
    response[i] = ((uint8_t *)&uid)[j] | 0xaa;
    response[i + 1] = ((uint8_t *)&uid)[j] | 0x55;
    checksum += ((uint8_t *)&uid)[j] + (0xaa + 0x55);
  }
  response[20] = (checksum >> 8) | 0xaa;
  response[21] = (checksum >> 8) | 0x55;
  response[22] = checksum | 0xaa;
  response[23] = checksum | 0x55;

  // Write the response
  return dmx_write(dmx_num, response, sizeof(response));
}

bool rdm_send_discovery_unmute(dmx_port_t dmx_num, uint64_t uid,
                               rdm_event_t *event, size_t *num_params,
                               rdm_disc_mute_response_t *params) {
  // TODO: check args
  
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Block so driver values may be accessed
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Write the RDM mute command packet
  uint8_t command[26];
  rdm_data_t *const rdm = (rdm_data_t *)command;
  rdm->sc = RDM_SC;
  rdm->sub_sc = RDM_SUB_SC;
  rdm->message_len = sizeof(command) - 2;
  uid_to_buf(rdm->destination_uid, uid);
  uid_to_buf(rdm->source_uid, rdm_get_uid());
  rdm->tn = 0; // TODO: Driver can track transaction num
  rdm ->port_id = dmx_num + 1;
  rdm->message_count = 0;
  rdm->sub_device = bswap16(0);
  rdm->cc = RDM_DISCOVERY_COMMAND;
  rdm->pid = bswap16(RDM_PID_DISC_UN_MUTE);
  rdm->pdl = 0;

  // Calculate and write the checksum
  uint16_t checksum = 0;
  for (int i = 0; i < rdm->message_len; ++i) {
    checksum += command[i];
  }
  *(uint16_t *)(&command[rdm->message_len]) = bswap16(checksum);

  // Wait, write, and send the command
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, command, sizeof(command));
  dmx_send(dmx_num, sizeof(command));

  // Determine if a response is expected
  dmx_event_t dmx_event;
  size_t ack_size = 0;
  const bool expected_reply = (uid != RDM_BROADCAST_UID);
  if (expected_reply) {
    ack_size = dmx_receive(dmx_num, &dmx_event, DMX_TIMEOUT_TICK);
  } else {
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  }
  xSemaphoreGiveRecursive(driver->mux);

  if (ack_size > 0) {
    // Ensure the received packet is valid

    if (dmx_event.err || !dmx_event.is_rdm) {
      return false;  // Invalid response
    } else if (!dmx_event.rdm.checksum_is_valid) {
      return false;  // Checksum is invalid
    } else if (dmx_event.rdm.cc != RDM_DISCOVERY_COMMAND_RESPONSE ||
               dmx_event.rdm.pid != RDM_PID_DISC_UN_MUTE || 
               dmx_event.rdm.response_type != RDM_RESPONSE_TYPE_ACK) {
      // Mute response type must only be RDM_RESPONSE_TYPE_ACK
      return false;  // Invalid response
    } else if (dmx_event.rdm.source_uid != uid ||
               dmx_event.rdm.destination_uid != rdm_get_uid()) {
      return false;  // Invalid response
    }
    
    // Read the data into a buffer
    // uint8_t ack[26 + 14];
    // dmx_read(dmx_num, ack, sizeof(ack));

    // Copy RDM message data block and parameters
    // TODO: 


    return true;
  }

  return false;
}

bool rdm_send_discovery_mute(dmx_port_t dmx_num, uint64_t uid,
                             rdm_event_t *event, int *control_field,
                             uint64_t *binding_uid) {
  // FIXME: bool rdm_func(dmx_num, uid, *rdm_event_t, *num_params, *params);
  // TODO: check args
  
  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Block so driver values may be accessed
  xSemaphoreTakeRecursive(driver->mux, portMAX_DELAY);

  // Write the RDM mute command packet
  uint8_t command[26];
  rdm_data_t *const rdm = (rdm_data_t *)command;
  rdm->sc = RDM_SC;
  rdm->sub_sc = RDM_SUB_SC;
  rdm->message_len = sizeof(command) - 2;
  uid_to_buf(rdm->destination_uid, uid);
  uid_to_buf(rdm->source_uid, rdm_get_uid());
  rdm->tn = 0; // TODO: Driver can track transaction num
  rdm ->port_id = dmx_num + 1;
  rdm->message_count = 0;
  rdm->sub_device = bswap16(0);
  rdm->cc = RDM_DISCOVERY_COMMAND;
  rdm->pid = bswap16(RDM_PID_DISC_MUTE);
  rdm->pdl = 0;

  // Calculate and write the checksum
  uint16_t checksum = 0;
  for (int i = 0; i < rdm->message_len; ++i) {
    checksum += command[i];
  }
  *(uint16_t *)(&command[rdm->message_len]) = bswap16(checksum);

  // Wait, write, and send the command
  dmx_wait_sent(dmx_num, portMAX_DELAY);
  dmx_write(dmx_num, command, sizeof(command));
  dmx_send(dmx_num, sizeof(command));

  // Determine if a response is expected
  dmx_event_t dmx_event;
  size_t ack_size = 0;
  const bool expected_reply = (uid != RDM_BROADCAST_UID);
  if (expected_reply) {
    ack_size = dmx_receive(dmx_num, &dmx_event, DMX_TIMEOUT_TICK);
  } else {
    dmx_wait_sent(dmx_num, pdMS_TO_TICKS(30));
  }
  xSemaphoreGiveRecursive(driver->mux);

  if (ack_size > 0) {
    // Copy the RDM message data block
    if (event != NULL) {
      memcpy(event, &dmx_event.rdm, sizeof(rdm_event_t));
    }

    // Ensure the received packet is valid
    if (dmx_event.err || !dmx_event.is_rdm) {
      return false;  // Invalid response
    } else if (!dmx_event.rdm.checksum_is_valid) {
      return false;  // Checksum is invalid
    } else if (dmx_event.rdm.cc != RDM_DISCOVERY_COMMAND_RESPONSE ||
               dmx_event.rdm.pid != RDM_PID_DISC_MUTE || 
               dmx_event.rdm.response_type != RDM_RESPONSE_TYPE_ACK) {
      // Mute response type must only be RDM_RESPONSE_TYPE_ACK
      return false;  // Invalid response
    } else if (dmx_event.rdm.source_uid != uid ||
               dmx_event.rdm.destination_uid != rdm_get_uid()) {
      return false;  // Invalid response
    }
    
    // Read the data into a buffer
    uint8_t ack[26 + 14];
    dmx_read(dmx_num, ack, sizeof(ack));
    rdm_data_t *const rdm = (rdm_data_t *)ack;

    // Copy RDM packet parameters
    if (control_field != NULL) {
      uint16_t *pd = &rdm->pd;
      *control_field = dmx_event.rdm.pdl >= 2 ? bswap16(*pd) : 0;
    }
    if (binding_uid != NULL) {
      uint8_t *pd = (&rdm->pd) + 2;
      *binding_uid = dmx_event.rdm.pdl >= 8 ? buf_to_uid(pd) : 0;
    }

    return true;
  }

  return false;
}