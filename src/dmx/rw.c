#include "dmx/rw.h"

#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"
#include "endian.h"

size_t dmx_read_offset(dmx_port_t dmx_num, size_t offset, void *destination,
                       size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(offset < DMX_PACKET_SIZE_MAX, 0, "offset error");
  DMX_CHECK(destination, 0, "destination is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp size to the maximum DMX packet size
  if (size + offset > DMX_PACKET_SIZE_MAX) {
    size = DMX_PACKET_SIZE_MAX - offset;
  } else if (size == 0) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Copy data from the driver buffer to the destination asynchronously
  memcpy(destination, driver->data + offset, size);

  return size;
}

size_t dmx_read(dmx_port_t dmx_num, void *destination, size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(destination, 0, "destination is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_read_offset(dmx_num, 0, destination, size);
}

int dmx_read_slot(dmx_port_t dmx_num, size_t slot_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, -1, "dmx_num error");
  DMX_CHECK(slot_num < DMX_PACKET_SIZE_MAX, -1, "slot_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), -1, "driver is not installed");

  uint8_t slot;
  dmx_read_offset(dmx_num, slot_num, &slot, 1);

  return slot;
}

size_t DMX_ISR_ATTR dmx_read_rdm(dmx_port_t dmx_num, rdm_header_t *header,
                                 void *pd, size_t num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t read = 0;

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get pointers to driver data buffer locations and declare checksum
  uint16_t checksum = 0;
  const uint8_t *header_ptr = driver->data;
  const void *pd_ptr = header_ptr + 24;

  // Verify start code and sub-start code are correct
  if (*(uint16_t *)header_ptr != (RDM_SC | (RDM_SUB_SC << 8)) &&
      *header_ptr != RDM_PREAMBLE && *header_ptr != RDM_DELIMITER) {
    return read;
  }

  // Get and verify the preamble_len if packet is a discovery response
  size_t preamble_len = 0;
  if (*header_ptr == RDM_PREAMBLE || *header_ptr == RDM_DELIMITER) {
    for (; preamble_len <= 7; ++preamble_len) {
      if (header_ptr[preamble_len] == RDM_DELIMITER) break;
    }
    if (preamble_len > 7) {
      return read;
    }
  }

  // Handle packets differently if a DISC_UNIQUE_BRANCH packet was received
  if (*header_ptr == RDM_SC) {
    // Verify checksum is correct
    const uint8_t message_len = header_ptr[2];
    for (int i = 0; i < message_len; ++i) {
      checksum += header_ptr[i];
    }
    if (checksum != bswap16(*(uint16_t *)(header_ptr + message_len))) {
      return read;
    }

    // Copy the header and pd from the driver
    if (header != NULL) {
      // Copy header without emplace so this function can be used in IRAM ISR
      for (int i = 0; i < sizeof(rdm_header_t); ++i) {
        ((uint8_t *)header)[i] = header_ptr[i];
      }
      header->dest_uid.man_id = bswap16(header->dest_uid.man_id);
      header->dest_uid.dev_id = bswap32(header->dest_uid.dev_id);
      header->src_uid.man_id = bswap16(header->src_uid.man_id);
      header->src_uid.dev_id = bswap32(header->src_uid.dev_id);
      header->sub_device = bswap16(header->sub_device);
      header->pid = bswap16(header->pid);
    }
    if (pd != NULL) {
      const uint8_t pdl = header_ptr[23];
      const size_t copy_size = pdl < num ? pdl : num;
      memcpy(pd, pd_ptr, copy_size);
    }

    // Update the read size
    read = message_len + 2;

  } else {
    // Verify the checksum is correct
    header_ptr += preamble_len + 1;
    for (int i = 0; i < 12; ++i) {
      checksum += header_ptr[i];
    }
    if (checksum != (((header_ptr[12] & header_ptr[13]) << 8) |
                     (header_ptr[14] & header_ptr[15]))) {
      return read;
    }

    // Decode the EUID
    uint8_t buf[6];
    for (int i = 0, j = 0; i < 6; ++i, j += 2) {
      buf[i] = header_ptr[j] & header_ptr[j + 1];
    }

    // Copy the data into the header
    if (header != NULL) {
      // Copy header without emplace so this function can be used in IRAM ISR
      for (int i = 0; i < sizeof(rdm_uid_t); ++i) {
        ((uint8_t *)&header->src_uid)[i] = buf[i];
      }
      header->src_uid.man_id = bswap16(header->src_uid.man_id);
      header->src_uid.dev_id = bswap32(header->src_uid.dev_id);
      header->dest_uid = (rdm_uid_t){0, 0};
      header->tn = 0;
      header->response_type = RDM_RESPONSE_TYPE_ACK;
      header->message_count = 0;
      header->sub_device = RDM_SUB_DEVICE_ROOT;
      header->cc = RDM_CC_DISC_COMMAND_RESPONSE;
      header->pid = RDM_PID_DISC_UNIQUE_BRANCH;
      header->pdl = preamble_len + 1 + 16;
    }

    // Update the read size
    read = preamble_len + 1 + 16;
  }

  return read;
}

size_t dmx_write_offset(dmx_port_t dmx_num, size_t offset, const void *source,
                        size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(offset < DMX_PACKET_SIZE_MAX, 0, "offset error");
  DMX_CHECK(source, 0, "source is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp size to the maximum DMX packet size
  if (size + offset > DMX_PACKET_SIZE_MAX) {
    size = DMX_PACKET_SIZE_MAX - offset;
  } else if (size == 0) {
    return 0;
  }

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if ((driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) && driver->rdm_type != 0) {
    // Do not allow asynchronous writes when sending an RDM packet
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    return 0;
  } else if (dmx_uart_get_rts(driver->uart) == 1) {
    // Flip the bus to stop writes from being overwritten by incoming data
    dmx_uart_set_rts(driver->uart, 0);
  }
  driver->tx_size = offset + size;  // Update driver transmit size

  // Copy data from the source to the driver buffer asynchronously
  memcpy(driver->data + offset, source, size);

  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return size;
}

size_t dmx_write(dmx_port_t dmx_num, const void *source, size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(source, 0, "source is null");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  return dmx_write_offset(dmx_num, 0, source, size);
}

int dmx_write_slot(dmx_port_t dmx_num, size_t slot_num, uint8_t value) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, -1, "dmx_num error");
  DMX_CHECK(slot_num < DMX_PACKET_SIZE_MAX, -1, "slot_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), -1, "driver is not installed");

  dmx_write_offset(dmx_num, slot_num, &value, 1);

  return value;
}

size_t dmx_write_rdm(dmx_port_t dmx_num, rdm_header_t *header, const void *pd) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(header != NULL || pd != NULL, 0,
            "header is null and pd does not contain a UID");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  size_t written = 0;

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Get pointers to driver data buffer locations and declare checksum
  uint16_t checksum = 0;
  uint8_t *header_ptr = driver->data;
  void *pd_ptr = header_ptr + 24;

  // RDM writes must be synchronous to prevent data corruption
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) {
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    return written;
  } else if (dmx_uart_get_rts(driver->uart) == 1) {
    dmx_uart_set_rts(driver->uart, 0);  // Stops writes from being overwritten
  }

  if (header != NULL && !(header->cc == RDM_CC_DISC_COMMAND_RESPONSE &&
                          header->pid == RDM_PID_DISC_UNIQUE_BRANCH)) {
    // Copy the header, pd, message_len, and pdl into the driver
    const size_t copy_size = header->pdl <= 231 ? header->pdl : 231;
    header->message_len = copy_size + 24;
    rdm_pd_emplace(header_ptr, "#cc01hbuubbbwbwb", header, sizeof(*header),
                   false);
    memcpy(pd_ptr, pd, copy_size);

    // Calculate and copy the checksum
    checksum = RDM_SC + RDM_SUB_SC;
    for (int i = 2; i < header->message_len; ++i) {
      checksum += header_ptr[i];
    }
    *(uint16_t *)(header_ptr + header->message_len) = bswap16(checksum);

    // Update written size
    written = header->message_len + 2;
  } else {
    // Encode the preamble bytes
    const size_t preamble_len = 7;
    for (int i = 0; i < preamble_len; ++i) {
      header_ptr[i] = RDM_PREAMBLE;
    }
    header_ptr[preamble_len] = RDM_DELIMITER;
    header_ptr += preamble_len + 1;

    // Encode the UID and calculate the checksum
    uint8_t uid[6];
    if (header == NULL) {
      memcpy(uid, pd, sizeof(rdm_uid_t));
    } else {
      rdm_uidcpy(uid, &header->src_uid);
    }
    for (int i = 0, j = 0; j < sizeof(rdm_uid_t); i += 2, ++j) {
      header_ptr[i] = uid[j] | 0xaa;
      header_ptr[i + 1] = uid[j] | 0x55;
      checksum += uid[j] + (0xaa | 0x55);
    }
    header_ptr += sizeof(rdm_uid_t) * 2;

    // Encode the checksum
    header_ptr[0] = (uint8_t)(checksum >> 8) | 0xaa;
    header_ptr[1] = (uint8_t)(checksum >> 8) | 0x55;
    header_ptr[2] = (uint8_t)(checksum) | 0xaa;
    header_ptr[3] = (uint8_t)(checksum) | 0x55;

    // Update written size
    written = preamble_len + 1 + 16;
  }

  // Update driver transmission size
  driver->tx_size = written;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return written;
}

size_t dmx_receive(dmx_port_t dmx_num, dmx_packet_t *packet,
                   TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_driver_is_enabled(dmx_num), 0, "driver is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Set default return value and default values for output argument
  dmx_err_t err = DMX_OK;
  uint32_t packet_size = 0;
  if (packet != NULL) {
    packet->err = DMX_ERR_TIMEOUT;
    packet->sc = -1;
    packet->size = 0;
    packet->is_rdm = 0;
  }

  // Block until mutex is taken and driver is idle, or until a timeout
  TimeOut_t timeout;
  vTaskSetTimeOutState(&timeout);
  if (!xSemaphoreTakeRecursive(driver->mux, wait_ticks) ||
      (wait_ticks && xTaskCheckForTimeOut(&timeout, &wait_ticks))) {
    return packet_size;
  } else if (!dmx_wait_sent(dmx_num, wait_ticks) ||
             (wait_ticks && xTaskCheckForTimeOut(&timeout, &wait_ticks))) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Set the RTS pin to enable reading from the DMX bus
  if (dmx_uart_get_rts(driver->uart) == 0) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    driver->head = -1;  // Wait for DMX break before reading data
    driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
    dmx_uart_set_rts(driver->uart, 1);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Wait for new DMX packet to be received
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  int driver_flags = driver->flags;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (!(driver_flags & DMX_FLAGS_DRIVER_HAS_DATA) && wait_ticks > 0) {
    // Set task waiting and get additional DMX driver flags
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    const int rdm_type = driver->rdm_type;
    driver->task_waiting = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

    // Check for early timeout according to RDM specification
    const int RDM_EARLY_TIMEOUT =
        (DMX_FLAGS_RDM_IS_REQUEST | DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH);
    if ((driver_flags & DMX_FLAGS_DRIVER_SENT_LAST) &&
        (rdm_type & RDM_EARLY_TIMEOUT) == RDM_EARLY_TIMEOUT) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      const int64_t last_timestamp = driver->last_slot_ts;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

      // Guard against setting hardware alarm durations with negative values
      int64_t elapsed = dmx_timer_get_micros_since_boot() - last_timestamp;
      if (elapsed >= RDM_PACKET_SPACING_CONTROLLER_NO_RESPONSE) {
        taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
        driver->task_waiting = NULL;
        taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
        xSemaphoreGiveRecursive(driver->mux);
        return packet_size;
      }

      // Set an early timeout with the hardware timer
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      dmx_timer_set_counter(driver->timer, elapsed);
      dmx_timer_set_alarm(driver->timer,
                          RDM_PACKET_SPACING_CONTROLLER_NO_RESPONSE, false);
      dmx_timer_start(driver->timer);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }

    // Wait for a task notification
    const bool notified = xTaskNotifyWait(0, -1, (uint32_t *)&err, wait_ticks);
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    dmx_timer_stop(driver->timer);
    driver->task_waiting = NULL;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (!notified) {
      xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
      xSemaphoreGiveRecursive(driver->mux);
      return packet_size;
    }
  } else if (!(driver_flags & DMX_FLAGS_DRIVER_HAS_DATA)) {
    // Fail early if there is no data available and this function cannot block
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Parse DMX data packet
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
  packet_size = driver->head;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (packet_size == -1) {
    packet_size = 0;
  }
  if (packet != NULL) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    packet->sc = packet_size > 0 ? driver->data[0] : -1;
    driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    packet->err = err;
    packet->size = packet_size;
    packet->is_rdm = 0;
  }

  // Return early if the no data was received
  if (packet_size == 0) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Return early if the packet is neither RDM nor an RDM request
  rdm_header_t header;
  if (!dmx_read_rdm(dmx_num, &header, NULL, 0) ||
      (header.cc != RDM_CC_DISC_COMMAND && header.cc != RDM_CC_GET_COMMAND &&
       header.cc != RDM_CC_SET_COMMAND)) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }
  if (packet != NULL) {
    packet->is_rdm = header.pid;
  }

  // Ignore the packet if it does not target this device
  rdm_uid_t my_uid;
  rdm_uid_get(dmx_num, &my_uid);
  if (!rdm_uid_is_target(&my_uid, &header.dest_uid)) {
    // The packet should be ignored
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Prepare the response packet parameter data and find the correct callback
  rdm_response_type_t response_type;
  uint8_t pdl_out;
  uint8_t pd[231];
  int cb_num = 0;
  for (; cb_num < driver->num_rdm_cbs; ++cb_num) {
    if (driver->rdm_cbs[cb_num].desc.pid == header.pid) {
      break;
    }
  }
  const rdm_pid_description_t *desc;
  void *param;
  if (cb_num < driver->num_rdm_cbs) {
    desc = &driver->rdm_cbs[cb_num].desc;
    param = driver->rdm_cbs[cb_num].param;
  } else {
    desc = NULL;
    param = NULL;
  }

  // Determine how this device should respond to the request
  if (header.pdl > sizeof(pd) || header.port_id == 0 ||
      rdm_uid_is_broadcast(&header.src_uid)) {
    // The packet format is invalid
    response_type = RDM_RESPONSE_TYPE_NACK_REASON;
    pdl_out = rdm_pd_emplace_word(pd, RDM_NR_FORMAT_ERROR);
  } else if (cb_num == driver->num_rdm_cbs) {
    // The requested PID is unknown
    response_type = RDM_RESPONSE_TYPE_NACK_REASON;
    pdl_out = rdm_pd_emplace_word(pd, RDM_NR_UNKNOWN_PID);
  } else if ((header.cc == RDM_CC_DISC_COMMAND && desc->cc != RDM_CC_DISC) ||
             (header.cc == RDM_CC_GET_COMMAND && !(desc->cc & RDM_CC_GET)) ||
             (header.cc == RDM_CC_SET_COMMAND && !(desc->cc & RDM_CC_SET))) {
    // The PID does not support the request command class
    response_type = RDM_RESPONSE_TYPE_NACK_REASON;
    pdl_out = rdm_pd_emplace_word(pd, RDM_NR_UNSUPPORTED_COMMAND_CLASS);
  } else if ((header.sub_device > 512 &&
              header.sub_device != RDM_SUB_DEVICE_ALL) ||
             (header.sub_device == RDM_SUB_DEVICE_ALL &&
              header.cc == RDM_CC_GET_COMMAND)) {
    // The sub-device is out of range
    response_type = RDM_RESPONSE_TYPE_NACK_REASON;
    pdl_out = rdm_pd_emplace_word(pd, RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  } else {
    // Call the appropriate driver-side RDM callback to process the request
    pdl_out = 0;
    dmx_read_rdm(dmx_num, NULL, pd, sizeof(pd));
    const char *param_str = driver->rdm_cbs[cb_num].param_str;
    response_type = driver->rdm_cbs[cb_num].driver_cb(
        dmx_num, &header, pd, &pdl_out, param_str);

    // Verify that the driver-side callback returned correctly
    if (pdl_out > sizeof(pd)) {
      DMX_WARN("PID 0x%04x pdl is too large", header.pid);
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      response_type = RDM_RESPONSE_TYPE_NACK_REASON;
      pdl_out = rdm_pd_emplace_word(pd, RDM_NR_HARDWARE_FAULT);
    } else if ((response_type != RDM_RESPONSE_TYPE_NONE &&
                response_type != RDM_RESPONSE_TYPE_ACK &&
                response_type != RDM_RESPONSE_TYPE_ACK_TIMER &&
                response_type != RDM_RESPONSE_TYPE_NACK_REASON &&
                response_type != RDM_RESPONSE_TYPE_ACK_OVERFLOW) ||
               (response_type == RDM_RESPONSE_TYPE_NONE &&
                (header.pid != RDM_PID_DISC_UNIQUE_BRANCH ||
                 !rdm_uid_is_broadcast(&header.dest_uid))) ||
               ((response_type != RDM_RESPONSE_TYPE_ACK &&
                 response_type != RDM_RESPONSE_TYPE_NONE) &&
                header.cc == RDM_CC_DISC_COMMAND)) {
      DMX_WARN("PID 0x%04x returned invalid response type", header.pid);
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      response_type = RDM_RESPONSE_TYPE_NACK_REASON;
      pdl_out = rdm_pd_emplace_word(pd, RDM_NR_HARDWARE_FAULT);
    }
  }

  // Don't respond to non-discovery broadcasts nor send NACK to DISC packets
  if ((rdm_uid_is_broadcast(&header.dest_uid) &&
       header.pid != RDM_PID_DISC_UNIQUE_BRANCH) ||
      (response_type != RDM_RESPONSE_TYPE_ACK &&
       header.cc == RDM_CC_DISC_COMMAND)) {
    response_type = RDM_RESPONSE_TYPE_NONE;
  }

  // Rewrite the header for the response packet
  header.message_len = 24 + pdl_out;  // Set for user callback
  header.dest_uid = header.src_uid;
  header.src_uid = my_uid;
  header.response_type = response_type;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  header.message_count = driver->rdm_queue_size;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  header.cc += 1;  // Set to RDM_CC_x_COMMAND_RESPONSE
  header.pdl = pdl_out;
  // These fields should not change: tn, sub_device, and pid

  // Send the response packet
  if (response_type != RDM_RESPONSE_TYPE_NONE) {
    const size_t response_size = dmx_write_rdm(dmx_num, &header, pd);
    if (!dmx_send(dmx_num, response_size)) {
      DMX_WARN("PID 0x%04x did not send a response", header.pid);
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    } else if (response_size > 0) {
      dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23));
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->head = -1;  // Wait for DMX break before reading data
      dmx_uart_set_rts(driver->uart, 1);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  }

  // Call the user-side callback
  if (cb_num < driver->num_rdm_cbs && driver->rdm_cbs[cb_num].user_cb != NULL) {
    void *context = driver->rdm_cbs[cb_num].context;
    driver->rdm_cbs[cb_num].user_cb(dmx_num, &header, context);
  }

  // Update NVS values
  if (driver->rdm_cbs[cb_num].non_volatile) {
    if (!dmx_nvs_set(dmx_num, header.pid, desc->data_type, param,
                     desc->pdl_size)) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->flags |= DMX_FLAGS_DRIVER_BOOT_LOADER;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      DMX_WARN("unable to save PID 0x%04x to NVS", header.pid);
    }
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return packet_size;
}

size_t dmx_send(dmx_port_t dmx_num, size_t size) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_driver_is_enabled(dmx_num), 0, "driver is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Block until the mutex can be taken
  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return 0;
  }

  // Block until the driver is done sending
  if (!dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23))) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Determine if it is too late to send a response packet
  int64_t elapsed = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const rdm_cc_t cc = driver->data[20];
  if (*(uint16_t *)driver->data == (RDM_SC | (RDM_SUB_SC << 8)) &&
      (cc == RDM_CC_DISC_COMMAND_RESPONSE ||
       cc == RDM_CC_GET_COMMAND_RESPONSE ||
       cc == RDM_CC_SET_COMMAND_RESPONSE)) {
    elapsed = dmx_timer_get_micros_since_boot() - driver->last_slot_ts;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (elapsed >= RDM_PACKET_SPACING_RESPONDER_NO_RESPONSE) {
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Determine if an alarm needs to be set to wait until driver is ready
  uint32_t timeout = 0;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver->flags & DMX_FLAGS_DRIVER_SENT_LAST) {
    if (driver->rdm_type & DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH) {
      timeout = RDM_PACKET_SPACING_DISCOVERY_NO_RESPONSE;
    } else if (driver->rdm_type & DMX_FLAGS_RDM_IS_BROADCAST) {
      timeout = RDM_PACKET_SPACING_BROADCAST;
    } else if (driver->rdm_type == DMX_FLAGS_RDM_IS_REQUEST) {
      timeout = RDM_PACKET_SPACING_REQUEST_NO_RESPONSE;
    }
  } else if (driver->rdm_type & DMX_FLAGS_RDM_IS_VALID) {
    timeout = RDM_PACKET_SPACING_RESPONSE;
  }
  elapsed = dmx_timer_get_micros_since_boot() - driver->last_slot_ts;
  if (elapsed < timeout) {
    dmx_timer_set_counter(driver->timer, elapsed);
    dmx_timer_set_alarm(driver->timer, timeout, false);
    dmx_timer_start(driver->timer);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Block if an alarm was set
  if (elapsed < timeout) {
    // FIXME: clean up this section
    bool notified = xTaskNotifyWait(0, ULONG_MAX, NULL, pdDMX_MS_TO_TICKS(23));
    if (!notified) {
      dmx_timer_stop(driver->timer);
      xTaskNotifyStateClear(driver->task_waiting);
    }
    driver->task_waiting = NULL;
    if (!notified) {
      xSemaphoreGiveRecursive(driver->mux);
      return 0;
    }
  }

  // Turn the DMX bus around and get the send size
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (dmx_uart_get_rts(driver->uart) == 1) {
    xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    dmx_uart_set_rts(driver->uart, 0);
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Update the transmit size if desired
  if (size > 0) {
    if (size > DMX_PACKET_SIZE_MAX) {
      size = DMX_PACKET_SIZE_MAX;
    }
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->tx_size = size;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    size = driver->tx_size;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Record the outgoing packet type
  const rdm_pid_t pid = bswap16(*(uint16_t *)&driver->data[21]);
  rdm_uid_t dest_uid;
  rdm_uidcpy(&dest_uid, &driver->data[3]);
  int rdm_type = 0;
  if (*(uint16_t *)driver->data == (RDM_SC | (RDM_SUB_SC << 8))) {
    rdm_type |= DMX_FLAGS_RDM_IS_VALID;
    if (cc == RDM_CC_DISC_COMMAND || cc == RDM_CC_GET_COMMAND ||
        cc == RDM_CC_SET_COMMAND) {
      rdm_type |= DMX_FLAGS_RDM_IS_REQUEST;
    }
    if (rdm_uid_is_broadcast(&dest_uid)) {
      rdm_type |= DMX_FLAGS_RDM_IS_BROADCAST;
    }
    if (pid == RDM_PID_DISC_UNIQUE_BRANCH) {
      rdm_type |= DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
    }
  } else if (driver->data[0] == RDM_PREAMBLE ||
             driver->data[0] == RDM_DELIMITER) {
    rdm_type |= DMX_FLAGS_RDM_IS_VALID | DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH;
  }
  driver->rdm_type = rdm_type;
  driver->flags |= DMX_FLAGS_DRIVER_SENT_LAST;
  if ((rdm_type & (DMX_FLAGS_RDM_IS_VALID | DMX_FLAGS_RDM_IS_REQUEST)) ==
      (DMX_FLAGS_RDM_IS_VALID | DMX_FLAGS_RDM_IS_REQUEST)) {
    ++driver->tn;
  }

  // Determine if a DMX break is required and send the packet
  if (rdm_type ==
      (DMX_FLAGS_RDM_IS_VALID | DMX_FLAGS_RDM_IS_DISC_UNIQUE_BRANCH)) {
    // RDM discovery responses do not send a DMX break - write immediately
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->flags |= DMX_FLAGS_DRIVER_IS_SENDING;

    size_t write_size = driver->tx_size;
    dmx_uart_write_txfifo(driver->uart, driver->data, &write_size);
    driver->head = write_size;

    // Enable DMX write interrupts
    dmx_uart_enable_interrupt(driver->uart, DMX_INTR_TX_ALL);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    // Send the packet by starting the DMX break
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->head = 0;
    driver->flags |=
        (DMX_FLAGS_DRIVER_IS_IN_BREAK | DMX_FLAGS_DRIVER_IS_SENDING);
    dmx_timer_set_counter(driver->timer, 0);
    dmx_timer_set_alarm(driver->timer, driver->break_len, true);
    dmx_timer_start(driver->timer);

    dmx_uart_invert_tx(driver->uart, 1);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return size;
}

bool dmx_wait_sent(dmx_port_t dmx_num, TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  // Block until the mutex can be taken
  TimeOut_t timeout;
  vTaskSetTimeOutState(&timeout);
  if (!xSemaphoreTakeRecursive(driver->mux, wait_ticks) ||
      (wait_ticks && xTaskCheckForTimeOut(&timeout, &wait_ticks))) {
    return false;
  }

  // Determine if the task needs to block
  bool result = true;
  if (wait_ticks > 0) {
    bool task_waiting = false;
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) {
      driver->task_waiting = xTaskGetCurrentTaskHandle();
      task_waiting = true;
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

    // Wait for a notification that the driver is done sending
    if (task_waiting) {
      result = xTaskNotifyWait(0, ULONG_MAX, NULL, wait_ticks);
      driver->task_waiting = NULL;
    }
  } else {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (driver->flags & DMX_FLAGS_DRIVER_IS_SENDING) {
      result = false;
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return result;
}