#include "esp_dmx.h"

#include "dmx/config.h"
#include "dmx/struct.h"
#include "endian.h"

bool DMX_ISR_ATTR dmx_driver_is_installed(dmx_port_t dmx_num) {
  return dmx_num < DMX_NUM_MAX && dmx_driver[dmx_num] != NULL;
}

bool dmx_driver_is_enabled(dmx_port_t dmx_num) {
  bool is_enabled;
  if (dmx_driver_is_installed(dmx_num)) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    is_enabled = dmx_driver[dmx_num]->flags & DMX_FLAGS_DRIVER_IS_ENABLED;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    is_enabled = false;
  }

  return is_enabled;
}

uint32_t dmx_get_break_len(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint32_t break_len;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  break_len = dmx_driver[dmx_num]->break_len;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return break_len;
}

uint32_t dmx_set_break_len(dmx_port_t dmx_num, uint32_t break_len) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp the break length to within DMX specification
  if (break_len < DMX_BREAK_LEN_MIN_US) {
    break_len = DMX_BREAK_LEN_MIN_US;
  } else if (break_len > DMX_BREAK_LEN_MAX_US) {
    break_len = DMX_BREAK_LEN_MAX_US;
  }

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->break_len = break_len;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return break_len;
}

uint32_t dmx_set_mab_len(dmx_port_t dmx_num, uint32_t mab_len) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  // Clamp the mark-after-break length to within DMX specification
  if (mab_len < DMX_MAB_LEN_MIN_US) {
    mab_len = DMX_MAB_LEN_MIN_US;
  } else if (mab_len > DMX_MAB_LEN_MAX_US) {
    mab_len = DMX_MAB_LEN_MAX_US;
  }

  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_driver[dmx_num]->mab_len = mab_len;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return mab_len;
}

uint32_t dmx_get_mab_len(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint32_t mab_len;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  mab_len = dmx_driver[dmx_num]->mab_len;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return mab_len;
}

uint8_t dmx_get_current_personality(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint8_t current_personality;

  const rdm_device_info_t *device_info =
      rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
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
  rdm_device_info_t *device_info = rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
  if (device_info == NULL) {
    dmx_driver_personality_t *personality = (void *)dmx_driver[dmx_num]->pd;
    current_personality = &personality->current_personality;
    footprint = NULL;
  } else {
    current_personality = &device_info->current_personality;
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
      rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
  if (device_info == NULL) {
    const dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->pd;
    return personality->personality_count;
  } else {
    return device_info->personality_count;
  }
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



uint16_t dmx_get_start_address(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  uint16_t dmx_start_address;

  const rdm_device_info_t *device_info =
      rdm_pd_find(dmx_num, RDM_PID_DEVICE_INFO);
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (device_info == NULL) {
    const dmx_driver_personality_t *personality =
        (void *)dmx_driver[dmx_num]->pd;
    dmx_start_address = personality->dmx_start_address;
  } else {
    dmx_start_address = device_info->dmx_start_address;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return dmx_start_address;
}

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

bool dmx_sniffer_is_enabled(dmx_port_t dmx_num) {
  return dmx_driver_is_installed(dmx_num) &&
         dmx_driver[dmx_num]->metadata_queue != NULL;
}

bool dmx_sniffer_get_data(dmx_port_t dmx_num, dmx_metadata_t *metadata,
                          TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(metadata, false, "metadata is null");
  DMX_CHECK(dmx_sniffer_is_enabled(dmx_num), false, "sniffer is not enabled");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  return xQueueReceive(driver->metadata_queue, metadata, wait_ticks);
}