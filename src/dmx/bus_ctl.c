#include "dmx/bus_ctl.h"

#include <string.h>

#include "dmx/driver.h"
#include "dmx/hal/nvs.h"
#include "dmx/struct.h"
#include "endian.h"
#include "rdm/utils/bus_ctl.h"
#include "rdm/utils/uid.h"

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

  // TODO: Return early if RDM is disabled

  // Validate checksum and get the packet header
  rdm_header_t header;
  if (!rdm_read_header(dmx_num, &header)) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Validate that the packet targets this device and is a request
  rdm_uid_t this_uid;
  rdm_uid_get(dmx_num, &this_uid);
  if (!rdm_uid_is_target(&this_uid, &header.dest_uid) ||
      !rdm_cc_is_request(header.cc)) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Get parameter definition
  size_t resp;  // Size of the response packet
  const rdm_pd_definition_t *def = rdm_pd_get_definition(header.pid);
  if (def == NULL) {
    // Unknown PID
    resp = rdm_write_nack_reason(dmx_num, &header, RDM_NR_UNKNOWN_PID);
  } else if (header.message_len < 24 ||
             (header.sub_device >= RDM_SUB_DEVICE_MAX &&
              header.sub_device != RDM_SUB_DEVICE_ALL) ||
             !rdm_cc_is_valid(header.cc) || header.pdl > 231) {
    // Header format is invalid
    resp = rdm_write_nack_reason(dmx_num, &header, RDM_NR_FORMAT_ERROR);
  } else {
    // Request is valid, handle the response

    // Validate the header against definition information
    const rdm_pid_cc_t pid_cc = def->pid_cc;
    if ((pid_cc == RDM_CC_DISC && header.cc != RDM_CC_DISC_COMMAND) ||
        (pid_cc == RDM_CC_GET_SET && header.cc == RDM_CC_DISC_COMMAND) ||
        (pid_cc == RDM_CC_GET && header.cc != RDM_CC_GET_COMMAND) ||
        (pid_cc == RDM_CC_SET && header.cc != RDM_CC_SET_COMMAND)) {
      // Unsupported command class
      resp = rdm_write_nack_reason(dmx_num, &header,
                                   RDM_NR_UNSUPPORTED_COMMAND_CLASS);
    } else {
      // Call the response handler for the parameter
      if (header.cc == RDM_CC_SET_COMMAND) {
        resp = def->set.handler(dmx_num, def, &header);
      } else {
        // RDM_CC_DISC_COMMAND uses get.handler()
        resp = def->get.handler(dmx_num, def, &header);
      }

      // Validate the response
      if (header.pid == RDM_PID_DISC_UNIQUE_BRANCH &&
          ((resp > 0 && resp < 17) || resp > 24)) {
        // Invalid RDM_CC_DISC_COMMAND_RESPONSE packet size
        resp = 0;  // Silence invalid discovery responses
        rdm_set_boot_loader(dmx_num);
      } else if (resp > 255 ||
                 (resp == 0 && !rdm_uid_is_broadcast(&header.dest_uid))) {
        // Response size is too large or zero after a non-broadcast request
        resp = rdm_write_nack_reason(dmx_num, &header, RDM_NR_HARDWARE_FAULT);
        rdm_set_boot_loader(dmx_num);
      }
    }
  }

  // Do not send a response to non-discovery broadcast packets
  if (rdm_uid_is_broadcast(&header.dest_uid) &&
      header.pid != RDM_PID_DISC_UNIQUE_BRANCH) {
    resp = 0;
  }

  // Send the RDM response
  if (resp > 0) {
    if (!dmx_send(dmx_num, resp)) {
      const int64_t micros_elapsed =
          dmx_timer_get_micros_since_boot() - driver->last_slot_ts;
      DMX_WARN("PID 0x%04x did not send a response (size: %i, time: %lli us)",
               header.pid, resp, micros_elapsed);
      // TODO: generate more debug info: i.e. GET/SET/DISC request?
      rdm_set_boot_loader(dmx_num);
    } else {
      dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23));
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->head = -1;  // Wait for DMX break before reading data
      dmx_uart_set_rts(driver->uart, 1);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  }

  // Call the after-response callback
  rdm_header_t response_header;
  if (!rdm_read_header(dmx_num, &response_header)) {
    // Set the response header to NULL if an RDM header can't be read
    memset(&response_header, 0, sizeof(response_header));
  }
  rdm_pd_handle_callback(dmx_num, header.pid, &header, &response_header);

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
