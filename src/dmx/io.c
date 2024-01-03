#include <string.h>

#include "dmx/hal/include/nvs.h"
#include "dmx/include/driver.h"
#include "dmx/include/parameter.h"
#include "dmx/include/service.h"
#include "rdm/include/driver.h"
#include "rdm/include/uid.h"
#include "rdm/responder/include/utils.h"

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
  memcpy(destination, driver->dmx.data + offset, size);

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

  // Check if the driver is currently sending an RDM packet
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const int driver_flags = driver->flags;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (driver_flags & DMX_FLAGS_DRIVER_IS_SENDING) {
    rdm_header_t header;
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    const bool is_rdm = rdm_read_header(dmx_num, &header);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (is_rdm) {
      return 0;  // Do not allow asynchronous writes while sending RDM
    }
  }

  // Flip the DMX bus to write mode
  if (dmx_uart_get_rts(dmx_num) == 1) {
    dmx_uart_set_rts(dmx_num, 0);
  }

  // Copy data from the source to the driver buffer asynchronously
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  memcpy(driver->dmx.data + offset, source, size);
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

  // Block until mutex is taken and driver is idle, or until a timeout
  TimeOut_t timeout;
  vTaskSetTimeOutState(&timeout);
  if (!xSemaphoreTakeRecursive(driver->mux, wait_ticks) ||
      (wait_ticks && xTaskCheckForTimeOut(&timeout, &wait_ticks))) {
    if (packet != NULL) {
      packet->err = DMX_ERR_TIMEOUT;
      packet->sc = -1;
      packet->size = 0;
      packet->is_rdm = 0;
    }
    return 0;
  } else if (!dmx_wait_sent(dmx_num, wait_ticks) ||
             (wait_ticks && xTaskCheckForTimeOut(&timeout, &wait_ticks))) {
    xSemaphoreGiveRecursive(driver->mux);
    if (packet != NULL) {
      packet->err = DMX_ERR_TIMEOUT;
      packet->sc = -1;
      packet->size = 0;
      packet->is_rdm = 0;
    }
    return 0;
  }

  // Set the RTS pin to enable reading from the DMX bus
  if (dmx_uart_get_rts(dmx_num) == 0) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    driver->dmx.head = -1;  // Wait for DMX break before reading data
    driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
    dmx_uart_set_rts(dmx_num, 1);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Wait for new DMX packet to be received
  dmx_err_t err = DMX_OK;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const int driver_flags = driver->flags;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (!(driver_flags & DMX_FLAGS_DRIVER_HAS_DATA) && wait_ticks > 0) {
    // Set task waiting and get additional DMX driver flags
    rdm_header_t header;
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    const bool is_rdm = rdm_read_header(dmx_num, &header);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

    // Check for early timeout according to RDM specification
    if ((driver_flags & DMX_FLAGS_DRIVER_SENT_LAST) && is_rdm &&
        header.cc == RDM_CC_DISC_COMMAND &&
        header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      const int64_t last_timestamp = driver->dmx.last_slot_ts;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

      // Guard against setting hardware alarm durations with negative values
      int64_t elapsed = dmx_timer_get_micros_since_boot() - last_timestamp;
      if (elapsed >= RDM_PACKET_SPACING_CONTROLLER_NO_RESPONSE) {
        taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
        driver->task_waiting = NULL;
        taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
        xSemaphoreGiveRecursive(driver->mux);
        if (packet != NULL) {
          packet->err = DMX_ERR_TIMEOUT;
          packet->sc = -1;
          packet->size = 0;
          packet->is_rdm = 0;
        }
        return 0;
      }

      // Set an early timeout with the hardware timer
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      dmx_timer_set_counter(dmx_num, elapsed);
      dmx_timer_set_alarm(dmx_num,
                          RDM_PACKET_SPACING_CONTROLLER_NO_RESPONSE, false);
      dmx_timer_start(dmx_num);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }

    // Wait for a task notification
    const bool notified = xTaskNotifyWait(0, -1, (uint32_t *)&err, wait_ticks);
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    dmx_timer_stop(dmx_num);
    driver->task_waiting = NULL;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (!notified) {
      xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
      xSemaphoreGiveRecursive(driver->mux);
      if (packet != NULL) {
        packet->err = DMX_ERR_TIMEOUT;
        packet->sc = -1;
        packet->size = 0;
        packet->is_rdm = 0;
      }
      dmx_parameter_commit(dmx_num);
      return 0;
    }
  } else if (!(driver_flags & DMX_FLAGS_DRIVER_HAS_DATA)) {
    // Fail early if there is no data available and this function cannot block
    xSemaphoreGiveRecursive(driver->mux);
    if (packet != NULL) {
      packet->err = DMX_ERR_TIMEOUT;
      packet->sc = -1;
      packet->size = 0;
      packet->is_rdm = 0;
    }
    dmx_parameter_commit(dmx_num);
    return 0;
  }

  // Parse DMX data packet
  uint32_t packet_size;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
  packet_size = driver->dmx.head;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (packet_size == -1) {
    packet_size = 0;
  }
  if (packet != NULL) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    packet->sc = packet_size > 0 ? driver->dmx.data[0] : -1;
    driver->flags &= ~DMX_FLAGS_DRIVER_HAS_DATA;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    packet->err = err;
    packet->size = packet_size;
  }

  // Return early if the no data was received
  if (packet_size == 0) {
    xSemaphoreGiveRecursive(driver->mux);
    if (packet != NULL) {
      packet->is_rdm = 0;
    }
    dmx_parameter_commit(dmx_num);
    return packet_size;
  }

  // Validate checksum and get the packet header
  rdm_header_t header;
  if (!rdm_read_header(dmx_num, &header)) {
    xSemaphoreGiveRecursive(driver->mux);
    if (packet != NULL) {
      packet->is_rdm = 0;
    }
    return packet_size;
  }

  // Record the RDM PID
  if (packet != NULL) {
    packet->is_rdm = header.pid;
  }

  // TODO: Return early if RDM is disabled

  // Validate that the packet targets this device and is a request
  if (!rdm_uid_is_target(rdm_uid_get(dmx_num), &header.dest_uid) ||
      !rdm_cc_is_request(header.cc)) {
    xSemaphoreGiveRecursive(driver->mux);
    return packet_size;
  }

  // Get parameter definition
  size_t resp;  // Size of the response packet
  const rdm_parameter_definition_t *def =
      dmx_parameter_rdm_lookup(dmx_num, header.sub_device, header.pid);
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
    if (pid_cc == RDM_CC_DISC && header.cc != RDM_CC_DISC_COMMAND) {
      resp = 0;  // Cannot send NACK to RDM_CC_DISC_COMMAND
    } else if ((pid_cc == RDM_CC_GET_SET && header.cc == RDM_CC_DISC_COMMAND) ||
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
    if (!dmx_send_num(dmx_num, resp)) {
      rdm_set_boot_loader(dmx_num);
      // Generate information for the warning message if a response wasn't sent
      const int64_t micros_elapsed =
          dmx_timer_get_micros_since_boot() - driver->dmx.last_slot_ts;
      const char *cc_str = header.cc == RDM_CC_GET_COMMAND   ? "GET"
                           : header.cc == RDM_CC_SET_COMMAND ? "SET"
                                                             : "DISC";
      DMX_WARN(
          "PID 0x%04x did not send a response (cc: %s, size: %i, time: %lli "
          "us)",
          header.pid, cc_str, resp, micros_elapsed);
    } else {
      dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23));
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->dmx.head = -1;  // Wait for DMX break before reading data
      dmx_uart_set_rts(dmx_num, 1);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  }

  // Call the after-response callback
  rdm_header_t response_header;
  if (!rdm_read_header(dmx_num, &response_header)) {
    // Set the response header to NULL if an RDM header can't be read
    memset(&response_header, 0, sizeof(response_header));
  }
  dmx_parameter_rdm_handle_callback(dmx_num, header.sub_device, header.pid,
                                    &header, &response_header);

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return packet_size;
}

size_t dmx_send_num(dmx_port_t dmx_num, size_t size) {
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

  // Determine if the packet was an RDM packet
  rdm_header_t header;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  const bool is_rdm = rdm_read_header(dmx_num, &header);
  const int driver_flags = driver->flags;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Determine if it is too late to send a response packet
  int64_t elapsed = 0;
  if (is_rdm && !rdm_cc_is_request(header.cc)) {
    elapsed = dmx_timer_get_micros_since_boot() - driver->dmx.last_slot_ts;
    if (elapsed >= RDM_PACKET_SPACING_RESPONDER_NO_RESPONSE) {
      xSemaphoreGiveRecursive(driver->mux);
      return 0;
    }
  }

  // Determine if an alarm needs to be set to wait until driver is ready
  uint32_t timeout;
  if (!is_rdm) {
    timeout = 0;
  } else if (driver_flags & DMX_FLAGS_DRIVER_SENT_LAST) {
    if (header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
      timeout = RDM_PACKET_SPACING_DISCOVERY_NO_RESPONSE;
    } else if (rdm_uid_is_broadcast(&header.dest_uid)) {
      timeout = RDM_PACKET_SPACING_BROADCAST;
    } else if (rdm_cc_is_request(header.cc)) {
      timeout = RDM_PACKET_SPACING_REQUEST_NO_RESPONSE;
    } else {
      timeout = 0;
    }
  } else {
    timeout = RDM_PACKET_SPACING_RESPONSE;
  }
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  elapsed = dmx_timer_get_micros_since_boot() - driver->dmx.last_slot_ts;
  if (elapsed < timeout) {
    dmx_timer_set_counter(dmx_num, elapsed);
    dmx_timer_set_alarm(dmx_num, timeout, false);
    dmx_timer_start(dmx_num);
    driver->task_waiting = xTaskGetCurrentTaskHandle();
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Block if an alarm was set
  if (elapsed < timeout) {
    if (!xTaskNotifyWait(0, ULONG_MAX, NULL, pdDMX_MS_TO_TICKS(20))) {
      __unreachable();  // The hardware timer should always notify the task
    }
    driver->task_waiting = NULL;
  }

  // Turn the DMX bus around and get the send size
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (dmx_uart_get_rts(dmx_num) == 1) {
    dmx_uart_set_rts(dmx_num, 0);
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Determine the size of the packet to send
  if (is_rdm) {
    if (header.cc == RDM_CC_DISC_COMMAND_RESPONSE &&
        header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
      size = header.message_len;  // Send an RDM_PID_DISC_UNIQUE_BRANCH response
    } else {
      size = header.message_len + 2;  // Send a standard RDM packet
    }
  } else if (size == 0 || size > DMX_PACKET_SIZE_MAX) {
    size = DMX_PACKET_SIZE_MAX;  // Send a standard DMX packet
  }
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  driver->dmx.tx_size = size;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Update driver flags and increment the RDM transaction number if applicable
  driver->flags |= DMX_FLAGS_DRIVER_SENT_LAST;
  if (is_rdm) {
    ++driver->rdm.tn;
  }

  // Determine if a DMX break is required and send the packet
  if (is_rdm && header.cc == RDM_CC_DISC_COMMAND_RESPONSE &&
      header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // RDM discovery responses do not send a DMX break - write immediately
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->flags |= DMX_FLAGS_DRIVER_IS_SENDING;

    size_t write_size = driver->dmx.tx_size;
    dmx_uart_write_txfifo(dmx_num, driver->dmx.data, &write_size);
    driver->dmx.head = write_size;

    // Enable DMX write interrupts
    dmx_uart_enable_interrupt(dmx_num, DMX_INTR_TX_ALL);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    // Send the packet by starting the DMX break
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.head = 0;
    driver->flags |=
        (DMX_FLAGS_DRIVER_IS_IN_BREAK | DMX_FLAGS_DRIVER_IS_SENDING);
    dmx_timer_set_counter(dmx_num, 0);
    dmx_timer_set_alarm(dmx_num, driver->break_len, true);
    dmx_timer_start(dmx_num);

    dmx_uart_invert_tx(dmx_num, 1);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Give the mutex back
  xSemaphoreGiveRecursive(driver->mux);
  return size;
}

size_t dmx_send(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_driver_is_enabled(dmx_num), 0, "driver is not enabled");

  return dmx_send_num(dmx_num, 0);
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
