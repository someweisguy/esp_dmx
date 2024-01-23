#include <string.h>

#include "dmx/hal/include/gpio.h"
#include "dmx/hal/include/nvs.h"
#include "dmx/hal/include/timer.h"
#include "dmx/hal/include/uart.h"
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

size_t dmx_receive_num(dmx_port_t dmx_num, dmx_packet_t *packet, size_t size,
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
    dmx_parameter_commit(dmx_num);  // Commit pending non-volatile parameters
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
    dmx_parameter_commit(dmx_num);  // Commit pending non-volatile parameters
    return 0;
  }

  // Set the RTS pin to enable reading from the DMX bus
  if (dmx_uart_get_rts(dmx_num) == 0) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    driver->dmx.progress = DMX_PROGRESS_STALE;
    driver->dmx.head = -1;  // Wait for DMX break before reading data
    dmx_uart_set_rts(dmx_num, 1);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Update the receive size only if it has changed
  if (size != driver->dmx.size) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.size = size;
    if (!driver->rdm.rx.type && driver->dmx.head > 0) {
      // It is necessary to revalidate if the DMX data is ready
      if (driver->dmx.head >= driver->dmx.size) {
        driver->dmx.progress = DMX_PROGRESS_COMPLETE;
      } else {
        driver->dmx.progress = DMX_PROGRESS_STALE;
      }
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Guard against condition where this task cannot block and data isn't ready
  uint8_t packet_status;
  int16_t packet_size;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  packet_status = driver->dmx.progress;
  packet_size = driver->dmx.head;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (packet_status != DMX_PROGRESS_COMPLETE && wait_ticks == 0) {
    // Not enough DMX data has been received yet - return early
    if (packet_size < 0) {
      packet_size = 0;
    }
    if (packet != NULL) {
      packet->err = DMX_ERR_TIMEOUT;
      if (packet_size > 0) {
        taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
        packet->sc = driver->dmx.data[0];
        taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      } else {
        packet->sc = -1;
      }
      packet->size = packet_size;
      packet->is_rdm = 0;
    }
    xSemaphoreGiveRecursive(driver->mux);
    dmx_parameter_commit(dmx_num);  // Commit pending non-volatile parameters
    return packet_size;
  }

  // Block the task to wait for data to be ready
  dmx_err_t err;
  if (packet_status != DMX_PROGRESS_COMPLETE) {
    // Tell the DMX driver that this task is awaiting a DMX packet
    bool sent_last;
    const TaskHandle_t this_task = xTaskGetCurrentTaskHandle();
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->task_waiting = this_task;
    sent_last = (driver->dmx.rx.ts < driver->dmx.tx.ts);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

    // Determine if it is necessary to set a hardware timeout alarm
    int64_t timer_alarm;
    if (sent_last && driver->rdm.rx.type) {
      timer_alarm = 0;  // TODO
    } else {
      timer_alarm = 0;  // No alarm is necessary
    }

    if (timer_alarm > 0) {
      int64_t last_timestamp;
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      last_timestamp = 0;  // FIXME determine timer_alarm value
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      int64_t timer_elapsed =
          dmx_timer_get_micros_since_boot() - last_timestamp;
      if (timer_elapsed > timer_alarm) {
        // Return early if the time elapsed is greater than the timer alarm
        if (packet_size < 0) {
          packet_size = 0;
        }
        if (packet != NULL) {
          packet->err = DMX_ERR_TIMEOUT;
          if (packet_size > 0) {
            taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
            packet->sc = driver->dmx.data[0];
            taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
          } else {
            packet->sc = -1;
          }
          packet->size = packet_size;
          packet->is_rdm = 0;
        }
        xSemaphoreGiveRecursive(driver->mux);
        return packet_size;
      }

      // Set the timer
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      dmx_timer_set_counter(dmx_num, timer_elapsed);
      dmx_timer_set_alarm(dmx_num, timer_alarm, false);
      dmx_timer_start(dmx_num);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }

    // Wait for the DMX driver to notify this task that DMX is ready
    const bool notified = xTaskNotifyWait(0, -1, (uint32_t *)&err, wait_ticks);
    if (timer_alarm > 0) {
      dmx_timer_stop(dmx_num);
    }
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    packet_size = driver->dmx.head;
    driver->task_waiting = NULL;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (!notified) {
      xTaskNotifyStateClear(this_task);  // Avoid potential race condition
      if (packet_size < 0) {
        packet_size = 0;
      }
      if (packet != NULL) {
        packet->err = DMX_ERR_TIMEOUT;
        if (packet_size > 0) {
          taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
          packet->sc = driver->dmx.data[0];
          taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
        } else {
          packet->sc = -1;
        }
        packet->size = packet_size;
        packet->is_rdm = 0;
      }
      xSemaphoreGiveRecursive(driver->mux);
      dmx_parameter_commit(dmx_num);
      return packet_size;
    }
  } else {
    // TODO: Fix condition where DMX error can be lost if no task is waiting?
    err = DMX_OK;
  }
  if (packet_size < 0) {
    packet_size = 0;
  }

  // Parse DMX packet data
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  driver->dmx.progress = DMX_PROGRESS_STALE;  // Prevent parsing old data
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (packet != NULL) {
    if (packet_size > 0) {
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      packet->sc = driver->dmx.data[0];
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    } else {
      packet->sc = -1;
    }
    packet->err = err;
    packet->size = packet_size;
    packet->is_rdm = 0;
  }

  // Return early if an error occurred
  if (err != DMX_OK) {
    xSemaphoreGiveRecursive(driver->mux);
    dmx_parameter_commit(dmx_num);  // Commit pending non-volatile parameters
    return packet_size;
  }

  // Get the RDM header information and update miscellaneous RDM driver fields
  bool is_rdm;
  rdm_header_t header;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  is_rdm = rdm_read_header(dmx_num, &header);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (is_rdm) {
    if (packet != NULL) {
      packet->is_rdm = header.pid;
    }
  } else {
    header.pid = 0;
  }
  if (driver->rdm.rx.pid == header.pid) {
    ++driver->rdm.rx.pid_repeats;
  } else {
    driver->rdm.rx.pid = header.pid;
    driver->rdm.rx.pid_repeats = 0;
  }

  // TODO: Return early if RDM is disabled

  // Return early if the packet isn't a request or this device isn't addressed
  const rdm_uid_t *this_uid = rdm_uid_get(dmx_num);
  if (!is_rdm || !rdm_cc_is_valid(header.cc) || !rdm_cc_is_request(header.cc) ||
      !rdm_uid_is_target(this_uid, &header.dest_uid)) {
    xSemaphoreGiveRecursive(driver->mux);
    dmx_parameter_commit(dmx_num);  // Commit pending non-volatile parameters
    return packet_size;
  }

  // Get parameter definition
  size_t reply;  // Size of the response packet
  const rdm_parameter_definition_t *def =
      rdm_definition_get(dmx_num, header.sub_device, header.pid);
  if (def == NULL) {
    // Unknown PID
    reply = rdm_write_nack_reason(dmx_num, &header, RDM_NR_UNKNOWN_PID);
  } else if ((header.sub_device >= RDM_SUB_DEVICE_MAX &&
              header.sub_device != RDM_SUB_DEVICE_ALL) ||
             header.pdl >= RDM_PD_SIZE_MAX) {
    // Header format is invalid
    reply = rdm_write_nack_reason(dmx_num, &header, RDM_NR_FORMAT_ERROR);
  } else {
    // Request is valid, handle the response

    // Validate the header against definition information
    const rdm_pid_cc_t pid_cc = def->pid_cc;
    if (pid_cc == RDM_CC_DISC && header.cc != RDM_CC_DISC_COMMAND) {
      reply = 0;  // Cannot send NACK to RDM_CC_DISC_COMMAND
    } else if ((pid_cc == RDM_CC_GET_SET && header.cc == RDM_CC_DISC_COMMAND) ||
               (pid_cc == RDM_CC_GET && header.cc != RDM_CC_GET_COMMAND) ||
               (pid_cc == RDM_CC_SET && header.cc != RDM_CC_SET_COMMAND)) {
      // Unsupported command class
      reply = rdm_write_nack_reason(dmx_num, &header,
                                    RDM_NR_UNSUPPORTED_COMMAND_CLASS);
    } else {
      // Call the response handler for the parameter
      if (header.cc == RDM_CC_SET_COMMAND) {
        reply = def->set.handler(dmx_num, def, &header);
      } else {
        // RDM_CC_DISC_COMMAND uses get.handler()
        reply = def->get.handler(dmx_num, def, &header);
      }

      // Validate the response
      if (header.pid == RDM_PID_DISC_UNIQUE_BRANCH &&
          ((reply > 0 && reply < 17) || reply > 24)) {
        // Invalid RDM_CC_DISC_COMMAND_RESPONSE packet size
        reply = 0;  // Silence invalid discovery responses
        rdm_set_boot_loader(dmx_num);
      } else if (reply > 255 ||
                 (reply == 0 && !rdm_uid_is_broadcast(&header.dest_uid))) {
        // Response size is too large or zero after a non-broadcast request
        reply = rdm_write_nack_reason(dmx_num, &header, RDM_NR_HARDWARE_FAULT);
        rdm_set_boot_loader(dmx_num);
      }
    }
  }

  // Do not send a response to non-discovery broadcast packets
  if (rdm_uid_is_broadcast(&header.dest_uid) &&
      header.pid != RDM_PID_DISC_UNIQUE_BRANCH) {
    reply = 0;
  }

  // Send the RDM response
  if (reply > 0) {
    if (!dmx_send_num(dmx_num, reply)) {
      rdm_set_boot_loader(dmx_num);
      // Generate information for the warning message if a response wasn't sent
      const int64_t micros_elapsed =
          dmx_timer_get_micros_since_boot() - driver->dmx.rx.ts;
      const char *cc_str = header.cc == RDM_CC_GET_COMMAND   ? "GET"
                           : header.cc == RDM_CC_SET_COMMAND ? "SET"
                                                             : "DISC";
      DMX_WARN(
          "PID 0x%04x did not send a response (cc: %s, size: %i, time: %lli "
          "us)",
          header.pid, cc_str, reply, micros_elapsed);
    } else {
      dmx_wait_sent(dmx_num, pdDMX_MS_TO_TICKS(23));
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->dmx.head = -1;  // Wait for DMX break before reading data
      dmx_uart_set_rts(dmx_num, 1);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  }

  // Call the after-response callback
  rdm_callback_handle(dmx_num, header.sub_device, header.pid, &header);

  xSemaphoreGiveRecursive(driver->mux);
  dmx_parameter_commit(dmx_num);  // Commit pending non-volatile parameters
  return packet_size;
}

size_t dmx_receive(dmx_port_t dmx_num, dmx_packet_t *packet,
                   TickType_t wait_ticks) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");
  DMX_CHECK(dmx_driver_is_enabled(dmx_num), 0, "driver is not enabled");

  size_t size;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  size = dmx_driver[dmx_num]->dmx.size;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return dmx_receive_num(dmx_num, packet, size, wait_ticks);
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
    elapsed = 0;  // FIXME
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
  elapsed = 0;  // FIXME
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
  driver->dmx.size = size;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Update driver flags and increment the RDM transaction number if applicable
  driver->flags |= DMX_FLAGS_DRIVER_SENT_LAST;
  if (is_rdm && rdm_cc_is_request(header.cc)) {
    ++driver->rdm.tn;
  }

  // Determine if a DMX break is required and send the packet
  if (is_rdm && header.cc == RDM_CC_DISC_COMMAND_RESPONSE &&
      header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // RDM discovery responses do not send a DMX break - write immediately
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->flags |= DMX_FLAGS_DRIVER_IS_SENDING;

    size_t write_size = driver->dmx.size;
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
