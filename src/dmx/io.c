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
  int dmx_status;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  dmx_status = driver->dmx.status;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (dmx_status == DMX_STATUS_SENDING) {
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
    xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.progress = DMX_PROGRESS_STALE;
    driver->dmx.head = DMX_HEAD_WAITING_FOR_BREAK;
    dmx_uart_set_rts(dmx_num, 1);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Update the receive size only if it has changed
  if (size != driver->dmx.size) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.size = size;
    if (driver->dmx.head > 0 && !dmx_start_code_is_rdm(driver->dmx.data[0])) {
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
    if (packet != NULL) {
      packet->err = DMX_ERR_TIMEOUT;
      packet->sc = -1;
      packet->size = 0;
      packet->is_rdm = 0;
    }
    xSemaphoreGiveRecursive(driver->mux);
    return 0;
  }

  // Block the task to wait for data to be ready
  dmx_err_t err;
  if (packet_status != DMX_PROGRESS_COMPLETE) {
    // Tell the DMX driver that this task is awaiting a DMX packet
    const TaskHandle_t current_task_handle = xTaskGetCurrentTaskHandle();
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->task_waiting = current_task_handle;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

    // Determine if it is necessary to set a hardware timeout alarm
    int64_t timer_alarm;
    if (driver->is_controller) {
      timer_alarm = RDM_TIMING_CONTROLLER_REQUEST_TO_RESPONSE_MAX;
    } else {
      timer_alarm = 0;
    }

    // Set an alarm to timeout early if an RDM response is expected
    if (timer_alarm > 0) {
      int64_t last_timestamp;
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      last_timestamp = driver->dmx.controller_eop_timestamp;
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
      const int64_t timer_elapsed =
          dmx_timer_get_micros_since_boot() - last_timestamp;
      if (timer_elapsed > timer_alarm) {
        // Return early if the time elapsed is greater than the timer alarm
        if (packet != NULL) {
          packet->err = DMX_ERR_TIMEOUT;
          packet->sc = -1;
          packet->size = 0;
          packet->is_rdm = 0;
        }
        xSemaphoreGiveRecursive(driver->mux);
        return 0;
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
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    packet_size = driver->dmx.head;
    driver->task_waiting = NULL;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (!notified) {
      xTaskNotifyStateClear(current_task_handle);  // Avoid race condition
      if (packet != NULL) {
        packet->err = DMX_ERR_TIMEOUT;
        packet->sc = -1;
        packet->size = 0;
        packet->is_rdm = 0;
      }
      xSemaphoreGiveRecursive(driver->mux);
      dmx_parameter_commit(dmx_num);
      return 0;
    }
  } else {
    /* Errors can be lost if they are not caught by dmx_receive() or 
      dmx_receive_num() but that is acceptable behavior. Errors should only be
      reported if the device is able to handle the packet.*/
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
    return packet_size;
  }

  // Get the RDM header information and update miscellaneous RDM driver fields
  bool is_rdm;
  rdm_header_t header;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  is_rdm = rdm_read_header(dmx_num, &header);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (!rdm_cc_is_valid(header.cc)) {
    is_rdm = false;  // Packet is not RDM if CC is invalid
  }
  const rdm_uid_t *this_uid = rdm_uid_get(dmx_num);
  if (is_rdm) {
    if (packet != NULL) {
      packet->is_rdm = header.pid;
    }
    if (!driver->is_controller) {
      // Packet is an RDM request packet
      driver->dmx.last_controller_pid = header.pid;
      if (rdm_uid_is_target(this_uid, &header.dest_uid)) {
        if (header.pid == driver->dmx.last_request_pid) {
          ++driver->dmx.last_request_pid_repeats;
        } else {
          driver->dmx.last_request_pid = header.pid;
          driver->dmx.last_request_pid_repeats = 0;
        }
      }
    } else {
      // Packet is an RDM response packet
      driver->dmx.last_responder_pid = header.pid;
    }
  } else {
    // Packet is DMX so this device must be a responder
    driver->dmx.last_controller_pid = 0;
  }

  // TODO: Return early if RDM is disabled

  // Return early if the packet isn't a request or this device isn't addressed
  if (!is_rdm || driver->is_controller || !rdm_cc_is_request(header.cc) ||
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
      const int64_t micros_elapsed = dmx_timer_get_micros_since_boot() -
                                     driver->dmx.controller_eop_timestamp;
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
      driver->dmx.head = DMX_HEAD_WAITING_FOR_BREAK;
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
  bool is_rdm;
  rdm_header_t header;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  is_rdm = rdm_read_header(dmx_num, &header);
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (is_rdm && !rdm_cc_is_valid(header.cc)) {
    is_rdm = false;
  }

  // Determine if this device is the controller
  driver->is_controller = !is_rdm || rdm_cc_is_request(header.cc);

  // Determine if it is necessary to set a hardware timeout alarm
  int64_t timer_alarm;
  if (driver->is_controller) {
    if (driver->dmx.last_controller_pid != RDM_PID_DISC_UNIQUE_BRANCH) {
      if (driver->dmx.responder_sent_last ||
          driver->dmx.last_controller_pid == 0 ||
          driver->dmx.last_request_was_broadcast) {
        timer_alarm = RDM_TIMING_CONTROLLER_REQUEST_TO_REQUEST_MIN;
      } else {
        timer_alarm = RDM_TIMING_CONTROLLER_RESPONSE_LOST_MIN;
      }
    } else {
      if (driver->dmx.responder_sent_last) {
        /* This is a condition in which the RDM controller sends an
          RDM_PID_DISC_UNIQUE_BRANCH message and a valid response has already
          been received. The RDM standard doesn't specify how long the RDM
          controller should wait before sending the next RDM request. This
          library makes an inference to what this duration should be in order to
          reduce RDM discovery times. This inference is based on related
          information from the RDM specification.*/
        timer_alarm = RDM_TIMING_CONTROLLER_DISCOVERY_RESPONSE_TO_REQUEST_MIN;
      } else {
        timer_alarm = RDM_TIMING_CONTROLLER_DISCOVERY_TO_REQUEST_MIN;
      }
    }
  } else {
    timer_alarm = RDM_TIMING_RESPONDER_MIN;
  }

  // If necessary, set an alarm to wait the minimum duration before sending
  int64_t timer_elapsed;
  const TaskHandle_t this_task_handle = xTaskGetCurrentTaskHandle();
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  timer_elapsed =
      dmx_timer_get_micros_since_boot() - driver->dmx.controller_eop_timestamp;
  if (timer_elapsed < timer_alarm) {
    dmx_timer_set_counter(dmx_num, timer_elapsed);
    dmx_timer_set_alarm(dmx_num, timer_alarm, false);
    dmx_timer_start(dmx_num);
    driver->task_waiting = this_task_handle;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  // Block if an alarm was set
  if (timer_elapsed < timer_alarm) {
    if (!xTaskNotifyWait(0, ULONG_MAX, NULL, pdDMX_MS_TO_TICKS(20))) {
      __unreachable();  // The hardware timer should always notify the task
    }
    driver->task_waiting = NULL;
  }

  // Return early if it is too late to send a response packet
  if (!driver->is_controller) {
    if (timer_elapsed > RDM_TIMING_RESPONDER_MAX) {
      xSemaphoreGiveRecursive(driver->mux);
      return 0;
    }
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

  // Record information about the packet that is being sent
  if (driver->is_controller) {
    rdm_pid_t pid = is_rdm ? header.pid : 0;
    bool was_broadcast = is_rdm ? rdm_uid_is_broadcast(&header.dest_uid) : 0;
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.last_controller_pid = pid;
    driver->dmx.last_request_was_broadcast = was_broadcast;
    driver->dmx.responder_sent_last = false;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    if (pid != 0) {
      ++driver->rdm.tn;
    }
  } else {
    rdm_pid_t pid = is_rdm ? header.pid : 0;
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.last_responder_pid = pid;
    driver->dmx.responder_sent_last = true;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Determine if a DMX break is required and send the packet
  if (is_rdm && header.cc == RDM_CC_DISC_COMMAND_RESPONSE &&
      header.pid == RDM_PID_DISC_UNIQUE_BRANCH) {
    // RDM discovery responses do not send a DMX break - write immediately
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.status = DMX_STATUS_SENDING;

    int write_len = driver->dmx.size;
    dmx_uart_write_txfifo(dmx_num, driver->dmx.data, &write_len);
    driver->dmx.head = write_len;

    // Enable DMX write interrupts
    dmx_uart_enable_interrupt(dmx_num, DMX_INTR_TX_ALL);
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    // Send the packet by starting the DMX break
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.head = 0;
    driver->dmx.progress = DMX_PROGRESS_IN_BREAK;
    driver->dmx.status = DMX_STATUS_SENDING;
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
    if (driver->dmx.status == DMX_STATUS_SENDING) {
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
    if (driver->dmx.status == DMX_STATUS_SENDING) {
      result = false;
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Give the mutex back and return
  xSemaphoreGiveRecursive(driver->mux);
  return result;
}
