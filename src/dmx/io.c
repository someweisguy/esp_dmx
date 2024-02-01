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
  int old_size;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  old_size = driver->dmx.size;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  if (size != old_size) {
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    driver->dmx.size = size;
    if (driver->dmx.progress != DMX_PROGRESS_STALE &&
        driver->dmx.head >= size) {
      driver->dmx.progress = DMX_PROGRESS_COMPLETE;
    }
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  }

  // Guard against condition where this task cannot block and data isn't ready
  int packet_status;
  int packet_size;
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
    // TODO: store error code on the driver
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
    packet->is_rdm = dmx_start_code_is_rdm(packet->sc);
  }

  xSemaphoreGiveRecursive(driver->mux);
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
  if (!dmx_wait_sent(dmx_num, dmx_ms_to_ticks(23))) {
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
          controller should wait before sending the next RDM request. Therefore
          this value is customizable by the user in the Kconfig.*/
        timer_alarm = RDM_TIMING_CONTROLLER_DISCOVERY_TRANSACTION_MIN;
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
    if (!xTaskNotifyWait(0, ULONG_MAX, NULL, dmx_ms_to_ticks(20))) {
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

  return dmx_send_num(dmx_num, DMX_PACKET_SIZE);
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
