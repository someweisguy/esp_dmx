#include "rdm/responder.h"

#include <string.h>

#include "dmx/hal/include/timer.h"
#include "dmx/hal/include/uart.h"
#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/include/types.h"
#include "rdm/include/uid.h"

bool rdm_send_response(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  dmx_driver_t *const driver = dmx_driver[dmx_num];

  if (!xSemaphoreTakeRecursive(driver->mux, 0)) {
    return false;
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
    // Packet is an RDM request packet
    if (rdm_uid_is_target(this_uid, &header.dest_uid)) {
      if (header.pid == driver->dmx.last_request_pid) {
        ++driver->dmx.last_request_pid_repeats;
      } else {
        driver->dmx.last_request_pid_repeats = 0;
      }
    }
  }

  // Return early if this packet isn't relevant to this device
  if (!is_rdm || !rdm_uid_is_target(this_uid, &header.dest_uid)) {
    xSemaphoreGiveRecursive(driver->mux);
    return false;
  }

  // Update PID of the last request to target this device
  driver->dmx.last_request_pid = header.pid;

  // Get parameter definition
  size_t packet_size;  // Size of the response packet
  const rdm_parameter_definition_t *def =
      rdm_definition_get(dmx_num, header.sub_device, header.pid);
  if (def == NULL) {
    // Unknown PID
    packet_size = rdm_write_nack_reason(dmx_num, &header, RDM_NR_UNKNOWN_PID);
  } else if ((header.sub_device >= RDM_SUB_DEVICE_MAX &&
              header.sub_device != RDM_SUB_DEVICE_ALL) ||
             header.pdl >= RDM_PD_SIZE_MAX) {
    // Header format is invalid
    packet_size = rdm_write_nack_reason(dmx_num, &header, RDM_NR_FORMAT_ERROR);
  } else {
    // Request is valid, handle the response

    // Validate the header against definition information
    const rdm_pid_cc_t pid_cc = def->pid_cc;
    if (pid_cc == RDM_CC_DISC && header.cc != RDM_CC_DISC_COMMAND) {
      packet_size = 0;  // Cannot send NACK to RDM_CC_DISC_COMMAND
    } else if ((pid_cc == RDM_CC_GET_SET && header.cc == RDM_CC_DISC_COMMAND) ||
               (pid_cc == RDM_CC_GET && header.cc != RDM_CC_GET_COMMAND) ||
               (pid_cc == RDM_CC_SET && header.cc != RDM_CC_SET_COMMAND)) {
      // Unsupported command class
      packet_size = rdm_write_nack_reason(dmx_num, &header,
                                          RDM_NR_UNSUPPORTED_COMMAND_CLASS);
    } else {
      // Call the response handler for the parameter
      if (header.cc == RDM_CC_SET_COMMAND) {
        packet_size = def->set.handler(dmx_num, def, &header);
      } else {
        // RDM_CC_DISC_COMMAND uses get.handler()
        packet_size = def->get.handler(dmx_num, def, &header);
      }

      // Validate the response
      if (header.pid == RDM_PID_DISC_UNIQUE_BRANCH &&
          ((packet_size > 0 && packet_size < 17) || packet_size > 24)) {
        // Invalid RDM_CC_DISC_COMMAND_RESPONSE packet size
        packet_size = 0;  // Silence invalid discovery responses
        rdm_set_boot_loader(dmx_num);
      } else if (packet_size > 255 ||
                 (packet_size == 0 &&
                  !rdm_uid_is_broadcast(&header.dest_uid))) {
        // Response size is too large or zero after a non-broadcast request
        packet_size =
            rdm_write_nack_reason(dmx_num, &header, RDM_NR_HARDWARE_FAULT);
        rdm_set_boot_loader(dmx_num);
      }
    }
  }

  // Do not send a response to non-discovery broadcast packets
  if (rdm_uid_is_broadcast(&header.dest_uid) &&
      header.pid != RDM_PID_DISC_UNIQUE_BRANCH) {
    packet_size = 0;
  }

  // Send the RDM response
  if (packet_size > 0) {
    if (!dmx_send_num(dmx_num, packet_size)) {
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
          header.pid, cc_str, packet_size, micros_elapsed);
    } else {
      dmx_wait_sent(dmx_num, dmx_ms_to_ticks(23));
      taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
      driver->dmx.head = DMX_HEAD_WAITING_FOR_BREAK;
      dmx_uart_set_rts(dmx_num, 1);
      taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
    }
  }

  // Call the after-response callback
  const dmx_parameter_t *parameter =
      dmx_parameter_get_entry(dmx_num, header.sub_device, header.pid);
  if (parameter != NULL && parameter->callback != NULL) {
    rdm_header_t response_header;
    if (!rdm_read_header(dmx_num, &response_header)) {
      // Set the response header to NULL if an RDM header can't be read
      memset(&response_header, 0, sizeof(response_header));
    }
    parameter->callback(dmx_num, &header, &response_header, parameter->context);
  }

  return (packet_size > 0);
}