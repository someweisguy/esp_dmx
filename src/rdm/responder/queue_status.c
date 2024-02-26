#include "include/queue_status.h"

#include "dmx/include/driver.h"
#include "dmx/include/service.h"
#include "rdm/include/driver.h"
#include "rdm/responder/include/utils.h"

/**
 * @brief The implementation for the RDM queue. The RDM queue is implemented
 * using a deque with an additional field to recall the previous PID which was
 * popped from the deque.
 */
typedef struct rdm_queue_t {
  uint16_t head;       // The head of the deque.
  uint16_t tail;       // The tail of the deque.
  rdm_pid_t previous;  // The PID that was previously popped from the RDM queue.
  uint16_t max_size;   // The maximum size of the RDM queue.
  rdm_pid_t data[];    // The buffer containing the RDM queue information.
} rdm_queue_t;

static rdm_queue_t *rdm_get_queue(dmx_port_t dmx_num) {
  return dmx_parameter_get_data(dmx_num, RDM_SUB_DEVICE_ROOT,
                           RDM_PID_QUEUED_MESSAGE);
}

static size_t rdm_rhd_get_queued_message(
    dmx_port_t dmx_num, const rdm_parameter_definition_t *definition,
    const rdm_header_t *header) {
  if (header->sub_device != RDM_SUB_DEVICE_ROOT) {
    // Requests may only be made to the root device
    return rdm_write_nack_reason(dmx_num, header,
                                 RDM_NR_SUB_DEVICE_OUT_OF_RANGE);
  }

  // Verify status type is valid
  uint8_t status_type;
  if (!rdm_read_pd(dmx_num, definition->get.request.format, &status_type,
                   sizeof(status_type))) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_FORMAT_ERROR);
  }
  if (status_type < definition->min_value ||
      status_type > definition->max_value) {
    return rdm_write_nack_reason(dmx_num, header, RDM_NR_DATA_OUT_OF_RANGE);
  }

  // Determine what the response will be depending on the RDM queue size
  rdm_pid_t pid;
  rdm_header_t response_header = *header;
  const rdm_parameter_definition_t *response_definition;
  if (rdm_queue_size(dmx_num) > 0) {
    pid = rdm_queue_pop(dmx_num);
    response_definition = rdm_definition_get(dmx_num, RDM_SUB_DEVICE_ROOT, pid);
    assert(response_definition != NULL);
  } else {
    pid = RDM_PID_STATUS_MESSAGE;
    response_definition = rdm_definition_get(dmx_num, RDM_SUB_DEVICE_ROOT, pid);
    if (response_definition == NULL) {
      response_header.pid = RDM_PID_STATUS_MESSAGE;
      return rdm_write_ack(dmx_num, &response_header, NULL, NULL, 0);
    }
  }

  response_header.pid = pid;
  return response_definition->get.handler(dmx_num, response_definition,
                                          &response_header);
}

bool rdm_register_queued_message(dmx_port_t dmx_num, uint32_t max_count,
                                 rdm_callback_t cb, void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(max_count > 0, false, "max_count error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  const rdm_pid_t pid = RDM_PID_QUEUED_MESSAGE;

  // Add the parameter
  const size_t size = sizeof(rdm_queue_t) + (sizeof(rdm_pid_t) * max_count);
  if (!dmx_parameter_add(dmx_num, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_DYNAMIC, NULL, size)) {
    return false;
  }
  rdm_queue_t *queue = rdm_get_queue(dmx_num);
  assert(queue != NULL);
  queue->max_size = max_count;

  // Define the parameter
  static const rdm_parameter_definition_t definition = {
      .pid_cc = RDM_CC_GET,
      .ds = RDM_DS_NOT_DEFINED,
      .get = {.handler = rdm_rhd_get_queued_message,
              .request.format = NULL,
              .response.format = "b$"},
      .set = {.handler = NULL, .request.format = NULL, .response.format = NULL},
      .pdl_size = sizeof(uint8_t),
      .max_value = RDM_STATUS_ERROR,
      .min_value = RDM_STATUS_GET_LAST_MESSAGE,
      .units = RDM_UNITS_NONE,
      .prefix = RDM_PREFIX_NONE,
      .description = NULL};
  rdm_definition_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, &definition);

  return rdm_callback_set(dmx_num, RDM_SUB_DEVICE_ROOT, pid, cb, context);
}

bool rdm_queue_push(dmx_port_t dmx_num, rdm_pid_t pid) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(pid > 0, false, "pid error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  rdm_queue_t *queue = rdm_get_queue(dmx_num);
  if (queue == NULL) {
    return false;
  }

  bool success = false;
  bool already_queued = false;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  // Iterate the queue to ensure that the PID isn't already queued
  for (int i = queue->tail; i != queue->head; ++i) {
    if (i == queue->max_size) {
      i = 0;
    }
    if (queue->data[i] == pid) {
      already_queued = true;
      success = true;
      break;
    }
  }

  // Push the new PID and increment the queue head
  if (!already_queued && (queue->head + 1) % queue->max_size != queue->tail) {
    queue->data[queue->head] = pid;
    ++queue->head;
    if (queue->head == queue->max_size) {
      queue->head = 0;
    }
    success = true;
  }
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return success;
}

rdm_pid_t rdm_queue_pop(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  rdm_pid_t pid;
  if (rdm_queue_size(dmx_num) > 0) {
    rdm_queue_t *queue = rdm_get_queue(dmx_num);
    assert(queue != NULL);
    taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
    pid = queue->data[queue->tail];
    ++queue->tail;
    if (queue->tail == queue->max_size) {
      queue->tail = 0;
    }
    queue->previous = pid;
    taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  } else {
    pid = 0;
  }

  return pid;
}

uint8_t rdm_queue_size(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  rdm_queue_t *queue = rdm_get_queue(dmx_num);
  if (queue == NULL) {
    return 0;
  }

  // Get the queue size by comparing the head and tail
  uint16_t head;
  uint16_t tail;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  head = queue->head;
  tail = queue->tail;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));
  int32_t size = -tail + head;
  if (size < 0) {
    size += queue->max_size;
  }

  // Clamp the queue size to 255
  if (size > 255) {
    size = 255;
  }

  return size;
}

rdm_pid_t rdm_queue_previous(dmx_port_t dmx_num) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, 0, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), 0, "driver is not installed");

  rdm_queue_t *queue = rdm_get_queue(dmx_num);
  if (queue == NULL) {
    return 0;
  }

  rdm_pid_t pid;
  taskENTER_CRITICAL(DMX_SPINLOCK(dmx_num));
  pid = queue->previous;
  taskEXIT_CRITICAL(DMX_SPINLOCK(dmx_num));

  return pid;
}