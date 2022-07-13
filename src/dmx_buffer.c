#include "dmx_buffer.h"

#include "dmx_caps.h"
#include "esp_system.h"
#include "freertos/task.h"

typedef struct DMXBufferDef_t {
  uint16_t head;
  uint16_t triggerLevel;
  TaskHandle_t task_waiting;
  TaskHandle_t calling_task;
  uint8_t data[DMX_MAX_PACKET_SIZE];
  uint8_t completed;
  portMUX_TYPE mux;
} DMXBuffer_t;

DMXBufferHandle_t DMXBufferCreate(size_t triggerLevel) {
  if (triggerLevel == 0) {
    triggerLevel = 1;
  } else if (triggerLevel > DMX_MAX_PACKET_SIZE) {
    triggerLevel = DMX_MAX_PACKET_SIZE;
  }

  // Initialize struct to default values
  DMXBuffer_t *dmx_buf = heap_caps_malloc(sizeof(DMXBuffer_t), 
      (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (dmx_buf == NULL) {
    return NULL;
  }
  dmx_buf->head = 0;
  dmx_buf->triggerLevel = triggerLevel;
  dmx_buf->task_waiting = NULL;
  dmx_buf->calling_task = xTaskGetCurrentTaskHandle();
  bzero(dmx_buf->data, DMX_MAX_PACKET_SIZE);
  dmx_buf->completed = false;
  portMUX_INITIALIZE(&dmx_buf->mux);

  return (DMXBufferHandle_t) dmx_buf;
}

void DMXBufferDelete(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  heap_caps_free(dmx_buf);
}

void DMXBufferClear(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  taskENTER_CRITICAL(&dmx_buf->mux);
  dmx_buf->head = 0;
  bzero(dmx_buf->data, DMX_MAX_PACKET_SIZE);
  taskEXIT_CRITICAL(&dmx_buf->mux);
}

size_t DMXBufferOverwrite(DMXBufferHandle_t DMXBufferHandle, const void *data,
                          size_t size) {
  configASSERT(DMXBufferHandle);
  configASSERT(data);
  if (size == 0) {
    return 0;
  }

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  // Clamp size to the size of the buffer  
  if (size > DMX_MAX_PACKET_SIZE) {
    size = DMX_MAX_PACKET_SIZE;
  }

  memcpy(dmx_buf->data, data, size);

  return size;
}

size_t DMXBufferSendFromFIFOFromISR(DMXBufferHandle_t DMXBufferHandle,
                                    const void *const data, size_t size,
                                    BaseType_t *higherPriorityTaskAwoken) {
  configASSERT(DMXBufferHandle);
  configASSERT(data);
  if (size == 0) {
    return 0;
  }

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;
  
  // Clamp the send size to the number of spaces available
  const size_t spaces_available = DMXBufferSpacesAvailable(DMXBufferHandle);
  if (size > spaces_available) {
    size = spaces_available;
  }

  // Get data from the FIFO, whose pointer doesn't change
  uint8_t *data_head = &dmx_buf->data[dmx_buf->head];
  for (int i = 0; i < size; ++i) data_head[i] = *(uint8_t *)data;
  dmx_buf->head += size;

  // TODO: check if task awoken

  return size;
}

size_t DMXBufferReceiveToFIFOFromISR(DMXBufferHandle_t DMXBufferHandle,
                                     void *const data, size_t size, 
                                     BaseType_t *higherPriorityTaskAwoken) {
  configASSERT(DMXBufferHandle);
  configASSERT(data);
  if (size == 0) {
    return 0;
  }
  
  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  // Clamp the receive size to the number of bytes available
  const size_t bytes_available = DMXBufferBytesAvailable(DMXBufferHandle);
  if (size > bytes_available) {
    size = bytes_available;
  }

  // Write data to the FIFO, whose pointer does change
  uint8_t *data_head = &dmx_buf->data[dmx_buf->head];
  for (int i = 0; i < size; ++i) *(volatile uint32_t *)data = data_head[i];
  dmx_buf->head += size;

  // Notify tasks when trigger has been reached
  if (dmx_buf->triggerLevel > dmx_buf->head) {
    xTaskNotifyFromISR(dmx_buf->calling_task, 0, eSetValueWithOverwrite,
                       higherPriorityTaskAwoken);
    dmx_buf->completed = true;
  }

  return size;
}

size_t DMXBufferPeek(DMXBufferHandle_t DMXBufferHandle, void *data, size_t size,
                     uint32_t *notificationValue, TickType_t ticksToWait) {
  configASSERT(DMXBufferHandle);
  configASSERT(data);
  if (size == 0) {
    return 0;
  }

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;
  
  // Block task if required
  size_t bytes_available;
  if (ticksToWait > 0) {
    taskENTER_CRITICAL(&dmx_buf->mux);
    bytes_available = DMXBufferBytesAvailable(DMXBufferHandle);
    if (bytes_available <= dmx_buf->triggerLevel) { // 5
      
      // Should only be one reader
      configASSERT(dmx_buf->task_waiting == NULL);
      dmx_buf->task_waiting = xTaskGetCurrentTaskHandle();

    }
    taskEXIT_CRITICAL(&dmx_buf->mux);

    if (bytes_available <= dmx_buf->triggerLevel) {
      xTaskNotifyWait(ULONG_MAX, ULONG_MAX, notificationValue, ticksToWait);
      dmx_buf->task_waiting = NULL;

      // Recheck the data available after blocking
      bytes_available = DMXBufferBytesAvailable(DMXBufferHandle);
    }
  } else {
    bytes_available = DMXBufferBytesAvailable(DMXBufferHandle);
  }

  // Clamp size to bytes available
  if (size > bytes_available) {
    size = bytes_available;
  }

  memcpy(dmx_buf->data, data, size);

  return size;
}

void DMXBufferSendCompleted(DMXBufferHandle_t DMXBufferHandle,
                             uint32_t notificationValue) {
  configASSERT(DMXBufferHandle);

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  dmx_buf->completed = true;
  xTaskNotify(dmx_buf->calling_task, notificationValue, eSetValueWithOverwrite);
}

void DMXBufferSendCompletedFromISR(DMXBufferHandle_t DMXBufferHandle,
                                   uint32_t notificationValue,
                                   BaseType_t *higherPriorityTaskAwoken) {
  configASSERT(DMXBufferHandle);

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  dmx_buf->completed = true;
  xTaskNotifyFromISR(dmx_buf->calling_task, notificationValue,
                     eSetValueWithOverwrite, higherPriorityTaskAwoken);

  // Notify task if required
  if (dmx_buf->task_waiting) {
    xTaskNotifyFromISR(dmx_buf->task_waiting, notificationValue,
                       eSetValueWithOverwrite, higherPriorityTaskAwoken);
    dmx_buf->task_waiting = NULL;
  }
}

BaseType_t DMXBufferSetTriggerLevel(DMXBufferHandle_t DMXBufferHandle,
                                    size_t triggerLevel) {
  configASSERT(DMXBufferHandle);
  if (triggerLevel > DMX_MAX_PACKET_SIZE) {
    return pdFALSE;
  }

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  if (triggerLevel == 0) {
    triggerLevel = 1;
  }

  dmx_buf->triggerLevel = triggerLevel;
  
  return pdPASS;
}

BaseType_t DMXBufferReset(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  BaseType_t reset = pdFALSE;

  taskENTER_CRITICAL(&dmx_buf->mux);
  if (dmx_buf->task_waiting == NULL) {
    dmx_buf->head = 0;
    dmx_buf->completed = false;
    reset = pdPASS;
  }
  taskEXIT_CRITICAL(&dmx_buf->mux);

  return reset;
}

size_t DMXBufferBytesAvailable(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  return dmx_buf->head;
}

size_t DMXBufferSpacesAvailable(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  return DMX_MAX_PACKET_SIZE - dmx_buf->head;
}

bool DMXBufferIsCompleted(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBuffer_t *dmx_buf = (DMXBuffer_t *)DMXBufferHandle;

  return dmx_buf->completed;
}
