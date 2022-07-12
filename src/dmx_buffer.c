#include "dmx_buffer.h"

#include "dmx_caps.h"
#include "esp_system.h"
#include "freertos/task.h"

typedef struct {
  uint16_t head;
  uint16_t triggerLevel;
  TaskHandle_t task_waiting;
  uint8_t data[DMX_MAX_PACKET_SIZE];
  uint8_t completed;
  portMUX_TYPE mux;
} DMXBufferDef_t;

DMXBufferHandle_t DMXBufferCreate(size_t triggerLevel) {
  if (triggerLevel == 0) {
    triggerLevel = 1;
  } else if (triggerLevel > DMX_MAX_PACKET_SIZE) {
    triggerLevel = DMX_MAX_PACKET_SIZE;
  }

  // Initialize struct to default values
  DMXBufferDef_t *dmx_buf = heap_caps_malloc(sizeof(DMXBufferDef_t), 
      (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (dmx_buf == NULL) {
    return NULL;
  }
  dmx_buf->head = 0;
  dmx_buf->triggerLevel = triggerLevel;
  dmx_buf->task_waiting = NULL;
  bzero(dmx_buf->data, DMX_MAX_PACKET_SIZE);
  dmx_buf->completed = false;
  portMUX_INITIALIZE(&dmx_buf->mux);

  return (DMXBufferHandle_t) dmx_buf;
}

void DMXBufferDelete(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;

  heap_caps_free(dmx_buf);
}

size_t DMXBufferOverwrite(DMXBufferHandle_t DMXBufferHandle, const void *data,
                          size_t size, TickType_t ticksToWait) {
  configASSERT(DMXBufferHandle);
  configASSERT(data);
  if (size == 0) {
    return 0;
  }

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;
  
  if (size > DMX_MAX_PACKET_SIZE) {
    size = DMX_MAX_PACKET_SIZE;
  }

  // TODO: block if (!dmx_buf->completed)
  // Ensures writes to the buffer are synchronized

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

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;
  
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
  
  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;

  // Clamp the receive size to the number of bytes available
  const size_t bytes_available = DMXBufferBytesAvailable(DMXBufferHandle);
  if (size > bytes_available) {
    size = bytes_available;
  }

  // Write data to the FIFO, whose pointer does change
  uint8_t *data_head = &dmx_buf->data[dmx_buf->head];
  for (int i = 0; i < size; ++i) *(uint8_t *)data = data_head[i];
  dmx_buf->head += size;

  // Notify tasks when trigger has been reached
  if (dmx_buf->triggerLevel > dmx_buf->head && dmx_buf->task_waiting) {
    xTaskNotifyFromISR(dmx_buf->task_waiting, 0, eSetValueWithOverwrite,
                       higherPriorityTaskAwoken);
    dmx_buf->task_waiting = NULL;
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

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;
  
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
      xTaskNotifyWait(ULONG_MAX, 0, notificationValue, ticksToWait);
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

void DMXBufferSendCompletedFromISR(DMXBufferHandle_t DMXBufferHandle,
                                   uint32_t notificationValue,
                                   BaseType_t *higherPriorityTaskAwoken) {
  configASSERT(DMXBufferHandle);

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;

  // Notify task if required
  if (dmx_buf->task_waiting) {
    xTaskNotifyFromISR(dmx_buf->task_waiting, notificationValue,
                      eSetValueWithOverwrite, higherPriorityTaskAwoken);
    dmx_buf->task_waiting = NULL;
  }
  dmx_buf->completed = true;
}

BaseType_t DMXBufferSetTriggerLevel(DMXBufferHandle_t DMXBufferHandle,
                                    size_t triggerLevel) {
  configASSERT(DMXBufferHandle);
  if (triggerLevel > DMX_MAX_PACKET_SIZE) {
    return pdFALSE;
  }

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;

  if (triggerLevel == 0) {
    triggerLevel = 1;
  }

  dmx_buf->triggerLevel = triggerLevel;
  
  return pdPASS;
}

BaseType_t DMXBufferReset(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;

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

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;

  return dmx_buf->head;
}

size_t DMXBufferSpacesAvailable(DMXBufferHandle_t DMXBufferHandle) {
  configASSERT(DMXBufferHandle);

  DMXBufferDef_t *dmx_buf = (DMXBufferDef_t *)DMXBufferHandle;

  return DMX_MAX_PACKET_SIZE - dmx_buf->head;
}
