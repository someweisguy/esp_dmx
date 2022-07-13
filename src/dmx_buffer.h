#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include <stdint.h>
#include <string.h>

struct DMXBuffer_t;
typedef struct DMXBuffer_t *DMXBufferHandle_t;

DMXBufferHandle_t DMXBufferCreate(size_t triggerLevel);

void DMXBufferDelete(DMXBufferHandle_t DMXBufferHandle);

void DMXBufferClear(DMXBufferHandle_t DMXBufferHandle);

size_t DMXBufferOverwrite(DMXBufferHandle_t DMXBufferHandle, const void *data,
                          size_t size);

// TODO: overwrite slot

size_t DMXBufferSendFromFIFOFromISR(DMXBufferHandle_t DMXBufferHandle,
                                    const void *const data, size_t size,
                                    BaseType_t *higherPriorityTaskAwoken);

size_t DMXBufferReceiveToFIFOFromISR(DMXBufferHandle_t DMXBufferHandle,
                                     void *const data, size_t size,
                                     BaseType_t *higherPriorityTaskAwoken);

size_t DMXBufferPeek(DMXBufferHandle_t DMXBufferHandle, void *data, size_t size,
                     uint32_t *notificationValue, TickType_t ticksToWait);

// TODO: peek slot

void DMXBufferSendCompleted(DMXBufferHandle_t DMXBufferHandle,
                            uint32_t notificationValue);

void DMXBufferSendCompletedFromISR(DMXBufferHandle_t DMXBufferHandle,
                                   uint32_t notificationValue,
                                   BaseType_t *higherPriorityTaskAwoken);

BaseType_t DMXBufferSetTriggerLevel(DMXBufferHandle_t DMXBufferHandle,
                                    size_t triggerLevel);

BaseType_t DMXBufferReset(DMXBufferHandle_t DMXBufferHandle);

size_t DMXBufferBytesAvailable(DMXBufferHandle_t DMXBufferHandle);

size_t DMXBufferSpacesAvailable(DMXBufferHandle_t DMXBufferHandle);

// TODO: find delimiter

bool DMXBufferIsCompleted(DMXBufferHandle_t DMXBufferHandle);

#ifdef __cplusplus
}
#endif
