#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_dmx.h"
#include "esp_log.h"
#include "esp_system.h"

#define TX_PIN 17 // the pin we are using to TX with
#define RX_PIN 16 // the pin we are using to RX with
#define EN_PIN 21 // the pin we are using to enable TX on the DMX transceiver

static const char *TAG = "main";
static uint8_t data[DMX_MAX_PACKET_SIZE] = {};

void handleRdmEvent(const dmx_event_t *event, const void *data, const uint16_t size);

void dmxTask(void *unused)
{
    ESP_ERROR_CHECK(dmx_set_pin(DMX_NUM_2, TX_PIN, RX_PIN, EN_PIN));
    ESP_ERROR_CHECK(dmx_driver_install(DMX_NUM_2, DMX_DEFAULT_INTR_FLAGS));

    while (true)
    {
        dmx_event_t event;
        size_t ret = 0;
        ret = dmx_receive(DMX_NUM_2, &event, DMX_TIMEOUT_TICK);
        if (ret)
        {
            if (event.err == ESP_OK)
            {
                if (event.is_rdm)
                {
                    dmx_read(DMX_NUM_2, data, event.size);
                    handleRdmEvent(&event, data, event.size);
                }
            }
            else
            {
                ESP_LOGE(TAG, "dmx error: %s", esp_err_to_name(event.err));
            }
        }
    }
}


void handleRdmEvent(const dmx_event_t *event, const void *data, const uint16_t size)
{
    rdm_header_t header;
    if (rdm_get_header(&header, data))
    {
        if (rdm_is_directed_at_us(DMX_NUM_2, &header))
        {
            if (header.cc == RDM_CC_DISC_COMMAND)
            {
                if (header.pid == RDM_PID_DISC_UNIQUE_BRANCH)
                {
                    const rdm_uid_t lowUid = buf_to_uid(data + 24);
                    const rdm_uid_t highUid = buf_to_uid(data + 30);
                    const rdm_uid_t ourUid = rdm_get_uid(DMX_NUM_2);

                    if (!rdm_is_muted(DMX_NUM_2) && lowUid <= ourUid && ourUid <= highUid)
                    {
                        const size_t respSize = rdm_send_disc_response(DMX_NUM_2, 7, ourUid);
                        ESP_LOGI("RDM", "Sent discovery response. %d bytes", respSize);
                    }
                }
                else if (header.pid == RDM_PID_DISC_UN_MUTE)
                {
                    ESP_LOGI("RDM", "Received UNMUTE");
                    rdm_set_muted(DMX_NUM_2, false);
                    //TODO add response for unmute

                }
                else if (header.pid == RDM_PID_DISC_MUTE)
                {
                    ESP_LOGI("RDM", "Received MUTE");
                    rdm_set_muted(DMX_NUM_2, true);
                    const size_t bytesSent = rdm_send_mute_response(DMX_NUM_2, header.source_uid, header.tn);
                }
            }
        }
    }
}

void app_main()
{
    TaskHandle_t dmxTaskHandle = NULL;
    xTaskCreatePinnedToCore(dmxTask, "DMX_TASK", 10240, NULL, 2, &dmxTaskHandle, 1);
    if (!dmxTaskHandle)
    {
        ESP_LOGE(TAG, "Failed to create dmx task");
    }
}