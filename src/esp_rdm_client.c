#include "esp_rdm_client.h"
#include "rdm_types.h"
#include "esp_rdm.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_dmx.h"
#include "private/rdm_encode/functions.h"

rdm_parameters_t rdm_parameters[DMX_NUM_MAX] = {0};

bool rdm_client_init(dmx_port_t dmx_num, uint16_t start_address, uint16_t footprint)
{
    if (dmx_num >= DMX_NUM_MAX)
    {
        ESP_LOGE("rdm_client", "dmx_num too large");
        return false;
    }

    rdm_parameters_t *params = &rdm_parameters[dmx_num];

    params->device_info.major_rdm_version = 1;
    params->device_info.minor_rdm_version = 0;
    params->device_info.model_id = 0x4242;
    params->device_info.coarse_product_category = 0x01; // TODO create constants for product category and fine category
    params->device_info.fine_product_category = 0x00;
    params->device_info.software_version_id = 0x42;
    params->device_info.footprint = footprint;
    params->device_info.current_personality = 1;
    params->device_info.personality_count = 1;
    params->device_info.start_address = start_address;
    params->device_info.sub_device_count = 0;
    params->device_info.sensor_count = 0;
    params->identify_device = false;

    return true;
}

void rdm_client_handle_discovery_command(dmx_port_t dmx_num, const rdm_header_t *header, const void *data, const uint16_t data_size)
{
    if (header->pid == RDM_PID_DISC_UNIQUE_BRANCH)
    {
        // TODO ensure that data_size is large enough
        const rdm_uid_t lowUid = buf_to_uid(data + 24);
        const rdm_uid_t highUid = buf_to_uid(data + 30);
        const rdm_uid_t ourUid = rdm_get_uid(dmx_num);

        if (!rdm_is_muted(dmx_num) && lowUid <= ourUid && ourUid <= highUid)
        {
            const size_t respSize = rdm_send_disc_response(dmx_num, 7, ourUid);
            ESP_LOGI("RDM", "Sent discovery response. %d bytes", respSize);
        }
    }
    else if (header->pid == RDM_PID_DISC_UN_MUTE)
    {
        ESP_LOGI("RDM", "Received UNMUTE");
        rdm_set_muted(dmx_num, false);
        // TODO add response for unmute
    }
    else if (header->pid == RDM_PID_DISC_MUTE)
    {
        ESP_LOGI("RDM", "Received MUTE");
        rdm_set_muted(dmx_num, true);

        // TODO store muteParams in rdm_client_params
        rdm_disc_mute_t muteParams;
        muteParams.managed_proxy = false;
        muteParams.sub_device = false;
        muteParams.boot_loader = false;
        muteParams.proxied_device = false;
        muteParams.binding_uid = 0;
        const size_t bytesSent = rdm_send_mute_response(dmx_num, header->source_uid, header->tn, &muteParams);
        if (bytesSent == 0)
        {
            // TODO print error or something
        }
    }
}

void rdm_client_handle_rdm_message(dmx_port_t dmx_num, const dmx_packet_t *dmxPacket, const void *data, const uint16_t size)
{
    rdm_header_t header;
    if (rdm_get_header(&header, data))
    {
        if (rdm_is_directed_at_us(dmx_num, &header))
        {
            if (header.cc == RDM_CC_DISC_COMMAND)
            {
                rdm_client_handle_discovery_command(dmx_num, &header, data, size);
            }
            else if (header.cc == RDM_CC_GET_COMMAND)
            {
                switch (header.pid)
                {
                case RDM_PID_DEVICE_INFO:
                {
                    uint8_t buffer[0x13]; // FIXME this is not good style
                    const size_t pdl = rdm_encode_device_info_(&buffer, &rdm_parameters[dmx_num].device_info);
                    if (pdl != 0x13)
                    {
                        ESP_LOGE("RDM DBG", "buffer overflow in rdm_encode_device_info_");
                        return;
                    }
                    const size_t bytesSent = rdm_send_get_param_response(dmx_num, header.source_uid, header.tn,
                                                                         RDM_PID_DEVICE_INFO, header.sub_device, buffer, pdl);
                    ESP_LOGI("RDM DBG", "Sent DEVICE_INFO response. %d bytes", bytesSent);
                }
                break;
                case RDM_PID_IDENTIFY_DEVICE:
                {
                    const rdm_parameters_t *params = &rdm_parameters[dmx_num];
                    const size_t bytesSent = rdm_send_get_param_response(dmx_num, header.source_uid, header.tn,
                                                                         RDM_PID_IDENTIFY_DEVICE, header.sub_device, &params->identify_device, 1);

                    ESP_LOGI("RDM DBG", "Sent IDENTIFY_DEVICE response. %d bytes", bytesSent);
                }
                break;
                case RDM_PID_DEVICE_LABEL:
                {
                }
                break;
                default:
                    ESP_LOGI("RDM DBG", " get pid: %04x", header.pid);
                }
            }
            else if (header.cc == RDM_CC_SET_COMMAND)
            {
                switch (header.pid)
                {
                case RDM_PID_IDENTIFY_DEVICE:
                {
                    // TODO notify someone that identify changed!
                    rdm_parameters[dmx_num].identify_device = ((uint8_t *)data)[24];
                    const size_t bytesSent = rdm_send_set_command_ack_response(dmx_num, header.source_uid, header.tn, header.sub_device, RDM_PID_IDENTIFY_DEVICE);
                    ESP_LOGI("RDM DBG", "Sent SET IDENTIFY_DEVICE response. %d bytes", bytesSent);
                    ESP_LOGI("RDM DBG", "Set identify: %d", rdm_parameters[dmx_num].identify_device);
                }
                break;
                default:
                    ESP_LOGI("RDM DBG", " RDM_CC_SET_COMMAND get pid: %04x", header.pid);
                }
            }
            else
            {
                // debugPrintRdm(header);
            }
        }
    }
}

void rdm_client_set_device_info(dmx_port_t dmx_num, const rdm_device_info_t *info)
{
    //   if(dmx_num >= DMX_NUM_MAX)
    //   {
    //     ESP_LOGE("rdm_client", "dmx_num too large");
    //     return;
    //   }

    //   rdm_parameters[dmx_num] = *info;
}

// void debugPrintRdm(const rdm_header_t header)
// {
//     switch (header.cc)
//     {
//     case RDM_CC_GET_COMMAND:
//         ESP_LOGI("RDM DBG", " command class: GET_COMMAND");
//         break;
//     case RDM_CC_SET_COMMAND:
//         ESP_LOGI("RDM DBG", " command class: SET_COMMAND");
//         break;
//     default:
//         ESP_LOGI("RDM DBG", " command class: UNKNOWN %02x", header.cc);
//     }
//     ESP_LOGI("RDM DBG", " pid: %04x", header.pid);
// }