#pragma once
#include <stdbool.h>
#include "rdm_types.h"
#include "dmx_types.h"

//TODO comment
bool rdm_client_init(dmx_port_t dmx_num, uint16_t start_address, uint16_t footprint, const char* device_label);

//TODO
void rdm_client_handle_rdm_message(dmx_port_t dmx_num, const dmx_packet_t *dmxPacket, const void *data, const uint16_t size);