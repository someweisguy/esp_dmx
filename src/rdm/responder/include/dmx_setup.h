/**
 * @file rdm/responder/dmx_setup.h
 * @author Mitch Weisbrod
 * @brief // TODO
 */
#pragma once

#include "dmx/include/types.h"
#include "rdm/include/types.h"
#include "rdm/responder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief This parameter is used to set the responder's DMX personality. Many
 * devices such as moving lights have different DMX personalities. Many RDM
 * parameters may be affected by changing personality.
 *
 * The DMX personality can also be retrieved as part of the RDM_PID_DEVICE_INFO
 * parameter message.
 *
 * @param dmx_num The DMX port number.
 * @param personality_count The number of personalities to register.
 * @param cb A callback which is called after receiving a request for this
 * parameter.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the parameter was registered.
 * @return false on failure.
 */
bool rdm_register_dmx_personality(dmx_port_t dmx_num, uint8_t personality_count,
                                  rdm_callback_t cb, void *context);

// TODO docs
size_t rdm_get_dmx_personality(dmx_port_t dmx_num,
                               rdm_dmx_personality_t *personality);

// TODO: docs
bool rdm_set_dmx_personality(dmx_port_t dmx_num, uint8_t personality_num);

/**
 * @brief This parameter is used to get a descriptive ASCII text label for a
 * given DMX personality. The label may be up to 32 characters.
 *
 * @param dmx_num The DMX port number.
 * @param personalities A pointer to an array of personalities to register.
 * @param count The number of personalities in the personalities array.
 * @param cb A callback which is called after receiving a request for this
 * parameter.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the parameter was registered.
 * @return false on failure.
 */
bool rdm_register_dmx_personality_description(
    dmx_port_t dmx_num, rdm_dmx_personality_description_t *personalities,
    uint32_t count, rdm_callback_t cb, void *context);

// TODO: docs
size_t rdm_get_dmx_personality_description(
    dmx_port_t dmx_num, uint8_t personality_num,
    rdm_dmx_personality_description_t *personality_description);

/**
 * @brief This parameter is used to set or get the DMX512 start address.
 * The DMX starting address can also be retrieved as part of the
 * RDM_PID_DEVICE_INFO parameter message.
 *
 * When this message is directed to a root device or sub-device that has a DMX
 * footprint of 0 for that root or sub-device, then the response shall be set to
 * 0xFFFF.
 *
 * @param dmx_num The DMX port number.
 * @param cb A callback which is called upon receiving a request for this PID.
 * @param[inout] context A pointer to context which is used in the user
 * callback.
 * @return true if the PID response was registered.
 * @return false if there is not enough memory to register additional responses.
 */
bool rdm_register_dmx_start_address(dmx_port_t dmx_num, rdm_callback_t cb,
                                    void *context);

/**
 * @brief Gets a copy of the DMX start address of this device.
 *
 * @param dmx_num The DMX port number.
 * @param[out] dmx_start_address A pointer which stores a copy of the DMX start
 * address of this device.
 * @return true on success.
 * @return false on failure.
 */  // TODO: update docs
size_t rdm_get_dmx_start_address(dmx_port_t dmx_num,
                                 uint16_t *dmx_start_address);

/**
 * @brief Sets the DMX start address of this device. The DMX start address of
 * this device may not be set when the DMX start address is set to
 * DMX_START_ADDRESS_NONE.
 *
 * @param dmx_num The DMX port number.
 * @param dmx_start_address The DMX start address to which to set this device.
 * @return true on success.
 * @return false on failure.
 */
bool rdm_set_dmx_start_address(dmx_port_t dmx_num,
                               const uint16_t dmx_start_address);

#ifdef __cplusplus
}
#endif
