#pragma once

#include "dmx/struct.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: docs
void dmx_nvs_init(dmx_port_t dmx_num);

/**
 * @brief Gets parameter data from non-volatile storage.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to get.
 * @param ds The parameter data type.
 * @param[out] param A pointer into which to copy the parameter data.
 * @param[inout] size The size of the param pointer. Upon getting the parameter
 * data, this value is set to the size of the gotten parameter.
 * @return ESP_OK on success or any of the ESP_ERR_NVS errors upon failure.
 */
// TODO: return bool, not esp_err_t
bool dmx_nvs_get(dmx_port_t dmx_num, rdm_pid_t pid, rdm_ds_t ds, void *param,
                 size_t *size);

/**
 * @brief Sets the parameter data to non-volatile storage.
 *
 * @param dmx_num The DMX port number.
 * @param pid The parameter ID to set.
 * @param ds The parameter data type.
 * @param[in] param A pointer to the parameter data to copy to NVS.
 * @param size The size of the parameter data.
 * @return ESP_OK on success or any of the ESP_ERR_NVS errors upon failure.
 */
// TODO: return bool, not esp_err_t
bool dmx_nvs_set(dmx_port_t dmx_num, rdm_pid_t pid, rdm_ds_t ds,
                 const void *param, size_t size);

#ifdef __cplusplus
}
#endif
