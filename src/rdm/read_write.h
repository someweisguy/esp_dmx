#pragma once

#include <stdint.h>
#include <string.h>

#include "endian.h"
#include "esp_dmx.h"
#include "rdm/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief A packed struct which can be used to help process raw RDM packets
 * instead of reading slots by index alone. RDM sends data in most-significant
 * byte first, so endianness must be swapped when using values larger than 8
 * bits.
 */
typedef struct __attribute__((__packed__)) rdm_data_t {
  uint8_t sc;                  // This field shall contain the defined RDM start code. Controllers and responders shall always send RDM_SC in this slot.
  uint8_t sub_sc;              // This field shall contain the sub-start code within RDM that defines this packet structure. Unless specified in future version, the sub-start code shall be equal to RDM_SUB_SC.
  uint8_t message_len;         // The message length value is defined as the number of slots int he RDM packet including the start code and excluding the checksum.
  uint8_t destination_uid[6];  // The UID of the target device(s).
  uint8_t source_uid[6];       // The UID of the device originating this packet.
  uint8_t tn;                  // The RDM transaction number. Controllers increment this field every time an RDM packet is transmitted. Responders set their transaction number to the transaction number of the packet to which they are responding.
  union {
    uint8_t port_id;           // The port ID field shall be set in the range 1-255 identifying the controller port being used, such that the combination of source UID and port ID will uniquely identify the controller and port where the message originated.
    uint8_t response_type;     // The response type field is used in messages from responders to indicate the acknowledgement type of the response.
  };
  uint8_t message_count;       // The message count field is used by a responder to indicate that additional data is now available for collection by a controller. The message count shall be set to 0 in all controller generated requests.
  uint16_t sub_device;         // Sub-devices should be used in devices containing a repetitive number of similar modules, such as a dimmer rack.
  uint8_t cc;                  // The command class (CC) specifies the action of the message. 
  uint16_t pid;                // The parameter ID (PID) identifies a specific type of parameter data.
  uint8_t pdl;                 // The parameter data length (PDL) is the number of slots included in the parameter data area that it precedes.
  struct {
  } pd;                        // The parameter data (PD) is of variable length. The content format is PID dependent.
} rdm_data_t;

// TODO: docs
bool rdm_read(dmx_port_t dmx_num, rdm_header_t *header, rdm_mdb_t *mdb);

// TODO: docs
size_t rdm_write(dmx_port_t dmx_num, const rdm_header_t *header,
                 const rdm_mdb_t *mdb);

/**
 * @brief Encode an array of 16-bit numbers into the desired array.
 *
 * @param[out] pd The buffer in which to encode the data.
 * @param[in] data A pointer to an array of values to encode.
 * @param size The size of the array of values.
 * @return The number of bytes encoded.
 */
size_t rdm_encode_16bit(void *pd, const void *data, int size);

/**
 * @brief Decode a string from a buffer.
 *
 * @param pd A buffer in which the to decode is stored.
 * @param data A pointer into which to store the decoded data.
 * @param size The size of the buffer to store the decoded data.
 * @return The number of characters that was decoded.
 */
int rdm_decode_string(const void *pd, void *data, int size);

/**
 * @brief Decodes RDM device info.
 *
 * @param[in] pd The buffer in which the data to decode is stored.
 * @param[out] data A pointer to a device info parameter to store decoded data.
 * @param size The size of the array to store decoded data.
 * @return The number of parameters decoded (always 1).
 */
int rdm_decode_device_info(const void *pd, void *data, int size);

// TODO: docs
size_t rdm_encode_nack_reason(rdm_mdb_t *mdb, rdm_nr_t nack_reason);

// TODO: docs
int rdm_decode_uids(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_mute(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
size_t rdm_encode_device_info(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
size_t rdm_encode_string(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
size_t rdm_encode_8bit(rdm_mdb_t *mdb, const void *data, int num);

// TODO: docs
int rdm_decode_8bit(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
int rdm_decode_16bit(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
int rdm_decode_mute(const rdm_mdb_t *mdb, void *data, int num);

// TODO: docs
size_t rdm_encode_uids(rdm_mdb_t *mdb, const void *data, int num);

#ifdef __cplusplus
}
#endif
