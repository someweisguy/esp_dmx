#pragma once

#include <stdint.h>

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

/**
 * @brief A packed struct which can be used to help process RDM discovery mute
 * parameters. RDM sends data in most-significant byte first, so endianness must
 * be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_disc_mute_data_t {
  union {
    struct {
      uint8_t managed_proxy : 1;
      uint8_t sub_device : 1;
      uint8_t boot_loader : 1;
      uint8_t proxied_device : 1;
    };
    uint16_t control_field;
  };
  uint8_t binding_uid[6];
} rdm_disc_mute_data_t;

/**
 * @brief A packed struct which can be used to help process RDM device info
 * parameters. RDM sends data in most-significant byte first, so endianness must
 * be swapped when using values larger than 8 bits.
 */
typedef struct __attribute__((__packed__)) rdm_device_info_data_t {
  uint8_t major_rdm_version;
  uint8_t minor_rdm_version;
  uint16_t model_id;
  uint8_t coarse_product_category;
  uint8_t fine_product_category;
  uint32_t software_version_id;
  uint16_t footprint;
  uint8_t current_personality;
  uint8_t personality_count;
  uint16_t start_address;
  uint16_t sub_device_count;
  uint8_t sensor_count;
} rdm_device_info_data_t;

#ifdef __cplusplus
}
#endif
