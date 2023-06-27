#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_DMX_ISR_IN_IRAM
#define DMX_ISR_ATTR IRAM_ATTR
#else
#define DMX_ISR_ATTR
#endif

#ifndef CONFIG_RDM_RESPONDER_MAX_PARAMETERS
#define CONFIG_RDM_RESPONDER_MAX_PARAMETERS 16
#endif
/** @brief The maximum number of parameters that the RDM responder can
 * support. This value is editable in the Kconfig.*/
#define RDM_RESPONDER_MAX_PIDS (8 + CONFIG_RDM_RESPONDER_MAX_PARAMETERS)

#ifndef CONFIG_DMX_MAX_PERSONALITIES
#define CONFIG_DMX_MAX_PERSONALITIES 16
#endif

#ifndef CONFIG_DMX_MAX_PERSONALITIES
#define CONFIG_DMX_MAX_PERSONALITIES 16
#endif

/**
 * @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID (as
 * long as it is used responsibly) or may choose to register their own
 * manufacturer ID.
 */
#define RDM_MAN_ID_DEFAULT (0x05e0)

#ifndef CONFIG_RDM_DEVICE_UID_MAN_ID
#define CONFIG_RDM_DEVICE_UID_MAN_ID RDM_MAN_ID_DEFAULT
#endif
#ifndef CONFIG_RDM_DEVICE_UID_DEV_ID
#define CONFIG_RDM_DEVICE_UID_DEV_ID 0xffffffff
#endif

#ifdef __cplusplus
}
#endif
