#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_DMX_ISR_IN_IRAM
#define DMX_ISR_ATTR IRAM_ATTR
#else
#define DMX_ISR_ATTR
#endif

#ifdef CONFIG_DMX_MAX_PERSONALITIES
#define DMX_MAX_PERSONALITIES (CONFIG_DMX_MAX_PERSONALITIES)
#else
#define DMX_MAX_PERSONALITIES (16)
#endif

#ifdef CONFIG_RDM_DEVICE_UID_MAN_ID
#define RDM_UID_MANUFACTURER_ID (CONFIG_RDM_DEVICE_UID_MAN_ID)
#else
/**
 * @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID (as
 * long as it is used responsibly) or may choose to register their own
 * manufacturer ID.
 */
#define RDM_UID_MANUFACTURER_ID (0x05e0)
#endif

#ifdef CONFIG_RDM_DEVICE_UID_DEV_ID
#define RDM_UID_DEVICE_ID (CONFIG_RDM_DEVICE_UID_DEV_ID)
#else
#define RDM_UID_DEVICE_UID (0xffffffff)
#endif

#ifdef CONFIG_RDM_DEBUG_DEVICE_DISCOVERY
#define RDM_DEBUG_DEVICE_DISCOVERY
#endif

#ifdef CONFIG_RDM_STACK_ALLOCATE_DISCOVERY
#define RDM_STACK_ALLOCATE_DISCOVERY
#endif

#ifdef CONFIG_RDM_RESPONDER_MAX_PARAMETERS
/** @brief The maximum number of parameters that the RDM responder can
 * support. This value is editable in the Kconfig.*/
#define RDM_RESPONDER_MAX_PIDS (8 + CONFIG_RDM_RESPONDER_MAX_PARAMETERS)
#else
/** @brief The maximum number of parameters that the RDM responder can
 * support.*/
#define RDM_RESPONDER_MAX_PIDS (8 + 16)
#endif

#ifdef __cplusplus
}
#endif
