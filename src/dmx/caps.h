#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** @brief DMX port max. Used for error checking.*/
#define DMX_NUM_MAX SOC_UART_NUM

#ifdef CONFIG_DMX_MAX_PERSONALITIES
/** @brief The maximum number of personalities that this device supports. This
 * value may be adjusted in the Kconfig.*/
#define DMX_PERSONALITIES_MAX (CONFIG_DMX_MAX_PERSONALITIES)
#else
/** @brief The maximum number of personalities that this device supports.*/
#define DMX_PERSONALITIES_MAX (16)
#endif

#ifdef CONFIG_RDM_DEVICE_UID_MAN_ID
#define RDM_UID_MANUFACTURER_ID (CONFIG_RDM_DEVICE_UID_MAN_ID)
#else
/**
 * @brief This is the RDM Manufacturer ID that was registered with ESTA for use
 * with this software. Any device that uses this ID is associated with this
 * library. Users of this library are welcome to use this manufacturer ID (as
 * long as it is used responsibly) or may choose to register their own
 * manufacturer ID.*/
#define RDM_UID_MANUFACTURER_ID (0x05e0)
#endif

#ifdef CONFIG_RDM_DEVICE_UID_DEV_ID
#define RDM_UID_DEVICE_ID (CONFIG_RDM_DEVICE_UID_DEV_ID)
#else
#define RDM_UID_DEVICE_ID (0xffffffff)
#endif

#ifdef CONFIG_RDM_RESPONDER_MAX_PARAMETERS
/** @brief The maximum number of parameters that the RDM responder can
 * support. This value is editable in the Kconfig.*/
#define RDM_RESPONDER_PIDS_MAX (8 + CONFIG_RDM_RESPONDER_MAX_PARAMETERS)
#else
/** @brief The maximum number of parameters that the RDM responder can
 * support.*/
#define RDM_RESPONDER_PIDS_MAX (8 + 16)
#endif

#ifdef __cplusplus
}
#endif
