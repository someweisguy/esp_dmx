#pragma once

#define DMX_NUM_0                   0             // DMX port 0.
#define DMX_NUM_1                   1             // DMX port 1.
#if SOC_UART_NUM > 2
#define DMX_NUM_2                   2             // DMX port 2.
#endif
#define DMX_NUM_MAX                 SOC_UART_NUM  // DMX port max.

/* DMX shared parameters */
#define DMX_MIN_BAUDRATE            245000        // DMX minimum baudrate.
#define DMX_TYP_BAUDRATE            250000        // DMX typical baudrate.
#define DMX_MAX_BAUDRATE            255000        // DMX maximum baudrate.
#define DMX_MAX_PACKET_SIZE         513           // DMX maximum packet size.

/* DMX client/receive timing parameters */
#define DMX_RX_MIN_SPACE_FOR_BRK_US 88            // DMX minimum receivable break length in microseconds.
#define DMX_RX_MIN_MRK_AFTER_BRK_US 8             // DMX minimum receivable mark after break length in microseconds.
#define DMX_RX_MAX_MRK_AFTER_BRK_US 999999        // DMX maximum receivable mark after break length in microseconds.
#define DMX_RX_MIN_BRK_TO_BRK_US    1196          // DMX minimum receivable break-to-break length in microseconds.
#define DMX_RX_MAX_BRK_TO_BRK_US    1250000       // DMX maximum receivable break-to-break length in microseconds.
#define DMX_RX_PACKET_TOUT_MS       1250          // DMX client packet timeout in milliseconds.
#define DMX_RX_PACKET_TOUT_TICK     ((TickType_t)DMX_RX_PACKET_TOUT_MS / portTICK_PERIOD_MS) // DMX client packet timeout in FreeRTOS ticks.

/* DMX host/transmit timing parameters */
#define DMX_TX_MIN_SPACE_FOR_BRK_US 92            // DMX minimum transmittable break length in microseconds.
#define DMX_TX_MIN_MRK_AFTER_BRK_US 12            // DMX minimum transmittable mark after break length in microseconds.
#define DMX_TX_MAX_MRK_AFTER_BRK_US 999999        // DMX maximum transmittable mark after break length in microseconds.
#define DMX_TX_MIN_BRK_TO_BRK_US    1204          // DMX minimum transmittable break-to-break length in microseconds.
#define DMX_TX_MAX_BRK_TO_BRK_US    1000000       // DMX maximum transmittable break-to-break length in microseconds.
#define DMX_TX_PACKET_TOUT_MS       1000          // DMX host packet timeout in milliseconds.
#define DMX_TX_PACKET_TOUT_TICK     ((TickType_t)DMX_TX_PACKET_TOUT_MS / portTICK_PERIOD_MS) // DMX host packet timeout in FreeRTOS ticks.
