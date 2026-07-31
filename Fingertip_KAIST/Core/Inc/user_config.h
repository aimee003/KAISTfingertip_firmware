/// Per-board settings. Edit these before flashing each fingertip. ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#include "main.h"       /* LED_1_GPIO_Port / LED_1_Pin, from CubeMX */

/* CAN bus addresses ------------------------------------------------------ */
#define FINGERTIP_SENSOR_TX_ID  24      /* sensor -> host, data frames (0x17) */
#define FINGERTIP_SENSOR_RX_ID  42      /* host -> sensor, commands    (0x20) */

/* Calibration ------------------------------------------------------------ */
#define CAL_SAMPLES         1000        /* rest samples averaged, 1 per tick -> ~5 s */
#define CAL_BLINK_SAMPLES   100         /* LED toggles every N samples -> ~1 Hz */
#define CAL_TX_EVERY_SAMPLES 20         /* boot calibration: status frame every N (~100 ms) */
#define CAL_SAMPLES_MAX     5000        /* clamp for a count sent over CAN (~25 s) */

/* Board pins ------------------------------------------------------------- */
#define STATUS_LED      LED_1_GPIO_Port, LED_1_Pin

#endif
