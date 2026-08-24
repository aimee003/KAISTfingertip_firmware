/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "user_config.h"   /* FINGERTIP_SENSOR_TX_ID / _RX_ID */
#include "sample.h"        /* fingertip_data_t */
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan2;

/* USER CODE BEGIN Private defines */

/* Bus addresses live in user_config.h; the protocol itself is defined here. */

/* Command frame: host -> sensor, standard id FINGERTIP_SENSOR_RX_ID.
 *
 *   byte   value
 *   0      opcode, FT_CMD_CALIBRATE
 *   1-2    optional big-endian uint16 sample count.
 *          Omit (1-byte frame) or send 0 -> CAL_SAMPLES.
 *          Clamped to CAL_SAMPLES_MAX.
 *
 * Recalibrate with the default count:      0B
 * Recalibrate averaging 250 samples:       0B 00 FA
 * Recalibrate averaging 2000 samples:      0B 07 D0
 *
 * python-can, RX id 42 and 500 samples:
 *   bus.send(can.Message(arbitration_id=42, data=bytes([0x0B, 0x01, 0xF4]),
 *                        is_extended_id=False, is_fd=True, bitrate_switch=True))
 *
 * The finger must be untouched and still for the whole calibration -- any
 * contact gets averaged into the baseline and subtracted from then on.
 * Progress is reported as FT_STATUS_CALIBRATING in byte 0 of the reply, then
 * ~32 ticks of FT_STATUS_WARMUP while the NN history refills.
 */
#define FT_CMD_CALIBRATE  0x0B          /* command payload byte 0 */

/* 26 bytes used; 32 is the next legal FD size. Keep LEN and DLC in sync. */
#define FINGER_MSG_LEN    32
#define FINGER_MSG_DLC    FDCAN_DLC_BYTES_32

/* Reply byte 0 */
#define FT_STATUS_OK           0   /* all fields current */
#define FT_STATUS_WARMUP       1   /* NN refilling; prob/F/u zero, ToF+IMU valid */
#define FT_STATUS_CALIBRATING  2   /* all data fields zero */

/* USER CODE END Private defines */

void MX_FDCAN2_Init(void);

/* USER CODE BEGIN Prototypes */

void can_init(void);
void pack_all_reply(uint8_t *msg, const fingertip_data_t *s);
void can_pack_reply(const fingertip_data_t *s);
void can_send_reply(void);
void can_set_calibrating(int on);
int  can_cal_requested(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

