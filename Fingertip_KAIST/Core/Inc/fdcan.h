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
#include "sample.h"     /* fingertip_data_t */
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan2;

/* USER CODE BEGIN Private defines */

#define SELECTED_FINGER   3      /* per-board; only thing that differs */

#define FINGER_MSG_ID     (0x10 + (SELECTED_FINGER - 1))   /* sensor -> host */
#define FT_CMD_ID         0x3F3                            /* host -> sensors */
#define FT_CMD_CALIBRATE  0x0B                             /* cmd payload byte 0 */

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

