/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "math.h"       /* lroundf */
#include <string.h>     /* memset */
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan2;

/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  /* Kernel clock = PCLK1 = 64 MHz.
   * Nominal (arbitration) 1 Mbit/s: 1 * (1 + 47 + 16) = 64 tq, 75% sample point.
   * Data phase        2 Mbit/s: 1 * (1 + 23 +  8) = 32 tq, 75% sample point. */
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 16;
  hfdcan2.Init.NominalTimeSeg1 = 47;
  hfdcan2.Init.NominalTimeSeg2 = 16;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 8;
  hfdcan2.Init.DataTimeSeg1 = 23;
  hfdcan2.Init.DataTimeSeg2 = 8;
  hfdcan2.Init.StdFiltersNbr = 1;   /* one slot for the FT_CMD_ID filter */
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN FDCAN2_MspInit 1 */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* TODO: bus is inert -- ConfigFilter/ConfigGlobalFilter/Start are never called,
 * so TX and RX both fail with HAL_FDCAN_ERROR_NOT_STARTED. */

static FDCAN_RxHeaderTypeDef rxMsg;
static FDCAN_TxHeaderTypeDef txMsg_all;
static FDCAN_FilterTypeDef   can_filt;
static uint8_t can_rx_buf[64];                  /* CAN FD max payload */
static uint8_t txMsg_all_data[FINGER_MSG_LEN];
static uint8_t ft_calibrating = 0;              /* drives FT_STATUS_CALIBRATING */
static uint8_t tx_seq = 0;                      /* rolling frame counter */
static volatile uint8_t cal_request = 0;        /* set by RX ISR */

/* big-endian, saturating */
static inline void put_i16(uint8_t *p, int32_t v){
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)( v       & 0xFF);
}
static inline int32_t q1000(float x){ return (int32_t)lroundf(1000.0f * x); }
static inline int32_t  q100(float x){ return (int32_t)lroundf( 100.0f * x); }

void pack_all_reply(uint8_t *msg, const fingertip_data_t *s){
    /* Fingertip reply: 32-byte CAN FD payload, standard ID 0x10 + (finger - 1).
     * All multi-byte fields are big-endian signed int16. Divide by the scale
     * to recover the physical value.
     *
     *  byte   field              type    scale  unit
     *  ----   -----              ----    -----  ----
     *  0      status             u8      -      see below
     *  1      seq                u8      -      rolling 0..255, +1 per frame;
     *                                           a gap means frames were dropped
     *  2- 3   contact prob       i16     1000   0..1 probability
     *  4- 9   Fx, Fy, Fz         i16     100    N
     * 10-15   ux, uy, uz         i16     100    mm
     * 16-19   ToF range 1, 2     i16     1      mm
     * 20-25   roll, pitch, yaw   i16     1      deg
     * 26-31   reserved                          always 0
     *
     * status:
     *  0  OK           every field valid
     *  1  WARMUP       NN history refilling (~160 ms after a calibration):
     *                  contact prob, F and u are 0; ToF and attitude valid
     *  2  CALIBRATING  collecting rest samples; all data fields are 0
     */
    memset(msg, 0, FINGER_MSG_LEN);
    msg[1] = tx_seq++;

    if (ft_calibrating) {
        msg[0] = FT_STATUS_CALIBRATING;
        return;
    }
    msg[0] = s->nn_valid ? FT_STATUS_OK : FT_STATUS_WARMUP;

    /* nn_infer() leaves s->nn stale while warming up -- send zeros instead. */
    if (s->nn_valid) {
        put_i16(&msg[2],  q1000(s->nn.contact_prob));
        put_i16(&msg[4],  q100(s->nn.F[0]));
        put_i16(&msg[6],  q100(s->nn.F[1]));
        put_i16(&msg[8],  q100(s->nn.F[2]));
        put_i16(&msg[10], q100(s->nn.u[0]));
        put_i16(&msg[12], q100(s->nn.u[1]));
        put_i16(&msg[14], q100(s->nn.u[2]));
    }
    /* range[] is already mm; scaling it would saturate int16 at 327 mm. */
    put_i16(&msg[16], s->range[1]);
    put_i16(&msg[18], s->range[2]);
    put_i16(&msg[20], (int32_t)lroundf(s->roll));
    put_i16(&msg[22], (int32_t)lroundf(s->pitch));
    put_i16(&msg[24], (int32_t)lroundf(s->yaw));
}

void can_set_calibrating(int on){
    ft_calibrating = on ? 1 : 0;
}

void can_init(void){
    /* Accept only FT_CMD_ID; mask 0x7FF is an exact match. */
    can_filt.IdType       = FDCAN_STANDARD_ID;
    can_filt.FilterIndex  = 0;
    can_filt.FilterType   = FDCAN_FILTER_MASK;
    can_filt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filt.FilterID1    = FT_CMD_ID;
    can_filt.FilterID2    = 0x7FF;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filt);

    /* Drop everything the filter didn't match, so motor traffic never
     * reaches the FIFO or wakes the ISR. Must run before Start(). */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    /* RX runs off the FIFO0 interrupt, independent of the 200 Hz loop. */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* TX header */
    txMsg_all.Identifier          = FINGER_MSG_ID;
    txMsg_all.IdType              = FDCAN_STANDARD_ID;
    txMsg_all.TxFrameType         = FDCAN_DATA_FRAME;
    txMsg_all.DataLength          = FINGER_MSG_DLC;
    txMsg_all.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txMsg_all.BitRateSwitch       = FDCAN_BRS_ON;
    txMsg_all.FDFormat            = FDCAN_FD_CAN;
    txMsg_all.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txMsg_all.MessageMarker       = 0;

    /* Leaves init mode; TX and RX both fail until this runs. */
    HAL_FDCAN_Start(&hfdcan2);
}

void can_pack_reply(const fingertip_data_t *s){
    pack_all_reply(txMsg_all_data, s);
}

void can_send_reply(void){
    /* Drops if the Tx FIFO is full; the host sees the gap in seq. */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txMsg_all, txMsg_all_data);
}

/* RX FIFO0 interrupt: drain and latch any calibrate request. */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan != &hfdcan2) return;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0,
                                   &rxMsg, can_rx_buf) != HAL_OK) break;
        if (rxMsg.Identifier == FT_CMD_ID &&
            rxMsg.DataLength >= 1 &&
            can_rx_buf[0] == FT_CMD_CALIBRATE) {
            cal_request = 1;
        }
    }
}

/* Read and clear the latched request. */
int can_cal_requested(void)
{
    __disable_irq();
    int req = cal_request;
    cal_request = 0;
    __enable_irq();
    return req;
}

/* USER CODE END 1 */
