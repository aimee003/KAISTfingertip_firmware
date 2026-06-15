#include <BNO080.h>
#include <fingertip.h>
#include "printing.h"
#include "math.h"
#include "ForceSensor.h"
#include "neural_nets.h"
#include "ms5849.h"
#include "ms5849_funcs.h"
#include "neural_nets.h"
#include "math_ops.h"
#include "sensor_interface.h"
#include "Quaternion.h"
#include <string.h>

#define SELECTED_FINGER 3

#define PR_1 			((SELECTED_FINGER - 1)*4 + 0)
#define PR_2 			((SELECTED_FINGER - 1)*4 + 1)
#define PR_3 			((SELECTED_FINGER - 1)*4 + 2)
#define PR_4 			((SELECTED_FINGER - 1)*4 + 3)
#define PR_TOF			((SELECTED_FINGER - 1)*4 + 4)  // ToF sensor message ID
#define PR_IMU			((SELECTED_FINGER - 1)*4 + 5)  // ToF sensor message ID

// Variables for force sensor data
int32_t pressure_raw[8];

ForceSensor fingertip;

//TOF
int range[9];

//IMU
float q[4];
float quatRadianAccuracy;

// Initialize CAN
FDCAN_RxHeaderTypeDef rxMsg;
FDCAN_TxHeaderTypeDef txMsg_t1, txMsg_i1, txMsg_p1, txMsg_p2, txMsg_p3, txMsg_p4 ; // ToF and force for each finger

//
FDCAN_FilterTypeDef can_filt;
uint8_t can_rx_buf[100];

uint8_t txMsg_t1_data[8];
uint8_t txMsg_i1_data[8];
uint8_t txMsg_p1_data[8];
uint8_t txMsg_p2_data[8];
uint8_t txMsg_p3_data[8];
uint8_t txMsg_p4_data[8];

static uint8_t tx_buf[1 + 48 + 1];

#define HEADER_Serial 0xAA

void pack_pressure_reply(uint8_t *msg1, uint8_t *msg2, uint8_t *msg3, uint8_t *msg4, ForceSensor * fs){
     msg1[3] = fs->raw_data[0]&0xFF;
     msg1[2] = (fs->raw_data[0]>>8)&0xFF;
     msg1[1] = (fs->raw_data[0]>>16)&0xFF;
     msg1[0] = (fs->raw_data[0]>>24)&0xFF;
     msg1[7] = fs->raw_data[1]&0xFF;
     msg1[6] = (fs->raw_data[1]>>8)&0xFF;
     msg1[5] = (fs->raw_data[1]>>16)&0xFF;
     msg1[4] = (fs->raw_data[1]>>24)&0xFF;

     msg2[3] = fs->raw_data[2]&0xFF;
     msg2[2] = (fs->raw_data[2]>>8)&0xFF;
     msg2[1] = (fs->raw_data[2]>>16)&0xFF;
     msg2[0] = (fs->raw_data[2]>>24)&0xFF;
     msg2[7] = fs->raw_data[3]&0xFF;
     msg2[6] = (fs->raw_data[3]>>8)&0xFF;
     msg2[5] = (fs->raw_data[3]>>16)&0xFF;
     msg2[4] = (fs->raw_data[3]>>24)&0xFF;

     msg3[3] = fs->raw_data[4]&0xFF;
     msg3[2] = (fs->raw_data[4]>>8)&0xFF;
     msg3[1] = (fs->raw_data[4]>>16)&0xFF;
     msg3[0] = (fs->raw_data[4]>>24)&0xFF;
     msg3[7] = fs->raw_data[5]&0xFF;
     msg3[6] = (fs->raw_data[5]>>8)&0xFF;
     msg3[5] = (fs->raw_data[5]>>16)&0xFF;
     msg3[4] = (fs->raw_data[5]>>24)&0xFF;

     msg4[3] = fs->raw_data[6]&0xFF;
     msg4[2] = (fs->raw_data[6]>>8)&0xFF;
     msg4[1] = (fs->raw_data[6]>>16)&0xFF;
     msg4[0] = (fs->raw_data[6]>>24)&0xFF;
     msg4[7] = fs->raw_data[7]&0xFF;
     msg4[6] = (fs->raw_data[7]>>8)&0xFF;
     msg4[5] = (fs->raw_data[7]>>16)&0xFF;
     msg4[4] = (fs->raw_data[7]>>24)&0xFF;
}

void pack_tof_reply(uint8_t * msg){
    /// pack ints into the can buffer ///
    msg[0] = range[0]; // top left
    msg[1] = range[1]; // front left
    msg[2] = range[2]; // top right
}

void pack_imu_reply(uint8_t * msg){
    /// pack ints into the can buffer ///
    msg[0] = BNO080_Roll; // top left
    msg[1] = BNO080_Pitch; // front left
    msg[2] = BNO080_Yaw; // top right
}

// main CPP loop
int fingertip_main(void){
//
//	for(volatile int i = 0; i < 100000; i++); // Manual burn loop

	HAL_Delay(3000);

	// initialize FDCAN filter (accept all messages)
	can_filt.IdType = FDCAN_STANDARD_ID;
	can_filt.FilterIndex = 0;
	can_filt.FilterType = FDCAN_FILTER_MASK;
	can_filt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filt.FilterID1 = 0x0000;  // Accept all IDs
	can_filt.FilterID2 = 0x0000;  // Mask (0 = don't care)

	// Configure FDCAN Tx headers
	txMsg_t1.Identifier = PR_TOF;
	txMsg_t1.IdType = FDCAN_STANDARD_ID;
	txMsg_t1.TxFrameType = FDCAN_DATA_FRAME;
	txMsg_t1.DataLength = FDCAN_DLC_BYTES_8;
	txMsg_t1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txMsg_t1.BitRateSwitch = FDCAN_BRS_OFF;
	txMsg_t1.FDFormat = FDCAN_CLASSIC_CAN;
	txMsg_t1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txMsg_t1.MessageMarker = 0;

	txMsg_i1.Identifier = PR_IMU;
	txMsg_i1.IdType = FDCAN_STANDARD_ID;
	txMsg_i1.TxFrameType = FDCAN_DATA_FRAME;
	txMsg_i1.DataLength = FDCAN_DLC_BYTES_8;
	txMsg_i1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txMsg_i1.BitRateSwitch = FDCAN_BRS_OFF;
	txMsg_i1.FDFormat = FDCAN_CLASSIC_CAN;
	txMsg_i1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txMsg_i1.MessageMarker = 0;

	txMsg_p1.Identifier = PR_1;
	txMsg_p1.IdType = FDCAN_STANDARD_ID;
	txMsg_p1.TxFrameType = FDCAN_DATA_FRAME;
	txMsg_p1.DataLength = FDCAN_DLC_BYTES_8;
	txMsg_p1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txMsg_p1.BitRateSwitch = FDCAN_BRS_OFF;
	txMsg_p1.FDFormat = FDCAN_CLASSIC_CAN;
	txMsg_p1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txMsg_p1.MessageMarker = 0;

	txMsg_p2.Identifier = PR_2;
	txMsg_p2.IdType = FDCAN_STANDARD_ID;
	txMsg_p2.TxFrameType = FDCAN_DATA_FRAME;
	txMsg_p2.DataLength = FDCAN_DLC_BYTES_8;
	txMsg_p2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txMsg_p2.BitRateSwitch = FDCAN_BRS_OFF;
	txMsg_p2.FDFormat = FDCAN_CLASSIC_CAN;
	txMsg_p2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txMsg_p2.MessageMarker = 0;

	txMsg_p3.Identifier = PR_3;
	txMsg_p3.IdType = FDCAN_STANDARD_ID;
	txMsg_p3.TxFrameType = FDCAN_DATA_FRAME;
	txMsg_p3.DataLength = FDCAN_DLC_BYTES_8;
	txMsg_p3.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txMsg_p3.BitRateSwitch = FDCAN_BRS_OFF;
	txMsg_p3.FDFormat = FDCAN_CLASSIC_CAN;
	txMsg_p3.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txMsg_p3.MessageMarker = 0;

	txMsg_p4.Identifier = PR_4;
	txMsg_p4.IdType = FDCAN_STANDARD_ID;
	txMsg_p4.TxFrameType = FDCAN_DATA_FRAME;
	txMsg_p4.DataLength = FDCAN_DLC_BYTES_8;
	txMsg_p4.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txMsg_p4.BitRateSwitch = FDCAN_BRS_OFF;
	txMsg_p4.FDFormat = FDCAN_CLASSIC_CAN;
	txMsg_p4.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txMsg_p4.MessageMarker = 0;

//	printf("Starting Fingertip Main Code.\n");
	// Configure FDCAN filter
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filt) != HAL_OK)
	{
//		printf("Failed to configure FDCAN filter.\n");
		while(1);
	}

	// Start FDCAN
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
//		printf("Failed to start FDCAN.\n");
		while(1);
	}

	// Activate FDCAN notification for Rx FIFO0
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

//	HAL_Delay(100);

//	HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);

	// initialize force sensor
	int8_t init_status_fs = fingertip.Initialize();
	if (init_status_fs != 0) {
//		printf("Force Sensor Init Failed! Status: %ld\n", init_status_fs);
		Error_Handler();
	} else {
//		printf("Force Sensor  Init Successful!\n");
	}
//	HAL_Delay(10);

	int32_t init_status_tof = TOF_Init();
	if (init_status_tof != 0) {
//		printf("TOF Init Failed! Status: %ld\n", init_status_tof);
		Error_Handler();
	} else {
//		printf("TOF Init Successful!\n");
	}
//	HAL_Delay(100);

//	BNO080_Hardware_Test();
	int32_t init_status_imu = BNO080_Initialization();
//	HAL_Delay(200);
	init_status_imu &= BNO080_enableRotationVector(2500);
	if (init_status_imu != 0) {
//		printf("IMU Init Failed! Status: %ld\n", init_status_imu);
		Error_Handler();
	} else {
//		printf("IMU Init Successful!\n");
	}
//	HAL_Delay(100);

	uint32_t last_hz_print = HAL_GetTick();
	uint32_t loop_counter = 0;
	uint32_t eval_time = 0;
//	uint32_t timer = 0;

	while (1) {
		/* Super loop */

		// Notes:
		// - Loop is set to run at 200Hz
		// - Sampling pressure sensors takes 1160us
		// - Sampling ToF sensors takes from 5*167=835us to 5*465=2325us, depending on how many have new results
		// - Packing and sending CAN messages takes ~610us (~120us each)
		// - At 200Hz loop timing, sampling typically takes ~2000-3100us
		// 1. Check for UART errors and clear them// RESET the UART peripheral state machines
//	    HAL_UART_DeInit(&huart1);
//	    HAL_UART_Init(&huart1);
//
//	    // Clear any potential error flags in the hardware registers
//	    __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
//
//	    HAL_Delay(100);
//		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
//			__HAL_UART_CLEAR_OREFLAG(&huart1);
//		}
//		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE) != RESET) {
//			__HAL_UART_CLEAR_FEFLAG(&huart1);
//		}
//		timer = HAL_GetTick();

		if (sample_flag == 1) {
			uint32_t start_time = __HAL_TIM_GET_COUNTER(&htim15);
			loop_counter++;
			if (HAL_GetTick() - last_hz_print >= 1000) {
//				printf("Loop Rate: %lu Hz, Eval Time: %lu us\n\r", loop_counter, eval_time);
				loop_counter = 0;
				last_hz_print = HAL_GetTick();
			}
			// reset interrupt flag
			sample_flag = 0;

			//read TOF
			VL53L4CD_Result_t Results;

			/* Poll for data using the global SensorObj */
			for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
				if (VL53L4CD_GetDistance(&SensorObjs[i], &Results) == 0) {
					range[i] = Results.ZoneResult[0].Distance[0];
//					printf("Sensor %d: %lu mm\n\r", i + 1, Results.ZoneResult[0].Distance[0]);
				}
			}
			if(BNO080_dataAvailable() == 1)
			  {
				  q[0] = BNO080_getQuatI();
				  q[1] = BNO080_getQuatJ();
				  q[2] = BNO080_getQuatK();
				  q[3] = BNO080_getQuatReal();
				  quatRadianAccuracy = BNO080_getQuatRadianAccuracy();
				  Quaternion_Update(&q[0]);
//				  printf("%.2d\t%.2d\t%.2d\n", BNO080_Roll, BNO080_Pitch, BNO080_Yaw); //print roll, pitch, yaw in degree
			  }


			// sample pressure sensors
			fingertip.Sample();


//	        // pack and send CAN messages
//	        pack_pressure_reply(txMsg_p1_data, txMsg_p2_data, txMsg_p3_data, txMsg_p4_data, &fingertip);
//	        pack_tof_reply(txMsg_t1_data);
//
//	    	// sending FDCAN messages
//	        // Helper lambda or function to send with wait
//	        auto send_can = [&](FDCAN_TxHeaderTypeDef* header, uint8_t* data) {
////	            while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0) {
////	                // Busy wait for free space
////	            }
////	            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, header, data);
//	        	uint32_t timeout = 100;
//				while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0 && timeout > 0) {
//					timeout--;
//				}
//				if (timeout > 0) {
//					HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, header, data);
//				}
//	        };
//
//	        send_can(&txMsg_t1, txMsg_t1_data);
//	        send_can(&txMsg_p1, txMsg_p1_data);
//	        send_can(&txMsg_p2, txMsg_p2_data);
//	        send_can(&txMsg_p3, txMsg_p3_data);
//	        send_can(&txMsg_p4, txMsg_p4_data);

	        eval_time = __HAL_TIM_GET_COUNTER(&htim15) - start_time;

////	         Serial version
//	        printf("%lu,0,0,0,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r",
//	        	HAL_GetTick(),
//	            fingertip.raw_data[0], fingertip.raw_data[1], fingertip.raw_data[2], fingertip.raw_data[3],
//	            fingertip.raw_data[4], fingertip.raw_data[5], fingertip.raw_data[6], fingertip.raw_data[7],
//	            range[0], range[1], range[2], range[3]);


//			uint8_t tx_buf[1 + 48 + 1]; // header + data + checksum

			// Fill header
			tx_buf[0] = HEADER_Serial;

			// Fill data (little endian from uint16_t)
			uint32_t packet[12];

			for (int i = 0; i < 8; ++i) {
				packet[i] = fingertip.raw_data[i];
			}
			for (int i = 0; i < 3; i++) {
				packet[i + 8] = range[i];
			}

			packet[11] = HAL_GetTick();

			// Copy data into tx buffer
			memcpy(&tx_buf[1], packet, sizeof(packet));

			// Compute checksum (simple sum)
			uint8_t checksum = 0;
			for (int i = 0; i < 48; i++) {
				checksum += tx_buf[1 + i];
			}

			// Store checksum
			tx_buf[49] = checksum;

			// Transmit
//			HAL_UART_Transmit(&huart1, tx_buf, sizeof(tx_buf),50);
//			HAL_UART_Transmit_IT(&huart1, tx_buf, sizeof(tx_buf));

//			printf("\n");
//			printf("\n");
//			printf("%lu\n", HAL_GetTick()-timer);
//			printf("\n");
//			printf("\n");

//	        char msg_buf[128];
//	        int msg_len = sprintf(msg_buf, "%lu,0,0,0,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
//	                              HAL_GetTick(),
//	                              fingertip.raw_data[0], fingertip.raw_data[1],
//	                              fingertip.raw_data[2], fingertip.raw_data[3],
//	                              fingertip.raw_data[4], fingertip.raw_data[5],
//	                              fingertip.raw_data[6], fingertip.raw_data[7],
//	                              range[0], range[1], range[2], range[3]);
//
//	        // Non-blocking — returns immediately, DMA handles the rest
	        if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
	        	HAL_UART_Transmit_DMA(&huart1, tx_buf, sizeof(tx_buf));
	        }

//	        char msg_buf[128]; // Create the buffer
//	        int msg_len;       // To store the actual length of the string
////
////	        // 1. Format the string into the buffer
//	        msg_len = sprintf(msg_buf, "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
//	                          HAL_GetTick(),
//	                          fingertip.raw_data[0], fingertip.raw_data[1], fingertip.raw_data[2], fingertip.raw_data[3],
//	                          fingertip.raw_data[4], fingertip.raw_data[5], fingertip.raw_data[6], fingertip.raw_data[7],
//	                          range[0], range[1], range[2]);
//
////	        // 2. Transmit the buffer via UART
////	        // We use msg_len to tell the hardware exactly how many bytes to send
////	        if (huart1.gState != HAL_UART_STATE_READY) {
////	            HAL_UART_Abort(&huart1);
////	            HAL_UART_DeInit(&huart1);
////	            HAL_UART_Init(&huart1);
////	        }
////	        HAL_StatusTypeDef status;
//	        HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, msg_len, 10);
//
//	        if (status != HAL_OK) {
//	            // The UART is stuck! Reset the hardware registers to unblock it.
//	            __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE);
//
//	            // Optional: Re-initialize if the error is terminal
//	            if(status == HAL_ERROR) {
//	                // Force-abort any stuck DMA/IT transfer
//	                HAL_UART_Abort(&huart1);
//
//	                // Clear all error flags
//	                __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE);
//
//	                // Always reinit — it's safe and fast
//	                HAL_UART_DeInit(&huart1);
//	                HAL_UART_Init(&huart1);
//	            }
//	        }


//			printf("Pressure: %03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d \n\r", fingertip.raw_data[0],fingertip.raw_data[1],fingertip.raw_data[2],
//						fingertip.raw_data[3],fingertip.raw_data[4],fingertip.raw_data[5],fingertip.raw_data[6],fingertip.raw_data[7]);
//			printf("TOF: %03d,%03d,%03d,%03d,%03d\n\r", range[0], range[1], range[2], range[3], range[4]);
//			printf("IMU: %03d,%03d,%03d\n\r", BNO080_Roll, BNO080_Pitch, BNO080_Yaw);
//			printf("\n\r\n\r");
//			printf("Force: %d,%d,%d\n\r", (int)(1000.0f*fingertip.output_data[0]), (int)(1000.0f*fingertip.output_data[1]), (int)(1000.0f*fingertip.output_data[2]));
//			printf("Angle: %d,%d\n\r\n\r", (int)(fingertip.output_data[3]), (int)(fingertip.output_data[4]));

		}

	}

}


