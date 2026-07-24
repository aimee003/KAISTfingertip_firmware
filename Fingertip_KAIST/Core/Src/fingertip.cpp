#include <BNO080.h>
#include <fingertip.h>
#include "printing.h"
#include "math.h"
#include "ForceSensor.h"
#include "neural_nets.h"
#include "kaist_net.h"
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

nn_output_t nn;

//TOF
uint16_t range[9];

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

#define HEADER_Serial 0xAA

#define FINGER_MSG_ID   (0x10 + (SELECTED_FINGER - 1))  // one ID per finger
#define FINGER_MSG_LEN  24                              // 22 used + 2 pad

FDCAN_TxHeaderTypeDef txMsg_all;
uint8_t txMsg_all_data[FINGER_MSG_LEN];

/* big-endian, saturating — matches the byte order of your existing packers */
static inline void put_i16(uint8_t *p, int32_t v){
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)( v       & 0xFF);
}
static inline int32_t q1000(float x){ return (int32_t)lroundf(1000.0f * x); }

void pack_all_reply(uint8_t *msg, const nn_output_t *nn){
    /*  0- 1  contact prob   x1000  (0..1000)
     *  2- 7  Fx,Fy,Fz       x1000  [mN]
     *  8-13  ux,uy,uz       x1000  [um]
     * 14-15  ToF range[1], range[2]  [mm]
     * 16-21  roll,pitch,yaw          [deg]
     * 22-23  reserved                                        */
    put_i16(&msg[0],  q1000(nn->contact_prob));
    put_i16(&msg[2],  q1000(nn->F[0]));
    put_i16(&msg[4],  q1000(nn->F[1]));
    put_i16(&msg[6],  q1000(nn->F[2]));
    put_i16(&msg[8],  q1000(nn->u[0]));
    put_i16(&msg[10], q1000(nn->u[1]));
    put_i16(&msg[12], q1000(nn->u[2]));
//    msg[14] = range[1];
//    msg[15] = range[2];
    put_i16(&msg[14], range[1]);
    put_i16(&msg[16], range[2]);
    put_i16(&msg[18], (int32_t)lroundf(BNO080_Roll));
    put_i16(&msg[20], (int32_t)lroundf(BNO080_Pitch));
    put_i16(&msg[22], (int32_t)lroundf(BNO080_Yaw));
}


//		    printf("%d,%d,%d,%d,%d,%d,%d\n\r",
//		           (int)(nn.contact_prob>0.5),     // 0..1000
//		           (int)(1000.0f * nn.F[0]),             // mN
//		           (int)(1000.0f * nn.F[1]),
//		           (int)(1000.0f * nn.F[2]),
//		           (int)(1000.0f * nn.u[0]),             // micrometres
//		           (int)(1000.0f * nn.u[1]),
//		           (int)(1000.0f * nn.u[2]));
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
	txMsg_all.Identifier          = FINGER_MSG_ID;
	txMsg_all.IdType              = FDCAN_STANDARD_ID;
	txMsg_all.TxFrameType         = FDCAN_DATA_FRAME;
	txMsg_all.DataLength          = FDCAN_DLC_BYTES_24;
	txMsg_all.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txMsg_all.BitRateSwitch       = FDCAN_BRS_ON;
	txMsg_all.FDFormat            = FDCAN_FD_CAN;
	txMsg_all.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	txMsg_all.MessageMarker       = 0;

//	printf("Starting Fingertip Main Code.\n");
	// Configure FDCAN filter
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filt) != HAL_OK)
	{
		printf("Failed to configure FDCAN filter.\n");
		while(1);
	}

	// Start FDCAN
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		printf("Failed to start FDCAN.\n");
		while(1);
	}

	// Activate FDCAN notification for Rx FIFO0
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

//	HAL_Delay(100);

//	HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);

	// initialize force sensor
	int8_t init_status_fs = fingertip.Initialize();
//	printf("Calibrating...");
	fingertip.Calibrate();
//	printf("Done.");
//	nn_reset();
	if (init_status_fs != 0) {
		printf("Force Sensor Init Failed! Status: %ld\n", init_status_fs);
		Error_Handler();
	} else {
//		printf("Force Sensor  Init Successful!\n");
	}
//	HAL_Delay(10);

	float worst;
	int ok = nn_selftest(&worst);
	if (!ok) {
		printf("NN Self Test Failed!");
		Error_Handler();}

	int32_t init_status_tof = TOF_Init();
	if (init_status_tof != 0) {
		printf("TOF Init Failed! Status: %ld\n", init_status_tof);
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
		printf("IMU Init Failed! Status: %ld\n", init_status_imu);
		Error_Handler();
	} else {
//		printf("IMU Init Successful!\n");
	}
//	HAL_Delay(100);

	uint32_t last_hz_print = HAL_GetTick();
	uint32_t loop_counter = 0;
	uint32_t eval_time = 0;
	uint32_t eval_time_nn = 0;
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
//				printf("Loop Rate: %lu Hz, Eval Time: %lu us, Eval Time nn: %lu us\n\r", loop_counter, eval_time, eval_time_nn);
				loop_counter = 0;
				last_hz_print = HAL_GetTick();
			}
			// reset interrupt flag
			sample_flag = 0;

			//read TOF
			VL53L4CD_Result_t Results;

			/* Poll for data using the global SensorObj */
			for (int i = 1; i < TOF_SENSOR_COUNT; i++) {
				if (VL53L4CD_GetDistance(&SensorObjs[i], &Results) == 0) {
					range[i] = Results.ZoneResult[0].Distance[0];
//					printf("Sensor %d: %lu mm\n\r", i , Results.ZoneResult[0].Distance[0]);
//					printf("Sensor %d: %lu mm\n\r", i , range[i]);
				}
			}
			printf("Sensor 1: %lu mm, Sensor 2: %lu mm\n\r", range[1], range[2]);
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

//			printf("Sensor %d: %lu mm\n\r", 1 , range[1]);
			// sample pressure sensors
			fingertip.Sample();

		    /* --- tactile inference ------------------------------------------------
		     * The net was trained on ForceSensor::raw_data[] -- the SAME array that
		     * pack_pressure_reply() transmits. Not offset_data[]. See kaist_net.h.  */
		    static float taxels[NN_N_TAXEL];
		    for (int i = 0; i < NN_N_TAXEL; ++i)
		        taxels[i] = (float)fingertip.offset_data[i];
		    nn_push(taxels);

//		    static nn_output_t nn;          /* stale until the first successful infer */
		    uint32_t start_time_nn = __HAL_TIM_GET_COUNTER(&htim15);
		    nn_infer(&nn);                  /* returns 0 for the first NN_HISTORY ticks */
		    eval_time_nn = __HAL_TIM_GET_COUNTER(&htim15) - start_time_nn;
		    /* nn.contact_prob  in [0,1]
		     * nn.F[3]          Fx, Fy, Fz  [N]
		     * nn.u[3]          contact point on the superellipsoid  [mm]            */
//		    printf("%d,%d,%d,%d,%d,%d,%d\n\r",
//		           (int)(nn.contact_prob>0.5),     // 0..1000
//		           (int)(1000.0f * nn.F[0]),             // mN
//		           (int)(1000.0f * nn.F[1]),
//		           (int)(1000.0f * nn.F[2]),
//		           (int)(1000.0f * nn.u[0]),             // micrometres
//		           (int)(1000.0f * nn.u[1]),
//		           (int)(1000.0f * nn.u[2]));

	        // pack and send CAN messages
		    pack_all_reply(txMsg_all_data, &nn);

	    	// sending FDCAN messages
	        // Helper lambda or function to send with wait
	        auto send_can = [&](FDCAN_TxHeaderTypeDef* header, uint8_t* data) {
	        	// G4 FDCAN Tx FIFO is only 3 deep; wait (time-based) for a free slot
	        	// so frames beyond the first 3 aren't silently dropped.
	        	uint32_t start = HAL_GetTick();
				while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0) {
					if (HAL_GetTick() - start > 3) {  // ~3ms cap; each frame drains in ~0.22ms
						return;
					}
				}
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, header, data);
	        };

	        send_can(&txMsg_all, txMsg_all_data);

	        eval_time = __HAL_TIM_GET_COUNTER(&htim15) - start_time;

//	         Serial version
//	        printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r",
//	        	HAL_GetTick(),
//	            fingertip.raw_data[0], fingertip.raw_data[1], fingertip.raw_data[2], fingertip.raw_data[3],
//	            fingertip.raw_data[4], fingertip.raw_data[5], fingertip.raw_data[6], fingertip.raw_data[7],
//	            range[1], range[2]);


//			uint8_t tx_buf[1 + 48 + 1]; // header + data + checksum
//
//			// Fill header
//			tx_buf[0] = HEADER_Serial;
//
//			// Fill data (little endian from uint16_t)
//			uint32_t packet[12];
//
//			for (int i = 0; i < 8; ++i) {
//				packet[i] = fingertip.raw_data[i];
//			}
//			for (int i = 0; i < 4; i++) {
//				packet[i + 8] = range[i];
//			}
//
//			// Copy data into tx buffer
//			memcpy(&tx_buf[1], packet, sizeof(packet));
//
//			// Compute checksum (simple sum)
//			uint8_t checksum = 0;
//			for (int i = 0; i < 48; i++) {
//				checksum += tx_buf[1 + i];
//			}
//
//			// Store checksum
//			tx_buf[49] = checksum;

			// Transmit
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
//	        if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
//	            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg_buf, msg_len);
//	        }

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
//			printf("Probability: %d\n\r", (int)(nn.contact_prob>0.5));
//	        float F_mag = sqrt(pow(nn.F[0],2)+pow(nn.F[1],2)+pow(nn.F[2],2));
//			int whole = (int)F_mag;
//			int frac = (int)((F_mag - whole) * 100);  // 2 decimal places
//			if (frac < 0) frac = -frac;  // handle negative numbers
//			printf("Force: %d.%02d\n\r", whole, frac);
//			printf("Position: %d,%d,%d\n\r\n\r", (int)(nn.u[0]), (int)(nn.u[1]), (int)(nn.u[2]));
//			printf(%d, %d.%02d, )

//			int whole[7];
//			int frac[7];
//			const char * sgn[7];
//			whole[0] = (int) nn.contact_prob;
//			frac[0] = (int)((nn.contact_prob - whole[0]) * 100);
//			for (int i = 0; i < 3; ++i) {
//				sgn[i+1] = (nn.F[i] < 0.0f) ? "-" : "";
//				whole[i+1] = (int) nn.F[i];
//				frac[i+1] = (int)((nn.F[i] - whole[i+1]) * 100);
//				if (whole[i+1] < 0) whole[i+1] = -whole[i+1];
//				if (frac[i+1] < 0) frac[i+1] = -frac[i+1];
//			}
//			for (int i = 0; i < 3; ++i) {
//				sgn[i+4] = (nn.u[i] < 0.0f) ? "-" : "";
//				whole[i+4] = (int) nn.u[i];
//				frac[i+4] = (int)((nn.u[i] - whole[i+4]) * 100);
//				if (whole[i+4] < 0) whole[i+4] = -whole[i+4];
//				if (frac[i+4] < 0) frac[i+4] = -frac[i+4];
//			}
//
//	        printf("%lu,%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d\n\r",
//	        	HAL_GetTick(),
//				whole[0], frac[0],
//				sgn[1], whole[1], frac[1], sgn[2], whole[2], frac[2], sgn[3], whole[3], frac[3],
//				sgn[4], whole[4], frac[4], sgn[5], whole[5], frac[5], sgn[6], whole[6], frac[6]);

		}

	}

}


