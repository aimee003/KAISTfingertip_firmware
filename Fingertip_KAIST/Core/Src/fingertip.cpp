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
#include "sample.h"

#define SELECTED_FINGER 3

#define PR_1 			((SELECTED_FINGER - 1)*4 + 0)
#define PR_2 			((SELECTED_FINGER - 1)*4 + 1)
#define PR_3 			((SELECTED_FINGER - 1)*4 + 2)
#define PR_4 			((SELECTED_FINGER - 1)*4 + 3)
#define PR_TOF			((SELECTED_FINGER - 1)*4 + 4)  // ToF sensor message ID
#define PR_IMU			((SELECTED_FINGER - 1)*4 + 5)  // ToF sensor message ID

// sample output
fingertip_data_t sensors;

// Initialize CAN
#define FINGER_MSG_ID   (0x10 + (SELECTED_FINGER - 1))  // one ID per finger
#define FINGER_MSG_LEN  24                              // 22 used + 2 pad

FDCAN_RxHeaderTypeDef rxMsg;
FDCAN_TxHeaderTypeDef txMsg_all;
//
FDCAN_FilterTypeDef can_filt;
uint8_t can_rx_buf[100];
uint8_t txMsg_all_data[FINGER_MSG_LEN];


#define HEADER_Serial 0xAA


/* big-endian, saturating — matches the byte order of your existing packers */
static inline void put_i16(uint8_t *p, int32_t v){
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)( v       & 0xFF);
}
static inline int32_t q1000(float x){ return (int32_t)lroundf(1000.0f * x); }
static inline int32_t  q100(float x){ return (int32_t)lroundf( 100.0f * x); }

void pack_all_reply(uint8_t *msg, const fingertip_data_t *s){
    /*  0- 1  contact prob   x1000  (0..1000)
     *  2- 7  Fx,Fy,Fz       x100  [N*1e-2]
     *  8-13  ux,uy,uz       x100  [mm*1e-2]
     * 14-17  ToF range[1], range[2]  [mm]
     * 18-23  roll,pitch,yaw          [deg]                  */
    put_i16(&msg[0],  q1000(s->nn.contact_prob));
    put_i16(&msg[2],  q100(s->nn.F[0]));
    put_i16(&msg[4],  q100(s->nn.F[1]));
    put_i16(&msg[6],  q100(s->nn.F[2]));
    put_i16(&msg[8],  q100(s->nn.u[0]));
    put_i16(&msg[10], q100(s->nn.u[1]));
    put_i16(&msg[12], q100(s->nn.u[2]));
    put_i16(&msg[10], q100(s->range[1]));
    put_i16(&msg[12], q100(s->range[2]));
    put_i16(&msg[18], (int32_t)lroundf(s->roll));
    put_i16(&msg[20], (int32_t)lroundf(s->pitch));
    put_i16(&msg[22], (int32_t)lroundf(s->yaw));
}

#define FT_CMD_ID         0x3F3    /* 0x3F3, standard ID */
#define FT_CMD_CALIBRATE  0x0B    /* payload byte 0 */

static int poll_can_command(void)
{
    int want_cal = 0;
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0,
                                   &rxMsg, can_rx_buf) != HAL_OK) break;
        if (rxMsg.Identifier == FT_CMD_ID &&
            rxMsg.DataLength >= 1 &&
            can_rx_buf[0] == FT_CMD_CALIBRATE) {
            want_cal = 1;        /* drain the rest of the FIFO first */
        }
    }
    return want_cal;
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
	txMsg_all.Identifier          = FINGER_MSG_ID;
	txMsg_all.IdType              = FDCAN_STANDARD_ID;
	txMsg_all.TxFrameType         = FDCAN_DATA_FRAME;
	txMsg_all.DataLength          = FDCAN_DLC_BYTES_24;
	txMsg_all.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txMsg_all.BitRateSwitch       = FDCAN_BRS_ON;
	txMsg_all.FDFormat            = FDCAN_FD_CAN;
	txMsg_all.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	txMsg_all.MessageMarker       = 0;

	// init all sensors
    int st = fingertip_init();
    if (st != FT_INIT_OK) {
        printf("Sensor init failed, stage %d\n\r", st);
        Error_Handler();
    }

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

		if (sample_flag == 1) {
			uint32_t start_time = __HAL_TIM_GET_COUNTER(&htim15);
			loop_counter++;
			if (HAL_GetTick() - last_hz_print >= 1000) {
//				printf("Loop Rate: %lu Hz, Eval Time: %lu us, Eval Time nn: %lu us\n\r", loop_counter, eval_time, eval_time_nn);
				loop_counter = 0;
				last_hz_print = HAL_GetTick();
			}
			// reset interrupt flag
	        if (poll_can_command()) { fingertip_calibrate(); }
			sample_flag = 0;

			// sample fingertip, ToF, and IMU
			fingertip_sample(&sensors);

	        // pack and send CAN messages
		    pack_all_reply(txMsg_all_data, &sensors);

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

//	        send_can(&txMsg_all, txMsg_all_data);

	        eval_time = __HAL_TIM_GET_COUNTER(&htim15) - start_time;

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
//
//			int whole[7];
//			int frac[7];
//			const char * sgn[7];
//			whole[0] = (int) sensors.nn.contact_prob;
//			frac[0] = (int)((sensors.nn.contact_prob - whole[0]) * 100);
//			for (int i = 0; i < 3; ++i) {
//				sgn[i+1] = (sensors.nn.F[i] < 0.0f) ? "-" : "";
//				whole[i+1] = (int) sensors.nn.F[i];
//				frac[i+1] = (int)((sensors.nn.F[i] - whole[i+1]) * 100);
//				if (whole[i+1] < 0) whole[i+1] = -whole[i+1];
//				if (frac[i+1] < 0) frac[i+1] = -frac[i+1];
//			}
//			for (int i = 0; i < 3; ++i) {
//				sgn[i+4] = (sensors.nn.u[i] < 0.0f) ? "-" : "";
//				whole[i+4] = (int) sensors.nn.u[i];
//				frac[i+4] = (int)((sensors.nn.u[i] - whole[i+4]) * 100);
//				if (whole[i+4] < 0) whole[i+4] = -whole[i+4];
//				if (frac[i+4] < 0) frac[i+4] = -frac[i+4];
//			}
//
//	        printf("%lu,%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d\n\r",
//	        	HAL_GetTick(),
//				whole[0], frac[0],
//				sgn[1], whole[1], frac[1], sgn[2], whole[2], frac[2], sgn[3], whole[3], frac[3],
//				sgn[4], whole[4], frac[4], sgn[5], whole[5], frac[5], sgn[6], whole[6], frac[6]);
//
		}

	}

}


