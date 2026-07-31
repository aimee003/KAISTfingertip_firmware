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

// sample output
fingertip_data_t sensors;

#define HEADER_Serial 0xAA

// main CPP loop
int fingertip_main(void){
//
//	for(volatile int i = 0; i < 100000; i++); // Manual burn loop

	for (int i = 0; i < 5; i++) {
		HAL_GPIO_WritePin(STATUS_LED, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(STATUS_LED, GPIO_PIN_RESET);
		HAL_Delay(100);
	}

	HAL_Delay(3000);

	// initialize FDCAN filter and Tx header
	can_init();

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
	int cal_mode = 0;
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
			sample_flag = 0;

			// sample fingertip, ToF, and IMU
			fingertip_sample(&sensors);

			// read once per tick, so requests during calibration are discarded
			int cal_req = can_cal_requested();

			if (cal_mode) {
				int cal_done = fingertip_calibrate_accumulate(&sensors);
				if (cal_done) {
					cal_mode = 0;
					can_set_calibrating(0);
				}
			} else if (cal_req) {
				cal_mode = 1;
				can_set_calibrating(1);
				fingertip_calibrate_reset(cal_req);   // count came with the command
			} else {
				// solid = running normally; calibration blinks it
				HAL_GPIO_WritePin(STATUS_LED, GPIO_PIN_SET);
			}

	        // pack and send CAN message
		    can_pack_reply(&sensors);
		    can_send_reply();

	        eval_time = __HAL_TIM_GET_COUNTER(&htim15) - start_time;

	        // raw taxels + ToF printing for debugging
	        // printf("%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d\n\r",
	        // 	HAL_GetTick(),
			// 	sensors.raw[0], sensors.raw[1], sensors.raw[2], sensors.raw[3],
			// 	sensors.raw[4], sensors.raw[5], sensors.raw[6], sensors.raw[7],
			// 	sensors.range[1], sensors.range[2]);

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

		}

	}

}


