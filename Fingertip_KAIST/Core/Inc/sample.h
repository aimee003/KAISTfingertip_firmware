#ifndef FINGERTIP_SENSORS_H_
#define FINGERTIP_SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "kaist_net.h"          /* nn_output_t, NN_N_TAXEL */
#include "sensor_interface.h"   /* TOF_SENSOR_COUNT        */
#include "user_config.h"        /* CAL_SAMPLES             */

/* Return codes for fingertip_init() */
#define FT_INIT_OK        0
#define FT_INIT_ERR_FS    1     /* pressure sensor / NN                */
#define FT_INIT_ERR_TOF   2     /* VL53L4CD ranging sensors            */
#define FT_INIT_ERR_IMU   3     /* BNO080                              */

/* Return codes for fingertip_sample() */
#define FT_SAMPLE_OK      0
#define FT_SAMPLE_WARMUP  1     /* valid frame, NN history not yet full */

typedef struct {
    /* --- neural net output ------------------------------------------- */
    nn_output_t nn;                     /* contact_prob, F[3] N, u[3] mm  */
    uint8_t     nn_valid;               /* 0 for the first NN_HISTORY ticks */

    /* --- time of flight ----------------------------------------------- */
    int16_t    range[TOF_SENSOR_COUNT];/* mm; index 0 unused             */
    uint8_t     range_valid[TOF_SENSOR_COUNT];

    /* --- IMU ----------------------------------------------------------- */
    float       q[4];                   /* i, j, k, real                   */
    float       quat_accuracy;          /* rad                             */
    float       roll, pitch, yaw;       /* deg                             */
    uint8_t     imu_updated;            /* 1 if new IMU data this tick     */

    /* --- raw taxels (debug / serial only, not in the CAN frame) -------- */
    int32_t     raw[NN_N_TAXEL];

    uint32_t    tick_ms;                /* HAL_GetTick() at sample time    */
} fingertip_data_t;

/* Initialize pressure sensors + NN, ToF, and IMU.
 * Returns FT_INIT_OK, or the FT_INIT_ERR_* code of the first stage to fail.
 * Does NOT touch FDCAN -- that stays in the caller.                       */
int fingertip_init(void);


/* Baseline calibration. fingertip_init() drives this blocking at boot; the
 * run loop drives it one sample per tick on a CAN command.
 * _reset() clears the accumulator and sets the target; _accumulate() takes one
 * fingertip_sample() result and returns 1 on the call that commits the new
 * offsets. */
void fingertip_calibrate_reset(int n_samples);
int  fingertip_calibrate_accumulate(const fingertip_data_t *s);

/* Take one full sample of all sensors and run NN inference.
 * `out` must be non-NULL. Returns FT_SAMPLE_OK, or FT_SAMPLE_WARMUP while
 * the NN input history is still filling (nn fields are zero until then).  */
int fingertip_sample(fingertip_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* FINGERTIP_SENSORS_H_ */
