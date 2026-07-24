#include "sample.h"

#include "ForceSensor.h"
#include "BNO080.h"
#include "Quaternion.h"
#include "sensor_interface.h"
#include "kaist_net.h"
#include "printing.h"
#include "main.h"

#include <string.h>

/* The one ForceSensor instance. Kept file-local so nothing else can poke at
 * it; expose an accessor below if another module ever needs the raw taxels. */
static ForceSensor fingertip;

ForceSensor *fingertip_device(void) { return &fingertip; }

/* -------------------------------------------------------------------------
 * fingertip_init
 * ---------------------------------------------------------------------- */
int fingertip_init(void)
{
    /* --- pressure sensors + neural net --------------------------------- */
    if (fingertip.Initialize() != 0) {
        printf("Force Sensor Init Failed!\n\r");
        return FT_INIT_ERR_FS;
    }
    fingertip.Calibrate();      /* averages 10 rest samples into offsets[] */
    nn_reset();

    float worst;
    if (!nn_selftest(&worst)) {
//        printf("NN Self Test Failed! worst=%f\n\r", (double)worst);
        return FT_INIT_ERR_FS;
    }

    /* --- time of flight ------------------------------------------------ */
    if (TOF_Init() != 0) {
        printf("TOF Init Failed!\n\r");
        return FT_INIT_ERR_TOF;
    }

    /* --- IMU ------------------------------------------------------------ */
    int32_t imu_status = BNO080_Initialization();
    imu_status &= BNO080_enableRotationVector(2500);
    if (imu_status != 0) {
        printf("IMU Init Failed! Status: %ld\n\r", imu_status);
        return FT_INIT_ERR_IMU;
    }

    return FT_INIT_OK;
}

/* fingertip_sensors.cpp */
int fingertip_calibrate()
{
	fingertip.Calibrate();
    nn_reset();
    return FT_CAL_OK;
}

/* -------------------------------------------------------------------------
 * fingertip_sample
 * ---------------------------------------------------------------------- */
int fingertip_sample(fingertip_data_t *out)
{
    if (out == NULL) return FT_SAMPLE_OK;

    memset(out->range_valid, 0, sizeof(out->range_valid));
    out->imu_updated = 0;
    out->tick_ms     = HAL_GetTick();

    /* --- time of flight ------------------------------------------------ */
    VL53L4CD_Result_t results;
    out->range[0] = 0;
    for (int i = 1; i < TOF_SENSOR_COUNT; ++i) {
        if (VL53L4CD_GetDistance(&SensorObjs[i], &results) == 0) {
            out->range[i]       = (int16_t)results.ZoneResult[0].Distance[0];
            out->range_valid[i] = 1;
        }
        /* else: leave the previous value in place, flag stays 0 */
    }

    /* --- IMU ------------------------------------------------------------ */
    if (BNO080_dataAvailable() == 1) {
        out->q[0] = BNO080_getQuatI();
        out->q[1] = BNO080_getQuatJ();
        out->q[2] = BNO080_getQuatK();
        out->q[3] = BNO080_getQuatReal();
        out->quat_accuracy = BNO080_getQuatRadianAccuracy();
        Quaternion_Update(&out->q[0]);
        out->imu_updated = 1;
    }
    /* Quaternion_Update() writes the globals; copy them out either way so a
     * tick with no new IMU data still carries the last known attitude. */
    out->roll  = BNO080_Roll;
    out->pitch = BNO080_Pitch;
    out->yaw   = BNO080_Yaw;

    /* --- pressure + NN --------------------------------------------------- */
    fingertip.Sample();

    static float taxels[NN_N_TAXEL];
    for (int i = 0; i < NN_N_TAXEL; ++i) {
        out->raw[i] = (int32_t)fingertip.raw_data[i];
        taxels[i]   = (float)fingertip.offset_data[i];
    }
    nn_push(taxels);

    out->nn_valid = (uint8_t)(nn_infer(&out->nn) != 0);

    return out->nn_valid ? FT_SAMPLE_OK : FT_SAMPLE_WARMUP;
}
