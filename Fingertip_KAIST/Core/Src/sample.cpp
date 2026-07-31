#include "sample.h"
#include "fdcan.h"      /* can_set_calibrating / can_pack_reply / can_send_reply */

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
    /* Same accumulator the run-time path uses, at the same 5 ms cadence, so
     * the baseline is measured under the conditions inference will see.
     * Blocking here is fine -- nothing else is running yet. ToF and IMU are
     * not up, so raw[] is filled directly instead of via fingertip_sample(). */
    can_set_calibrating(1);     /* so the host sees CALIBRATING, not silence */
    fingertip_calibrate_reset(CAL_SAMPLES);
    fingertip_data_t tmp = {};
    for (int n = 0; ; ++n) {
        fingertip.Sample();
        for (int i = 0; i < NN_N_TAXEL; ++i) tmp.raw[i] = (int32_t)fingertip.raw_data[i];
        if (fingertip_calibrate_accumulate(&tmp)) break;   /* also does nn_reset() */
        if ((n % CAL_TX_EVERY_SAMPLES) == 0) {             /* heartbeat; loop is blocked */
            can_pack_reply(&tmp);
            can_send_reply();
        }
        HAL_Delay(4);           /* + ~1.2 ms Sample() -> ~5 ms per sample */
    }
    can_set_calibrating(0);

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

/* -------------------------------------------------------------------------
 * Stepped calibration: one rest sample per call. Call it at the normal
 * sampling rate so the baseline is measured under the same conditions the
 * neural net sees at inference time.
 * ---------------------------------------------------------------------- */
static float cal_acc[8];
static int   cal_count;
static int   cal_target;

void fingertip_calibrate_reset(int n_samples)
{
    memset(cal_acc, 0, sizeof cal_acc);
    cal_count  = 0;
    cal_target = (n_samples > 0) ? n_samples : 1;
    HAL_GPIO_WritePin(STATUS_LED, GPIO_PIN_RESET);
}

/* Returns 1 on the call that commits the new offsets. */
int fingertip_calibrate_accumulate(const fingertip_data_t *s)
{
    for (int i = 0; i < 8; ++i)
        cal_acc[i] += ((float)s->raw[i]) / ((float)cal_target);

    if (++cal_count < cal_target) {
        if ((cal_count % CAL_BLINK_SAMPLES) == 0)
            HAL_GPIO_TogglePin(STATUS_LED);
        return 0;
    }

    for (int i = 0; i < 8; ++i) fingertip.offsets[i] = (int)cal_acc[i];
    nn_reset();
    HAL_GPIO_WritePin(STATUS_LED, GPIO_PIN_SET);   /* solid = running */
    return 1;
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
