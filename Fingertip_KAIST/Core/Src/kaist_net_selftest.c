/* AUTO-GENERATED. Call nn_selftest(&w) at boot before trusting the net.
 *
 * Replays golden vectors through the emitted C and compares against the fp32
 * PyTorch model. Verifies weights, layer order, activations, normalize_u and --
 * for int16 builds -- the quantization scales and packed byte order.
 *
 * It does NOT verify the ring-buffer flatten order: it calls nn_forward_raw()
 * directly, bypassing nn_push(). To check that, capture one real NN_HISTORY-deep
 * window of raw_data, run it through the Python model, and compare nn_infer().
 *
 * *worst_out receives the largest RELATIVE tolerance usage: 0.5 means "used half
 * the allowed error", 1.0 means "at the limit". Returns 1 on pass. */
#include "kaist_net.h"
#include "kaist_net_golden.h"
#include <math.h>

int nn_selftest(float *worst_out)
{
    nn_output_t o;
    float worst = 0.0f, r;
    for (int k = 0; k < NN_GOLDEN_N; ++k) {
        nn_forward_raw(&nn_golden_in[k * NN_IN_CHAN], &o);
        r = fabsf(o.contact_prob - nn_golden_prob[k]) / NN_TOL_PROB;
        if (r > worst) worst = r;
        for (int i = 0; i < 3; ++i) {
            r = fabsf(o.F[i] - nn_golden_F[k * 3 + i]) / NN_TOL_F;
            if (r > worst) worst = r;
            r = fabsf(o.u[i] - nn_golden_u[k * 3 + i]) / NN_TOL_U;
            if (r > worst) worst = r;
        }
    }
    if (worst_out) *worst_out = worst;
    return worst <= 1.0f;
}
