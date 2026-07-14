/* AUTO-GENERATED. Call nn_selftest(&w) at boot before trusting the net.
 *
 * Replays golden vectors through the emitted C and compares against PyTorch.
 * Verifies: weights, layer order, activations, normalize_u.
 * Does NOT verify the ring-buffer flatten order -- it calls nn_forward_raw()
 * directly, bypassing nn_push(). To check that, capture one real NN_HISTORY-deep
 * window, run it through the Python model, and compare against nn_infer(). */
#include "kaist_net.h"
#include "kaist_net_golden.h"
#include <math.h>

int nn_selftest(float *worst_out)
{
    nn_output_t o;
    float worst = 0.0f;
    for (int k = 0; k < NN_GOLDEN_N; ++k) {
        nn_forward_raw(&nn_golden_in[k * NN_IN_CHAN], &o);
        const float e[7] = {
            fabsf(o.contact_prob - nn_golden_prob[k]),
            fabsf(o.F[0] - nn_golden_F[k * 3 + 0]),
            fabsf(o.F[1] - nn_golden_F[k * 3 + 1]),
            fabsf(o.F[2] - nn_golden_F[k * 3 + 2]),
            fabsf(o.u[0] - nn_golden_u[k * 3 + 0]),
            fabsf(o.u[1] - nn_golden_u[k * 3 + 1]),
            fabsf(o.u[2] - nn_golden_u[k * 3 + 2]),
        };
        for (int i = 0; i < 7; ++i) if (e[i] > worst) worst = e[i];
    }
    if (worst_out) *worst_out = worst;
    return worst < 1e-3f;   /* fp32 accumulation-order slack; u is in mm */
}
