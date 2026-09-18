/*
 * notchfilter.c
 *
 *  Created on: Mar 30, 2026
 *      Author: User
 */
#include "main.h"
#include "notchfilter.h"
#include "stdint.h"
#include <math.h>



uint32_t start_notch, stop_notch;
float usnotch;



//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////PHILS_LAB_NOTCH_FILTER////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void NotchFilter_Init(NotchFilter *filt, float centerFreq_Hz, float notchWidth_Hz, float sampleFreq_Hz)
{
    /* Store sample frequency */
    filt->sampleFreq_Hz = sampleFreq_Hz;

    /* Compute filter coefficients */
    NotchFilter_SetCenterFreqHz(filt, centerFreq_Hz);
    NotchFilter_SetNotchWidthHz(filt, notchWidth_Hz);

    /* Clear input and output arrays */
    for (uint8_t n = 0; n < 3; n++)
    {
        filt->x[n] = 0.0f;
        filt->y[n] = 0.0f;
    }
}


void NotchFilter_SetCenterFreqHz(NotchFilter *filt, float centerFreq_Hz) {

    /* Convert filter frequency to angular frequency (rad/s) */
    float w0_rps = 2.0f * M_PI * centerFreq_Hz;

    /* Pre-warp center frequency */
    start_notch = DWT->CYCCNT;
    float w0_pw_rps = (2.0f * filt->sampleFreq_Hz) * tanf(0.5f * w0_rps / filt->sampleFreq_Hz);
    //float w0_pw_rps = (2.0f * filt->sampleFreq_Hz) * (arm_sin_f32(0.5f * w0_rps / filt->sampleFreq_Hz) / arm_cos_f32(0.5f * w0_rps / filt->sampleFreq_Hz));
    stop_notch = DWT->CYCCNT;
    usnotch = (stop_notch - start_notch) / (SystemCoreClock / 1000000.0f); // convert to microseconds

    /* Compute filter coefficient */
    filt->alpha = 4.0f + w0_pw_rps * w0_pw_rps / (filt->sampleFreq_Hz * filt->sampleFreq_Hz);

}

void NotchFilter_SetNotchWidthHz(NotchFilter *filt, float notchWidth_Hz) {

    /* Convert filter frequency to angular frequency (rad/s) */
    float ww_rps = 2.0f * M_PI * notchWidth_Hz;

    /* Compute filter coefficient */
    filt->beta = 2.0f * ww_rps / filt->sampleFreq_Hz;

}



float NotchFilter_Update(NotchFilter *filt, float in) {

    /* Shift samples */
    filt->x[2] = filt->x[1];
    filt->x[1] = filt->x[0];

    filt->y[2] = filt->y[1];
    filt->y[1] = filt->y[0];

    /* Store new input sample */
    filt->x[0] = in;

    /* Compute new output sample */
    filt->y[0] = (filt->alpha * filt->x[0] + 2.0f * (filt->alpha - 8.0f) * filt->x[1] + filt->alpha * filt->x[2]
                 - (2.0f * (filt->alpha - 8.0f) * filt->y[1] + (filt->alpha - filt->beta) * filt->y[2]))
                 / (filt->alpha + filt->beta);

    /* Return filtered output */
    return filt->y[0];

}















///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////ARDUCOPTER_FIRMWARE_NOTCH_FILTER//////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////







const static float NOTCH_MAX_SLEW       = 0.05f;
const static float NOTCH_MAX_SLEW_LOWER = 1.0f - NOTCH_MAX_SLEW;
const static float NOTCH_MAX_SLEW_UPPER = 1.0f / NOTCH_MAX_SLEW_LOWER;



/* ── Helper macros ── */

/* Safe floating-point equality (tolerance-based) */
#define FLOAT_EPSILON  1e-6f
#define is_equal(a, b) (fabsf((a) - (b)) < FLOAT_EPSILON)
#define is_zero(a)     (fabsf(a) < FLOAT_EPSILON)
#define is_positive(a) ((a) > FLOAT_EPSILON)
#define sq(x)          ((x) * (x))



/* Clamp x to [low, high] */
static inline float constrain_float(float x, float low, float high)
{
    if (x < low)  return low;
    if (x > high) return high;
    return x;
}




/**
 * Calculate the attenuation factor (A) and quality factor (Q)
 * for a notch filter.
 *
 * @param center_freq_hz  - Center frequency of the notch (Hz)
 * @param bandwidth_hz    - Bandwidth of the notch (Hz)
 * @param attenuation_dB  - Desired attenuation depth (dB, positive value)
 * @param A               - OUTPUT: linear attenuation factor
 * @param Q               - OUTPUT: quality factor
 */
void notch_calculate_A_and_Q(float center_freq_hz,
                              float bandwidth_hz,
                              float attenuation_dB,
                              float* A,
                              float* Q)
{
    /* Compute linear attenuation factor */
    *A = powf(10.0f, -attenuation_dB / 40.0f);

    /* Compute quality factor using octave-based bandwidth */
    if (center_freq_hz > 0.5f * bandwidth_hz) {
        float octaves = log2f(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0f)) * 2.0f;
        float pow2    = powf(2.0f, octaves);
        *Q = sqrtf(pow2) / (pow2 - 1.0f);
    } else {
        *Q = 0.0f;
    }
}





void notch_init_with_A_and_Q(NotchFilterFloat* f,
                              float sample_freq_hz,
                              float center_freq_hz,
                              float A,
                              float Q)
{
    /* Step 1 — Skip if nothing changed */
    if (f->initialised &&
        is_equal(center_freq_hz, f->_center_freq_hz) &&
        is_equal(sample_freq_hz, f->_sample_freq_hz) &&
        is_equal(A,              f->_A)) {
        return;
    }

    /* Step 2 — Apply slew rate limit to center frequency */
    float new_center_freq = center_freq_hz;

    if (f->initialised && !f->need_reset && !is_zero(f->_center_freq_hz)) {
        new_center_freq = constrain_float(new_center_freq,
                                          f->_center_freq_hz * NOTCH_MAX_SLEW_LOWER,
                                          f->_center_freq_hz * NOTCH_MAX_SLEW_UPPER);
    }

    /* Step 3 — Validate inputs before computing coefficients */
    if (is_positive(new_center_freq) &&
        (new_center_freq < 0.5f * sample_freq_hz) &&
        (Q > 0.0f))
    {
        /* Step 4 — Normalized angular frequency (radians per sample) */
        float omega = 2.0f * (float)M_PI * new_center_freq / sample_freq_hz;

        /* Step 5 — Bandwidth parameter */
        float alpha = sinf(omega) / (2.0f * Q);

        /* Step 6 — Raw biquad notch coefficients (from Audio EQ Cookbook) */
        float b0 =  1.0f + alpha * sq(A);
        float b1 = -2.0f * cosf(omega);
        float b2 =  1.0f - alpha * sq(A);
        float a1 =  b1;
        float a2 =  1.0f - alpha;
        /* a0 = 1.0 + alpha  (used only for normalization below) */

        /* Step 7 — Pre-divide everything by a0 to avoid runtime division */
        float a0_inv = 1.0f / (1.0f + alpha);

        f->b0 = b0 * a0_inv;
        f->b1 = b1 * a0_inv;
        f->b2 = b2 * a0_inv;
        f->a1 = a1 * a0_inv;
        f->a2 = a2 * a0_inv;

        /* Step 8 — Save state */
        f->_center_freq_hz = new_center_freq;
        f->_sample_freq_hz = sample_freq_hz;
        f->_A              = A;
        f->initialised     = true;
    }
    else {
        /* Invalid parameters — disable filter */
        f->initialised = false;
    }
}






/* ────────────────────────────────────────────────
   init
   ──────────────────────────────────────────────── */
void notch_init(NotchFilterFloat* f,
                float sample_freq_hz,
                float center_freq_hz,
                float bandwidth_hz,
                float attenuation_dB)
{
    /* Always reset first — recompute from scratch */
    f->initialised = false;

    /* Center freq must be above half the bandwidth (so lower edge > 0 Hz)
       and below Nyquist (so it is representable in discrete time)        */
    if ((center_freq_hz > 0.5f * bandwidth_hz) &&
        (center_freq_hz < 0.5f * sample_freq_hz))
    {
        float A, Q;
        notch_calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, &A, &Q);
        notch_init_with_A_and_Q(f, sample_freq_hz, center_freq_hz, A, Q);
    }
    /* If conditions fail, filter stays initialised=false (passthrough mode) */
}

/* ────────────────────────────────────────────────
   apply
   ──────────────────────────────────────────────── */
float notch_apply(NotchFilterFloat* f, float sample)
{
    /* If not ready, flush delay lines and pass sample through unchanged */
    if (!f->initialised || f->need_reset) {
        f->signal1   = sample;
        f->signal2   = sample;
        f->ntchsig1  = sample;
        f->ntchsig2  = sample;
        f->need_reset = false;
        return sample;
    }

    /* Biquad difference equation:
       y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
                      - a1*y[n-1] - a2*y[n-2]   */
    float output = f->b0 * sample
                 + f->b1 * f->ntchsig1
                 + f->b2 * f->ntchsig2
                 - f->a1 * f->signal1
                 - f->a2 * f->signal2;

    /* Shift delay lines forward by one sample */
    f->ntchsig2 = f->ntchsig1;   /* x[n-2] ← x[n-1] */
    f->ntchsig1 = sample;         /* x[n-1] ← x[n]   */
    f->signal2  = f->signal1;     /* y[n-2] ← y[n-1] */
    f->signal1  = output;         /* y[n-1] ← y[n]   */

    return output;
}

/* ────────────────────────────────────────────────
   reset
   ──────────────────────────────────────────────── */
void notch_reset(NotchFilterFloat* f)
{
    f->need_reset = true;
}





