/*
 * notchfilter.h
 *
 *  Created on: Mar 30, 2026
 *      Author: User
 */

#ifndef INC_NOTCHFILTER_H_
#define INC_NOTCHFILTER_H_

#include "math.h"
#include <stdbool.h>

typedef struct {

    /* Sampling frequency (Hz) */
    float sampleFreq_Hz;

    /* Filter 'coefficients' */
    float alpha;
    float beta;

    /* Input array */
    float x[3];

    /* Output array */
    float y[3];

} NotchFilter;


void NotchFilter_Init(NotchFilter *filt, float centerFreq_Hz, float notchWidth_Hz, float sampleFreq_Hz);

void NotchFilter_SetCenterFreqHz(NotchFilter *filt, float centerFreq_Hz);

void NotchFilter_SetNotchWidthHz(NotchFilter *filt, float notchWidth_Hz);

float NotchFilter_Update(NotchFilter *filt, float in);




















/* Biquad notch filter state */
typedef struct {
    /* Coefficients (pre-divided by a0) */
    float b0, b1, b2;
    float a1, a2;

    /* Delay lines — input history */
    float ntchsig1;   /* x[n-1] */
    float ntchsig2;   /* x[n-2] */

    /* Delay lines — output history */
    float signal1;    /* y[n-1] */
    float signal2;    /* y[n-2] */

    /* Saved parameters for change detection and slew limiting */
    float _center_freq_hz;
    float _sample_freq_hz;
    float _A;

    /* State flags */
    bool  initialised;
    bool  need_reset;
} NotchFilterFloat;

/* ── Function declarations ── */

void notch_calculate_A_and_Q(float  center_freq_hz,
                              float  bandwidth_hz,
                              float  attenuation_dB,
                              float* A,
                              float* Q);

void notch_init_with_A_and_Q(NotchFilterFloat* f,
                              float sample_freq_hz,
                              float center_freq_hz,
                              float A,
                              float Q);




void  notch_init               (NotchFilterFloat* f,
                                float sample_freq_hz,
                                float center_freq_hz,
                                float bandwidth_hz,
                                float attenuation_dB);

float notch_apply              (NotchFilterFloat* f,
                                float sample);

void  notch_reset              (NotchFilterFloat* f);













#endif /* INC_NOTCHFILTER_H_ */
