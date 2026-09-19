#pragma once

// Beta band power from an EEG signal.
//
// Feed it filtered samples one at a time. Every BETA_FFT_SIZE samples it runs
// a radix-2 FFT, sums the power in each EEG band, and reports beta as a
// percentage of total power. That percentage is what a focus threshold is
// compared against, and it is self normalising, so electrode contact quality
// matters much less than it would for a raw amplitude.
//
// The FFT is written out here rather than pulled from esp-dsp, because the
// Arduino ESP32 core does not bundle that component. The C6 has no hardware
// floating point either, so one transform is a few milliseconds of work. It
// runs once per window, roughly once a second at 500 Hz with a 512 point size.

#include <math.h>

#ifndef BETA_FFT_SIZE
#define BETA_FFT_SIZE 512          // power of two, 512 matches the reference
#endif

#ifndef BETA_SMOOTHING
#define BETA_SMOOTHING 0.63f       // exponential smoothing on the band powers
#endif

// EEG band edges in Hz
#define EEG_DELTA_LOW   0.5f
#define EEG_DELTA_HIGH  4.0f
#define EEG_THETA_LOW   4.0f
#define EEG_THETA_HIGH  8.0f
#define EEG_ALPHA_LOW   8.0f
#define EEG_ALPHA_HIGH  13.0f
#define EEG_BETA_LOW    13.0f
#define EEG_BETA_HIGH   30.0f
#define EEG_GAMMA_LOW   30.0f
#define EEG_GAMMA_HIGH  45.0f

class BetaPower {
private:
    float re[BETA_FFT_SIZE];
    float im[BETA_FFT_SIZE];
    int   fill = 0;

    float sampleRate   = 500.0f;
    float smoothedBeta = 0.0f;
    float smoothedAll  = 0.0f;
    float percent      = 0.0f;

    // In place iterative Cooley-Tukey. Twiddles come from a recurrence
    // rather than a table, which costs a little accuracy and saves the RAM.
    void transform() {
        const int n = BETA_FFT_SIZE;

        for (int i = 1, j = 0; i < n; i++) {
            int bit = n >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) {
                float tr = re[i]; re[i] = re[j]; re[j] = tr;
                float ti = im[i]; im[i] = im[j]; im[j] = ti;
            }
        }

        for (int len = 2; len <= n; len <<= 1) {
            float ang = -2.0f * (float)M_PI / (float)len;
            float wr = cosf(ang), wi = sinf(ang);
            for (int i = 0; i < n; i += len) {
                float cr = 1.0f, ci = 0.0f;
                for (int k = 0; k < len / 2; k++) {
                    int a = i + k, b = i + k + len / 2;
                    float vr = re[b] * cr - im[b] * ci;
                    float vi = re[b] * ci + im[b] * cr;
                    re[b] = re[a] - vr;  im[b] = im[a] - vi;
                    re[a] = re[a] + vr;  im[a] = im[a] + vi;
                    float ncr = cr * wr - ci * wi;
                    ci = cr * wi + ci * wr;
                    cr = ncr;
                }
            }
        }
    }

public:
    void begin(float fs) {
        sampleRate = fs;
        reset();
    }

    // Returns true on the sample that completes a window and updates value().
    bool push(float sample) {
        re[fill] = sample;
        im[fill] = 0.0f;
        if (++fill < BETA_FFT_SIZE) return false;
        fill = 0;

        transform();

        const int   half   = BETA_FFT_SIZE / 2;
        const float binRes = sampleRate / (float)BETA_FFT_SIZE;
        float beta = 0.0f, total = 0.0f;

        for (int i = 1; i < half; i++) {
            float power = re[i] * re[i] + im[i] * im[i];
            float freq  = i * binRes;
            total += power;
            if (freq >= EEG_BETA_LOW && freq < EEG_BETA_HIGH) beta += power;
        }

        smoothedBeta = BETA_SMOOTHING * beta  + (1.0f - BETA_SMOOTHING) * smoothedBeta;
        smoothedAll  = BETA_SMOOTHING * total + (1.0f - BETA_SMOOTHING) * smoothedAll;
        percent      = (smoothedBeta / (smoothedAll + 1e-7f)) * 100.0f;
        return true;
    }

    // Beta power as a percentage of total power, 0 to 100.
    float value() const { return percent; }

    void reset() {
        resetWindow();
        smoothedBeta = smoothedAll = percent = 0.0f;
    }

    // Throws away the part-collected window but keeps the last value, for
    // when the filters restart mid-stream. Reporting zero there would make a
    // live bar drop to the floor and climb back for no real reason.
    void resetWindow() {
        fill = 0;
        for (int i = 0; i < BETA_FFT_SIZE; i++) { re[i] = 0.0f; im[i] = 0.0f; }
    }
};

// Example usage:
// BetaPower beta;
// beta.begin(500.0f);
// if (beta.push(eegSample)) {
//     if (beta.value() > FOCUS_THRESHOLD) { ... }
// }
