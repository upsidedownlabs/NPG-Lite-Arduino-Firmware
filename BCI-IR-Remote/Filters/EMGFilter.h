#pragma once

// High-Pass Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// Reference: https://github.com/upsidedownlabs/BioAmp-Filter-Designer

class EMGFilter {
private:
    struct BiquadState { float z1 = 0, z2 = 0; };
    BiquadState state0;

public:
    float process(float input) {
        float output = input;

        // Biquad section 0
        float x0 = output - (-0.82523238f * state0.z1) - (0.29463653f * state0.z2);
        output = 0.52996723f * x0 + -1.05993445f * state0.z1 + 0.52996723f * state0.z2;
        state0.z2 = state0.z1;
        state0.z1 = x0;

        return output;
    }

    void reset() {
        state0.z1 = state0.z2 = 0;
    }
};

// Example usage:
// Single channel:
// EMGFilter filter;
// filter.reset();
// float filtered_output = filter.process(sample);
// 
// Multi-channel (3 channels):
// EMGFilter filters[3];  // One filter per channel
// float filtered_1 = filters[0].process(raw1);
// float filtered_2 = filters[1].process(raw2);
// float filtered_3 = filters[2].process(raw3);
