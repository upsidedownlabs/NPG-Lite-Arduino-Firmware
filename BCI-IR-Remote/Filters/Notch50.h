#pragma once

// Band-Stop Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// Reference: https://github.com/upsidedownlabs/BioAmp-Filter-Designer

class Notch50 {
private:
    struct BiquadState { float z1 = 0, z2 = 0; };
    BiquadState state0;
    BiquadState state1;

public:
    float process(float input) {
        float output = input;

        // Biquad section 0
        float x0 = output - (-1.56858163f * state0.z1) - (0.96424138f * state0.z2);
        output = 0.96508099f * x0 + -1.56202714f * state0.z1 + 0.96508099f * state0.z2;
        state0.z2 = state0.z1;
        state0.z1 = x0;

        // Biquad section 1
        float x1 = output - (-1.61100358f * state1.z1) - (0.96592171f * state1.z2);
        output = 1.00000000f * x1 + -1.61854514f * state1.z1 + 1.00000000f * state1.z2;
        state1.z2 = state1.z1;
        state1.z1 = x1;

        return output;
    }

    void reset() {
        state0.z1 = state0.z2 = 0;
        state1.z1 = state1.z2 = 0;
    }
};

// Example usage:
// Single channel:
// Notch50 filter;
// filter.reset();
// float filtered_output = filter.process(sample);
// 
// Multi-channel (3 channels):
// Notch50 filters[3];  // One filter per channel
// float filtered_1 = filters[0].process(raw1);
// float filtered_2 = filters[1].process(raw2);
// float filtered_3 = filters[2].process(raw3);
