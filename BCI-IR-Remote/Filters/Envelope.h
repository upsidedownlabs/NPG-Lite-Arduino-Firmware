#pragma once

// EMG envelope - BioAmp EXG Pill
// https://github.com/upsidedownlabs/BioAmp-EXG-Pill
//
// Moving average of the rectified signal. Same algorithm as the reference
// sketch, wrapped per instance so each channel keeps its own window.
//
// Window size trade-off:
//   large -> smooth but less responsive
//   small -> responsive but noisy

#ifndef ENVELOPE_BUFFER_SIZE
#define ENVELOPE_BUFFER_SIZE 64
#endif

class Envelope {
private:
    int buffer[ENVELOPE_BUFFER_SIZE] = {0};
    int index = 0;
    long sum = 0;

public:
    int process(int absEmg) {
        sum -= buffer[index];
        sum += absEmg;
        buffer[index] = absEmg;
        index = (index + 1) % ENVELOPE_BUFFER_SIZE;
        return (int)((sum / ENVELOPE_BUFFER_SIZE) * 2);
    }

    void reset() {
        for (int i = 0; i < ENVELOPE_BUFFER_SIZE; i++) buffer[i] = 0;
        index = 0;
        sum = 0;
    }
};

// Example usage:
// Single channel:
// Envelope envelope;
// int value = envelope.process(abs(filtered));
//
// Multi-channel (2 channels):
// Envelope envelopes[2];
// int value0 = envelopes[0].process(abs(filtered0));
// int value1 = envelopes[1].process(abs(filtered1));
