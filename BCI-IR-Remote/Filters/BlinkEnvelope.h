#pragma once

// Envelope for blink detection.
//
// A moving average of the rectified EOG signal, the same shape as Envelope.h
// but kept separate because blinks want their own window. The reference blink
// sketch smooths over 100 ms, which is long enough to turn the sharp edges of
// a blink into one clean bump and short enough that two deliberate blinks stay
// apart. It returns the plain mean, so the level it reports is directly
// comparable to the blink threshold.

#ifndef BLINK_ENVELOPE_MS
#define BLINK_ENVELOPE_MS 100
#endif

template <int WINDOW>
class BlinkEnvelopeT {
private:
    float buffer[WINDOW] = {0};
    int   index = 0;
    float sum = 0;

public:
    float process(float sample) {
        float rectified = fabsf(sample);
        sum -= buffer[index];
        sum += rectified;
        buffer[index] = rectified;
        index = (index + 1) % WINDOW;
        return sum / (float)WINDOW;
    }

    void reset() {
        for (int i = 0; i < WINDOW; i++) buffer[i] = 0.0f;
        index = 0;
        sum = 0.0f;
    }
};

// Example usage, with the window sized from your sample rate:
// typedef BlinkEnvelopeT<(BLINK_ENVELOPE_MS * SAMPLE_RATE) / 1000> BlinkEnvelope;
// BlinkEnvelope blinkEnv;
// float level = blinkEnv.process(eogFiltered);
