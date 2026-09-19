#pragma once

// Mains notch, switchable at runtime.
//
// Both designs are kept and one is chosen per sample, because the mains
// frequency is a setting the app can change without a reflash. Holding both
// costs four floats of state, which is cheaper than rebuilding coefficients.

#include "Notch50.h"
#include "Notch60.h"

class Notch {
private:
    Notch50 f50;
    Notch60 f60;
    bool    use60 = false;

public:
    // hz is 50 or 60; anything else leaves the current setting alone
    void setFrequency(int hz) {
        if (hz != 50 && hz != 60) return;
        bool next = (hz == 60);
        if (next != use60) {
            use60 = next;
            reset();
        }
    }

    int frequency() const { return use60 ? 60 : 50; }

    float process(float input) {
        return use60 ? f60.process(input) : f50.process(input);
    }

    void reset() {
        f50.reset();
        f60.reset();
    }
};
