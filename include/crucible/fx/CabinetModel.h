// SPDX-License-Identifier: MIT
#pragma once

// Parametric guitar / bass speaker-cabinet model (fx.cabinet). No recorded impulse responses are used anywhere: the
// magnitude response of a cabinet is written down from public acoustics (the low-frequency resonance of the driver in its
// box, the cone's breakup peaks and dips, the roll-off of the voice coil and cone, the reflections off a back wall for an
// open back, the microphone's off-axis tilt and proximity effect), and the impulse response is the minimum-phase filter
// that has exactly that magnitude (real-cepstrum folding), plus one early room reflection at the microphone distance.
//
// Everything here is a pure function of its arguments; it allocates and runs on the control thread only.

#include <vector>

namespace crucible::fx
{

constexpr int kCabinetCount = 8;

// One speaker-cabinet design. Frequencies in Hz, levels in dB.
struct CabinetSpec
{
    struct Bell
    {
        double hz = 0.0;
        double gainDb = 0.0;
        double q = 1.0; // the bump is a Gaussian in log frequency of width 0.6 / q (nepers)
    };
    const char* name = "";
    // Low-frequency resonance of the driver(s) in the box: the peak of the response sits at resonanceHz with the height the
    // quality factor gives (a second-order high-pass, Q above 0.707). A ported box adds a second, lower high-pass corner.
    double resonanceHz = 100.0;
    double resonanceQ = 1.0;
    bool ported = false;
    // Roll-off of the top end: a Butterworth-type low-pass of the given order (6 dB per octave per order), placed so that, above
    // rolloffHz, the finished response (half-octave smoothed) stays more than 3 dB below its highest level in the presence
    // region (0.3 * rolloffHz up to rolloffHz), i.e. rolloffHz is the last frequency within 3 dB of that peak, for the default microphone (centre, 8 cm).
    double rolloffHz = 5000.0;
    int rolloffOrder = 3;
    // Cone breakup: broad peaks and dips between 1 and 6 kHz.
    Bell bells[4]{};
    int bellCount = 0;
    // Back / baffle reflection: the direct sound plus a copy delayed by combDelayMs at combGain gives a first notch at
    // 1 / (2 * delay). An open back reflects with the same sign here, a closed back barely at all (combGain small).
    double combDelayMs = 0.0;
    double combGain = 0.0;
    // Dust-cap / cone-edge notch the microphone hears near the centre; it moves down and deepens toward the edge.
    double notchHz = 2500.0;
};

const CabinetSpec& cabinetSpec(int index);

struct CabinetSettings
{
    int cabinet = 0;
    float micPosition = 0.0f;    // 0 = on the dust cap, 1 = at the cone edge
    float micDistanceCm = 5.0f;  // 0 to 60
};

// Longest impulse response length in samples at a sample rate: 2048 at 48 kHz, scaled with the rate (at most 8192).
int cabinetTapCount(double sampleRate);

// The designed magnitude response (linear, normalised so that its mean power between 400 Hz and 2.5 kHz is one) of the
// cabinet at a frequency: what generateCabinetIr() realises. Room reflection not included.
double cabinetMagnitude(const CabinetSettings& settings, double hz);

// Frequency of the low-frequency peak of cabinetSpec(i) at the default microphone (equals resonanceHz by calibration of
// the high-pass corner).
double cabinetResonanceHz(int cabinet);

// The minimum-phase impulse response, cabinetTapCount(rate) samples, level-normalised as above. Control thread.
void generateCabinetIr(const CabinetSettings& settings, double sampleRate, std::vector<float>& out);

} // namespace crucible::fx
