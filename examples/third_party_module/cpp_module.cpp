// SPDX-License-Identifier: MIT
// Two modules written with the C++ layer of the SDK (crucible/sdk/Module.h): a sine generator and a one-pole low-pass,
// both nodes of the voice graph. The library links against nothing of the engine.

#include <crucible/sdk/Module.h>

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kTwoPi = 6.283185307179586;
}

// A sine voice: pitch follows the note, the detune parameter and the "pitch" modulation input (24 semitones per unit).
class SineGenerator
{
public:
    static crucible::sdk::Info info()
    {
        using namespace crucible::sdk;
        Info i;
        i.id = "acme.cpp_sine";
        i.name = "C++ Sine";
        i.vendor = "Acme";
        i.category = "Oscillator";
        i.description = "A sine generator written with the C++ layer of the module SDK.";
        i.kind = Kind::Generator;
        i.params = {Param::floating("level", "Level", 0.0f, 1.0f, 0.5f),
                    Param::floating("detune", "Detune", -100.0f, 100.0f, 0.0f).unit("cents")};
        i.inputs = {{"pitch", PortKind::Modulation}};
        i.outputs = {{"out", PortKind::AudioStereo}};
        return i;
    }

    void prepare(const crucible::sdk::Prepare& p) { rate = p.sampleRate; }
    void reset() { phase = 0.0; }
    void noteOn(const crucible::sdk::Note&) { phase = 0.0; }

    void process(crucible::sdk::Process& p)
    {
        const double semitones = static_cast<double>(p.note()) + p.param(1) * 0.01 + 24.0 * p.mod(0);
        const double step = kTwoPi * 440.0 * std::exp2((semitones - 69.0) / 12.0) / rate;
        const float gain = p.param(0) * p.velocity();
        float* l = p.left();
        float* r = p.right();
        for (int i = 0; i < p.frames(); ++i)
        {
            const float s = static_cast<float>(std::sin(phase)) * gain;
            l[i] = s;
            r[i] = s;
            phase += step;
            if (phase > kTwoPi)
                phase -= kTwoPi;
        }
    }

    // Four voices in one call. A real module would use SIMD here; the host falls back to process() when this returns false.
    static bool processQuad(SineGenerator* const (&voices)[4], crucible::sdk::Process (&data)[4])
    {
        for (int k = 0; k < 4; ++k)
            voices[k]->process(data[k]);
        return true;
    }

private:
    double rate = 48000.0;
    double phase = 0.0;
};

// A one-pole low-pass with a cutoff modulation input (5 octaves per unit).
class OnePole
{
public:
    static crucible::sdk::Info info()
    {
        using namespace crucible::sdk;
        Info i;
        i.id = "acme.cpp_onepole";
        i.name = "C++ One-Pole";
        i.vendor = "Acme";
        i.category = "Filter";
        i.description = "A one-pole low-pass filter written with the C++ layer of the module SDK.";
        i.kind = Kind::Filter;
        i.params = {Param::floating("cutoff", "Cutoff", 20.0f, 20000.0f, 2000.0f).unit("Hz").logSmoothing().skew(0.3f)};
        i.inputs = {{"in", PortKind::AudioStereo}, {"cutoff", PortKind::Modulation}};
        i.outputs = {{"out", PortKind::AudioStereo}};
        return i;
    }

    bool prepare(const crucible::sdk::Prepare& p)
    {
        rate = p.sampleRate;
        return true;
    }
    void reset() { z[0] = z[1] = 0.0f; }
    void noteOn(const crucible::sdk::Note&) { reset(); }

    void process(crucible::sdk::Process& p)
    {
        const double fc = std::min(static_cast<double>(p.param(0)) * std::exp2(5.0 * p.mod(1)), 0.45 * rate);
        const float a = static_cast<float>(1.0 - std::exp(-kTwoPi * fc / rate));
        const float* in[2] = {p.inLeft(), p.inRight()};
        float* out[2] = {p.left(), p.right()};
        for (int c = 0; c < 2; ++c)
            for (int i = 0; i < p.frames(); ++i)
            {
                z[c] += a * (in[c][i] - z[c]);
                out[c][i] = z[c];
            }
    }

private:
    double rate = 48000.0;
    float z[2] = {0.0f, 0.0f};
};

CRUCIBLE_SDK_EXPORT_MODULES(SineGenerator, OnePole)
