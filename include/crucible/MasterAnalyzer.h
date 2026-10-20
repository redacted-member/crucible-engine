// SPDX-License-Identifier: MIT
#pragma once

// Master-bus analysis for the synth's single meter set: loudness (ITU-R BS.1770 K-weighted, momentary 400 ms and
// short-term 3 s), true peak (4x oversampled), stereo correlation, and a scope buffer from which a control thread
// takes oscilloscope and spectrum snapshots. The loudness and true-peak measurement is crucible::fx::LoudnessMeter, the
// implementation the maximiser and the meter module use as well. The audio side only filters, accumulates and copies into a
// ring; spectra are computed by the caller of spectrum(). Disable it to skip all of that work.

#include <crucible/fx/Loudness.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

namespace crucible
{

struct LoudnessSnapshot
{
    float momentaryLufs = -120.0f;
    float shortTermLufs = -120.0f;
    float truePeakDb = -120.0f;    // decaying (about 1.5 s release)
    float maxTruePeakDb = -120.0f; // since the last reset
    float correlation = 0.0f;      // -1..1 over the last 100 ms (0 for silence)
};

class MasterAnalyzer
{
public:
    static constexpr int scopeSize = 4096;

    void prepare(double sampleRate);
    void setEnabled(bool on) { enabled.store(on, std::memory_order_relaxed); }
    bool isEnabled() const { return enabled.load(std::memory_order_relaxed); }

    // Audio thread.
    void process(const float* left, const float* right, int numSamples);

    // Any thread.
    LoudnessSnapshot loudness() const;
    // Forgets the largest true peak; the measurement itself restarts at the next process() call.
    void resetMax()
    {
        maxTruePeak.store(0.0f, std::memory_order_relaxed);
        resetRequested.store(true, std::memory_order_relaxed);
    }

    // Control thread: the most recent count (<= scopeSize) samples per channel, oldest first. A read racing the
    // audio thread may mix two consecutive blocks; fine for display.
    void scope(std::vector<float>& left, std::vector<float>& right, int count) const;
    // Control thread: magnitude spectrum (dB re full-scale sine) of the most recent fftSize (power of two,
    // 256..4096) mono samples with a Hann window; fftSize / 2 + 1 bins from 0 Hz to Nyquist.
    std::vector<float> spectrum(int fftSize = 2048) const;

    // Allocation free: the same samples as scope(), written to left / right (count <= scopeSize entries each).
    void copyScope(float* left, float* right, int count) const;
    // The FFT length spectrum() uses for a requested size (a power of two, 256..4096).
    static int fftLength(int fftSize);
    // The spectrum of count = fftLength(fftSize) mono samples given as the two channels (what spectrum() computes from the
    // scope). Used by the spectrum views, which work from a published scope snapshot.
    static std::vector<float> spectrumOf(const float* left, const float* right, int fftSize);

private:
    std::atomic<bool> enabled{true};
    std::atomic<bool> resetRequested{false};
    double sr = 48000.0;
    fx::LoudnessMeter meter;
    // Correlation over 100 ms blocks.
    int blockFill = 0;
    int blockLength = 4800;
    double sumLR = 0.0, sumLL = 0.0, sumRR = 0.0;
    std::array<float, scopeSize> scopeL{}, scopeR{};
    std::atomic<int> scopeWrite{0};
    std::atomic<float> momentary{-120.0f}, shortTerm{-120.0f}, truePeak{0.0f}, maxTruePeak{0.0f},
        correlationValue{0.0f}; // LUFS, LUFS, linear
};

} // namespace crucible
