// SPDX-License-Identifier: MIT
#pragma once

// Spectral tools for wavetables (P13): spectral warps and spectral frame interpolation.
//
// A wavetable frame is one cycle of 2048 samples, so its spectrum lives on the integer harmonics 1 .. 1023. Linear
// interpolation between two frames (the engine's default) is a crossfade of the two spectra: a spectral peak in frame A
// fades out while the peak of frame B fades in. Spectral sweep interpolation moves the peaks instead: the peaks of the two
// frames are matched (control thread, SpectralTable::analyse), the frequency axes of both spectra are warped piecewise
// linearly so that matched peaks meet at the interpolated position, and the warped spectra are blended in amplitude. A
// formant at harmonic 8 in frame A and at harmonic 24 in frame B sits at harmonic 16 halfway; a crossfade shows two half
// strength formants at 8 and 24. Energy is kept equal to the interpolation of the two frame energies.
//
// Threading: SpectralTable::analyse allocates and transforms (control thread, once per table). SpectralSynth is
// built on a control thread; render() only does arithmetic on preallocated arrays and an inverse transform, no analysis,
// allocation or locks, and may be called from one audio thread at a time (give each engine its own).

#include <crucible/Fft.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace crucible::spectral
{

constexpr int kCycle = 2048;          // samples per analysed frame (WavetableBank::tableSize)
constexpr int kBins = kCycle / 2;     // arrays are indexed by harmonic number 0 .. kBins - 1
constexpr int kHarmonics = kBins - 1; // highest harmonic of a frame
constexpr int kMaxFrames = 64;
constexpr int kMaxPeaks = 24;
constexpr int kMaxSynthSize = 4096; // largest cycle length SpectralSynth renders
constexpr int kMinSynthSize = 128;

// Interpolation between neighbouring frames of a table (oscN_wt_interp).
enum class Interp
{
    Crossfade = 0, // the legacy linear blend
    Sweep = 1      // peaks move in frequency
};

// Spectral warps applied after interpolation (oscN_wt_spec), amount -1 .. 1; the level is matched to the unwarped frame.
enum class Warp
{
    Off = 0,
    Shift,      // every harmonic moves up (amount > 0) or down by up to 32 harmonics
    Stretch,    // the harmonic axis is scaled by 2^amount (inharmonic partials are rounded onto the grid by interpolation)
    Blur,       // magnitudes are smeared over +-16 harmonics x |amount|
    Comb,       // notches (amount > 0) or boosts (amount < 0) every 2 .. 16 harmonics
    Dispersion, // all-pass phase rotation growing with the square of the harmonic number
    Tilt,       // +-12 dB per octave of harmonic number
    Count
};

const char* warpName(Warp w);

struct WarpSettings
{
    Warp mode = Warp::Off;
    float amount = 0.0f;
};

// Harmonic-domain description of a cycle: x[n] = sum_k Re((re[k] + i im[k]) e^(i 2 pi k n / N)), k = 1 .. kHarmonics.
struct Spectrum
{
    std::array<float, kBins> re{};
    std::array<float, kBins> im{};
};

struct Peak
{
    float position = 0.0f; // harmonic number (fractional)
    float magnitude = 0.0f;
};

struct Match
{
    int a = 0; // index into the peaks of the first frame
    int b = 0; // index into the peaks of the second frame
};

// Analysis of up to kMaxFrames frames: complex harmonic amplitudes, spectral peaks and the peak matching between
// neighbouring frames. Immutable once built.
class SpectralTable
{
public:
    // frame(i) returns kCycle samples. Control thread: allocates and runs FFTs.
    static std::unique_ptr<SpectralTable> analyse(int numFrames, const std::function<const float*(int)>& frame);
    // Number of analyse() calls in the process (tests assert that rendering never analyses).
    static long analysisCount() noexcept;

    int frames() const noexcept { return count; }
    // Unique among the analyses of the process: a cache of synthesised cycles tells a replaced table from the old one.
    std::uint64_t id() const noexcept { return serial; }
    const float* re(int frame) const noexcept { return data.data() + static_cast<size_t>(frame) * 2 * kBins; }
    const float* im(int frame) const noexcept { return re(frame) + kBins; }
    float energy(int frame) const noexcept { return energies[static_cast<size_t>(frame)]; }
    const std::vector<Peak>& peaks(int frame) const noexcept { return peakList[static_cast<size_t>(frame)]; }
    // Matches between frame and frame + 1.
    const std::vector<Match>& matches(int frame) const noexcept { return matchList[static_cast<size_t>(frame)]; }

private:
    int count = 0;
    std::uint64_t serial = 0;
    std::vector<float> data;
    std::vector<float> energies;
    std::vector<std::vector<Peak>> peakList;
    std::vector<std::vector<Match>> matchList;
};

// Spectra from outside (tests, tools): the peaks of one magnitude curve and their matching.
std::vector<Peak> findPeaks(const float* magnitude);
std::vector<Match> matchPeaks(const std::vector<Peak>& a, const std::vector<Peak>& b);

// Renders interpolated, warped cycles. Not copyable; one per engine (the transforms use per-instance scratch memory).
class SpectralSynth
{
public:
    SpectralSynth(); // control thread: allocates the transforms and scratch
    ~SpectralSynth();
    SpectralSynth(const SpectralSynth&) = delete;
    SpectralSynth& operator=(const SpectralSynth&) = delete;

    // The interpolated spectrum between frame and frame + 1 at mix 0 .. 1 (before any warp).
    void interpolate(const SpectralTable& table, int frame, float mix, Interp interp, Spectrum& out);
    // The warp alone: in -> out (in and out must differ).
    void warp(const WarpSettings& settings, const Spectrum& in, Spectrum& out);
    // One cycle of `size` samples (a power of two from kMinSynthSize to kMaxSynthSize) into out[0 .. size), with the cycle
    // repeated in out[-2], out[-1] and out[size .. size + 2] (the guard samples of the wavetable bank's frames). Only
    // harmonics 1 .. min(harmonics, size / 2 - 1) sound. Returns false (and writes nothing) for an unsupported size.
    bool render(const SpectralTable& table, int frame, float mix, Interp interp, const WarpSettings& warpSettings,
                int harmonics, int size, float* out);
    // The same for a spectrum.
    bool synthesise(const Spectrum& spectrum, int harmonics, int size, float* out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

// Spectral analysis of the factory wavetables, built on demand from a control thread and kept for the life of the process
// (like the tables themselves). get() is lock free and returns null until ensure() has run for the table.
class FactoryTables
{
public:
    static FactoryTables& instance();
    void ensure(int table);
    const SpectralTable* get(int table) const noexcept;

private:
    FactoryTables() = default;
    std::mutex mutex;
    std::array<std::atomic<const SpectralTable*>, 16> tables{};
    std::vector<std::unique_ptr<SpectralTable>> owned;
};

} // namespace crucible::spectral
