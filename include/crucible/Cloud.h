// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/RtShared.h>
#include <crucible/UserWavetables.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace crucible
{

// The 3D cloud: a point cloud of partials that defines a sound.
//
//   x  partial number (1 = fundamental, 2 = octave, 2.5 = inharmonic between them), 1..kCloudMaxPartial
//   y  level 0..1
//   z  position 0..1 along the morph axis
//   phase  start phase in cycles 0..1
//   pan    -1..1 (direct rendering only; wavetables are mono)
//   random 0..1: per-note jitter of level, phase and (direct rendering) a slight detune
//
// At a morph position p every point contributes with weight exp(-0.5 * ((z - p) / spread)^2), so a point is
// heard near its z and fades in and out as the position moves past it. The cloud is used two ways:
//   - toWavetable(): 64 frames along z, written into a user wavetable slot, played by the normal oscillator;
//   - CloudOscillator: additive rendering of the cloud at any pitch (inharmonic x allowed), real-time safe.
// Partials above 0.45 x the sample rate are skipped (band-limited). Everything is deterministic for a given seed.
constexpr int kCloudMaxPoints = 512;
constexpr float kCloudMaxPartial = 512.0f;

// Spectral drawbars: the partials are grouped into eight octave bands by partial number x:
// [1,2) 1, [2,3) 2, [3,5) 3-4, [5,9) 5-8, [9,17) 9-16, [17,33) 17-32, [33,65) 33-64, 65 and above.
constexpr int kCloudBands = 8;
constexpr int cloudBandOf(float x)
{
    if (x < 2.0f)
        return 0;
    if (x < 3.0f)
        return 1;
    int band = 2;
    for (float edge = 5.0f; band < kCloudBands - 1 && x >= edge; edge = (edge - 1.0f) * 2.0f + 1.0f)
        ++band;
    return band;
}

struct CloudPoint
{
    float x = 1.0f;
    float y = 1.0f;
    float z = 0.0f;
    float phase = 0.0f;
    float pan = 0.0f;
    float random = 0.0f;
};

struct CloudModel
{
    std::vector<CloudPoint> points;
    float spread = 0.15f; // morph influence width along z
    std::string name;

    // Clamps every field into range and drops points beyond kCloudMaxPoints.
    void sanitise();
    // Level of point i at morph position p (0..1), before per-note randomisation.
    float weight(const CloudPoint& pt, float position) const { return weightWithSpread(pt, position, spread); }
    float weightWithSpread(const CloudPoint& pt, float position, float spreadOverride) const;

    // Plain-text serialisation, one point per line "x y z phase pan random", header "cloud 1 <spread> <name>".
    std::string serialise() const;
    static bool deserialise(const std::string& text, CloudModel& out);

    // Generators for starting points: a saw-like harmonic series morphing to a sparse bell spectrum, and a
    // seeded random cloud.
    static CloudModel harmonicMorph(int partials = 64);
    static CloudModel random(int points, std::uint32_t seed);
};

// 64 single cycles (2048 samples) along z: harmonics are rounded to the nearest integer partial, level by the
// morph weight, phase from the point. Feed to UserWavetableBank::loadCycles().
std::vector<std::vector<float>> cloudToCycles(const CloudModel& model);

// What a 3D viewer polls: the model's points, the current morph slice and the energy each point contributes.
struct CloudSnapshot
{
    std::vector<CloudPoint> points;
    float position = 0.0f;
    std::vector<float> energy; // per point, 0..1 (level x weight of the most recently rendered note)
    int activeNotes = 0;
};

// Additive renderer for one note at a time per instance (hosts keep one per voice). setModel() runs on a control
// thread; render() on the audio thread, allocation and lock free.
class CloudOscillator
{
public:
    void prepare(double sampleRate);
    void setModel(const CloudModel& model); // control thread
    const CloudModel* model() const { return shared.latest(); }

    void noteOn(float frequencyHz, std::uint32_t seed);
    void noteOff() { active = false; }
    bool isActive() const { return active.load(std::memory_order_relaxed); }
    // Adds the cloud at morph position into left / right (gain applied by the caller).
    void render(float* left, float* right, int numSamples, float position, float frequencyHz);

    CloudSnapshot snapshot() const; // any thread except the audio thread (copies the model)

    // Voice use: render a model owned elsewhere (pinned by the caller for the call), with the morph width and a
    // pan width scale overriding the model's own spread (spreadOverride <= 0 keeps it). bandGains (kCloudBands
    // values, 0..2) scale the spectral drawbars; null means all ones, which renders exactly as without them.
    void renderModel(const CloudModel* model, float* left, float* right, int numSamples, float position, float frequencyHz,
                     float spreadOverride, float panScale, const float* bandGains = nullptr);
    // Energy per point (0..1) of the most recently rendered chunk; any thread.
    void energies(std::vector<float>& out, size_t count) const;
    float lastMorphPosition() const { return lastPosition.load(std::memory_order_relaxed); }

private:
    struct Partial
    {
        double re = 1.0, im = 0.0; // rotator state
        float gainL = 0.0f, gainR = 0.0f;
        float jitterLevel = 1.0f;
        float detune = 1.0f;
    };
    RtShared<CloudModel> shared;
    std::array<Partial, kCloudMaxPoints> partials{};
    std::array<std::atomic<float>, kCloudMaxPoints> energy{};
    std::atomic<float> lastPosition{0.0f};
    std::uint32_t noteSeed = 1;
    const CloudModel* lastModel = nullptr;
    std::atomic<bool> active{false};
    bool fresh = false;
    double sr = 48000.0;
};

// Implemented by the voice's cloud oscillator instances so the engine can read their snapshot data.
class CloudOscillatorView
{
public:
    virtual ~CloudOscillatorView() = default;
    virtual CloudOscillator& cloudOscillator() = 0;
};

// One cloud per oscillator slot, published to the audio thread without locks. Every slot starts with the harmonic
// morph model.
class CloudBank
{
public:
    CloudBank();
    void set(int slot, const CloudModel& model); // control thread; sanitised
    void reset(int slot);                        // back to the default
    bool isUser(int slot) const;
    const CloudModel* model(int slot) const; // control thread
    void collectGarbage();

    // User slots as text (CloudModel::serialise()), for plugin state and patches. deserialise() resets every slot
    // first; records that do not parse are skipped.
    struct Record
    {
        int slot = 0;
        std::string text;
    };
    std::vector<Record> serialise() const;
    void deserialise(const std::vector<Record>& records);

    // Any thread except the audio thread: runs f(const CloudModel&) while the model cannot be replaced or freed.
    template <typename F> bool withModel(int slot, F&& f) const
    {
        return slot >= 0 && slot < static_cast<int>(slots.size()) && slots[static_cast<size_t>(slot)].withLatest(f);
    }

    using Pinned = std::array<const CloudModel*, 8>;
    void acquire(Pinned& out) noexcept;
    void release() noexcept;

private:
    std::array<RtShared<CloudModel>, 8> slots;
    std::array<bool, 8> user{};
};

} // namespace crucible
