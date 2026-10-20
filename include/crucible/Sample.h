// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/AudioFile.h>
#include <crucible/RtShared.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace crucible
{

// Imported audio for the sample / granular oscillator. Besides the original, the sample keeps band-limited and
// decimated copies (two per octave, cutoff falling by sqrt 2 per step), so pitched playback can pick a copy whose
// content cannot fold back below 20 kHz at the playback rate. Built on a control thread; immutable afterwards.
struct SampleLevel
{
    int decimation = 1;
    double cutoff = 0.5; // cycles per source sample (of the original rate)
    int frames = 0;
    // Per channel (data[1] stays empty for mono), padded by kSamplePad zeros on both sides.
    std::array<std::vector<float>, 2> data;
};

constexpr int kSamplePad = 16;
constexpr int kSampleMaxChannels = 2;
constexpr int kSampleMaxFrames = 48000 * 60 * 4; // four minutes at 48 kHz
constexpr float kSampleMaxValue = 16.0f; // imported sample values are clamped to +-16 (non-finite become 0)

struct SampleData
{
    std::string name;
    double sampleRate = 48000.0;
    int frames = 0;
    int channels = 1;
    std::vector<SampleLevel> levels; // level 0 is the original
    // Waveform overview: kSamplePeakColumns min / max pairs of the channel mix.
    std::vector<float> peakMin;
    std::vector<float> peakMax;
    // Detected pitch in MIDI note units (autocorrelation), or -1; used as the default root note by UIs.
    float detectedNote = -1.0f;

    // The level to read at `ratio` source samples per output sample for an engine running at outputRate.
    const SampleLevel& levelFor(double ratio, double outputRate) const
    {
        return levels[static_cast<size_t>(levelIndexFor(ratio, outputRate))];
    }
    int levelIndexFor(double ratio, double outputRate) const;
    static std::unique_ptr<SampleData> build(const AudioData& audio, std::string name);
};
constexpr int kSamplePeakColumns = 1024;

// Slice markers of a sample: positions (0..1 along the sample) where a new slice starts. Slice 0 starts at 0, the last
// slice ends at 1, so n markers make n + 1 slices. Immutable once published; the sample oscillator plays one slice per
// note (see SampleBank::setSlices).
constexpr int kMaxSlices = 128;
struct SliceSet
{
    std::vector<float> markers; // ascending, strictly inside (0, 1)

    int count() const { return static_cast<int>(markers.size()) + 1; }
    // Source frames [begin, end) of a slice (index is clamped).
    void region(int index, int frames, double& begin, double& end) const;
    // Sorts, clamps into the sample, drops markers closer than 1/2048 of the sample and caps at kMaxSlices - 1.
    static std::unique_ptr<SliceSet> make(std::vector<float> markers);
};
// Detected pitch of a mono signal in MIDI note units (autocorrelation over the loudest window), or -1 when the
// signal is too short, too quiet or not clearly pitched. Control thread.
float detectSamplePitch(const std::vector<float>& mono, double rate);

// Zero crossings, for loop points and snapping. Positions are normalised 0..1 over the sample length and refer to
// the channel mix. Control thread (reads the original level only).
// Nearest rising zero crossing (negative to non-negative) to pos01 in direction -1 (earlier), +1 (later) or 0
// (either side); the position is interpolated between the two samples. Returns pos01 unchanged when there is none.
double nearestZeroCrossing(const SampleData& sample, double pos01, int direction);
// Searches around the given loop region for a click-free loop: start and end are moved to rising zero crossings
// (at most 5 percent of the region each) so that the waveform slope and value match across the seam as well as
// possible. Returns false (outputs = inputs, clamped) when the region is empty or no crossing exists.
bool findLoopPoints(const SampleData& sample, double start01, double end01, double& loopStart01, double& loopEnd01);

// Windowed-sinc interpolation kernel tables (16 taps and 8 taps, linear between phases). Built once; call
// SincKernel::instance() from a control thread before the first use on an audio thread.
class SincKernel
{
public:
    static constexpr int phases = 512;
    static const SincKernel& instance();

    template <int Taps> float read(const float* x0, int index, float frac) const
    {
        // x0[index] is the sample at or before the position, frac in [0, 1).
        const auto& t = Taps == 16 ? t16 : t8;
        const float fp = frac * static_cast<float>(phases);
        const int p = static_cast<int>(fp);
        const float f = fp - static_cast<float>(p);
        const float* c = t.data() + static_cast<size_t>(p) * (2 * Taps);
        const float* d = c + Taps;
        const float* x = x0 + index - (Taps / 2 - 1);
        float acc = 0.0f;
        for (int j = 0; j < Taps; ++j)
            acc += x[j] * (c[j] + f * d[j]);
        return acc;
    }

private:
    SincKernel();
    std::vector<float> t16; // per phase: Taps coefficients then Taps deltas
    std::vector<float> t8;
};

// Samples for the oscillator slots (one per slot), published to the audio thread without locks.
class SampleBank
{
public:
    bool load(int slot, const AudioData& audio, const std::string& name, std::string& error);
    bool loadFile(int slot, const std::string& wavPath, std::string& error);
    // Publishes a sample that was built elsewhere (SampleData::build on a worker thread), so that the control thread
    // only swaps a pointer. Removes the slot's slice markers like load(). False for a bad slot or a null sample.
    bool install(int slot, std::unique_ptr<SampleData> built);
    void clear(int slot);
    bool isLoaded(int slot) const;
    // The loaded sample (control thread; valid until the slot is replaced) or nullptr.
    const SampleData* sample(int slot) const;
    // Any thread except the audio thread: runs f(const SampleData&) on the slot's sample while it cannot be replaced
    // or freed. False when the slot is empty.
    template <typename F> bool withSample(int slot, F&& f) const
    {
        return slot >= 0 && slot < static_cast<int>(slots.size()) && slots[static_cast<size_t>(slot)].withLatest(f);
    }
    // The original audio, for patches. False when the slot is empty.
    bool original(int slot, AudioData& out, std::string& name) const;
    void collectGarbage();

    // Slice markers (control thread). Loading or clearing a sample removes its markers. setSlices() fails when the
    // slot holds no sample. An empty list means one slice (the whole sample).
    bool setSlices(int slot, std::vector<float> markers);
    std::vector<float> slices(int slot) const; // any thread except the audio thread
    // n equal slices (n clamped to 1..kMaxSlices); returns the slice count.
    int sliceEqually(int slot, int n);
    // Markers at detected onsets (rises of the short-term energy), at most maxSlices slices; sensitivity 0..1 (higher
    // finds more). Returns the slice count.
    int sliceByTransients(int slot, int maxSlices = 32, float sensitivity = 0.5f);
    void clearSlices(int slot) { setSliceSet(slot, nullptr); }
    // Original audio of every loaded slot (interleaved float frames), for plugin state and patches. deserialise()
    // clears every slot, then loads the records that decode; control thread, allocates.
    struct Record
    {
        int slot = 0;
        std::string name;
        double rate = 48000.0;
        int channels = 1;
        std::vector<float> frames;
    };
    std::vector<Record> serialise() const;
    void deserialise(const std::vector<Record>& records);

    using Pinned = std::array<const SampleData*, 8>;
    void acquire(Pinned& out) noexcept;
    void release() noexcept;
    using PinnedSlices = std::array<const SliceSet*, 8>;
    void acquireSlices(PinnedSlices& out) noexcept;
    void releaseSlices() noexcept;

private:
    void setSliceSet(int slot, std::unique_ptr<SliceSet> set);
    std::array<RtShared<SampleData>, 8> slots;
    std::array<RtShared<SliceSet>, 8> sliceSlots;
};

} // namespace crucible
