// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/AudioFile.h>
#include <crucible/RtShared.h>
#include <crucible/Wavetables.h>

#include <array>
#include <utility>
#include <memory>
#include <string>
#include <vector>

namespace crucible
{

namespace spectral
{
class SpectralTable;
}

// User wavetables: imported audio (WAV or raw single cycles) or generated data (e.g. a 3D cloud), stored in eight
// slots that the oscillators select with table indices WavetableBank::numTables .. +7 ("User 1".."User 8").
// A user table has exactly the factory layout: 64 frames, 27 band-limited mip levels, so it plays through the
// normal wavetable oscillator (warps, unison, FM source) with the same anti-aliasing.
constexpr int kUserTables = 8;
constexpr int kTotalTables = WavetableBank::numTables + kUserTables;

// Factory names followed by "User 1".."User 8" (the choices of the osc*_table parameters).
const std::vector<std::string>& allTableNames();

struct WavetableImport
{
    // Samples per source frame. 0 = automatic: the file's 'clm ' marker if present, else 2048 when the length is
    // a multiple of 2048, else the whole file is one cycle when it is at most 8192 samples long, else 2048
    // (trailing samples that do not fill a frame are dropped).
    int frameSize = 0;
    enum class Normalise
    {
        Table, // one gain for the whole table (keeps level differences between frames)
        Frame, // every frame peaks at 1
        None
    };
    Normalise normalise = Normalise::Table;
    int channel = -1; // -1 = mix all channels to mono, else that channel
};

// One pitch range of a zoned table: a full set of source cycles (same rules as fromCycles) and the highest fundamental
// (Hz) the zone is meant for.
struct WavetableZone
{
    std::vector<std::vector<float>> cycles;
    double maxHz = 0.0;
};
constexpr int kMaxWavetableZones = 16;

class UserWavetable
{
public:
    UserWavetable();
    ~UserWavetable();
    // cycles: one or more single-cycle waveforms of any length (>= 4 samples). They are resampled spectrally to
    // the engine's 2048-sample frame, spread over 64 frames (with spectral crossfades when the count differs),
    // DC removed, normalised and band-limited per mip like the factory tables. Control thread: allocates.
    static std::unique_ptr<UserWavetable> fromCycles(const std::vector<std::vector<float>>& cycles,
                                                     WavetableImport::Normalise normalise, std::string name);
    // A table whose mip levels come from different sources per pitch range, so pitch-dependent formants (fixed in Hz)
    // survive across the keyboard. 1..kMaxWavetableZones zones with finite, positive, strictly ascending maxHz (the last
    // zone also covers everything above its maxHz); null when that is violated or a zone has no cycle of >= 4 samples.
    // Mip level m is band-limited from the zone containing the level's representative fundamental (the geometric centre
    // of the notes for which WavetableBank::mipForIncrement picks m at the output band limit) at sampleRate, so the
    // oscillator reads it at no extra cost. One gain normalises the whole table (Table mode: zones keep their relative
    // level; Frame mode: each frame of each zone peaks at 1). baseFrames() / the spectral view show the zone of the
    // lowest level (zone 0 unless the rate makes a higher one the lowest pitch). A one-zone table equals fromCycles().
    static std::unique_ptr<UserWavetable> fromZones(const std::vector<WavetableZone>& zones,
                                                    WavetableImport::Normalise normalise, std::string name,
                                                    double sampleRate);
    // The same zones rebuilt for another output rate (the level -> pitch map depends on it); null when the table is
    // not zoned (a one-zone table does not depend on the rate) or already built for that rate. Control thread.
    std::unique_ptr<UserWavetable> rebuiltForRate(double sampleRate) const;
    int zoneCount() const { return zoneN; }
    // The zone inputs of a table with two or more zones; empty for a one-zone table.
    const std::vector<WavetableZone>& zones() const { return zoneSources; }
    double sampleRate() const { return builtRate; }
    WavetableImport::Normalise normalisationMode() const { return normalisation; }
    // Restores a table from its own saved 64 x 2048 baseFrames(), taken verbatim (no resampling, crossfade or
    // normalising), so save / load / save is bit-exact. The mips and the spectral analysis are rebuilt from the frames.
    static std::unique_ptr<UserWavetable> fromBaseFrames(const std::vector<float>& frames, std::string name);

    // Same layout and guard samples as WavetableBank::frame().
    const float* frame(int mip, int frameIndex) const
    {
        return data.data() + mipOffset[static_cast<size_t>(mip)] +
               static_cast<size_t>(frameIndex) *
                   static_cast<size_t>(WavetableBank::mipSize(mip) + WavetableBank::guardBefore +
                                       WavetableBank::guardAfter) +
               WavetableBank::guardBefore;
    }

    const std::string& name() const { return label; }
    int sourceFrameCount() const { return sourceFrames; }
    // The 64 full-resolution frames (64 x 2048 floats, frame-major): the serialised form and what a UI draws.
    const std::vector<float>& baseFrames() const { return base; }
    // Spectral analysis of the 64 frames (peaks and their matching, for the spectral sweep interpolation), built with
    // the table on the control thread; never null for a table made by fromCycles().
    const spectral::SpectralTable* spectral() const { return spectralData.get(); }

private:
    static std::unique_ptr<UserWavetable> build(const std::vector<WavetableZone>& zones,
                                                WavetableImport::Normalise normalise, std::string name, bool verbatim,
                                                double sampleRate);
    std::vector<WavetableZone> zoneSources; // the inputs of a zoned table (2+ zones), kept to rebuild it for another rate
    int zoneN = 1;
    WavetableImport::Normalise normalisation = WavetableImport::Normalise::Table;
    double builtRate = 48000.0;
    std::vector<float> data;
    std::vector<size_t> mipOffset;
    std::vector<float> base;
    std::unique_ptr<spectral::SpectralTable> spectralData;
    std::string label;
    int sourceFrames = 0;
};

// How an import will cut the audio into frames, and why (what a UI shows as the detection caption).
struct FrameDetection
{
    enum class Reason
    {
        ClmMarker,    // the file's 'clm ' chunk
        Multiple2048, // the length is a multiple of 2048 samples
        SingleCycle,  // up to 8192 samples: the whole file is one cycle
        Fallback,     // none of the above: 2048-sample frames, trailing samples dropped
        Forced        // WavetableImport::frameSize was set explicitly
    };
    int frameSize = 0; // samples per source frame
    Reason reason = Reason::Fallback;
    int frames = 0; // complete frames sliceCycles() will produce
};
FrameDetection detectWavetableFrames(const AudioData& audio, const WavetableImport& options);
// Length in samples of one period of the audio's fundamental (the sample pitch detector, first loud window), or 0
// when no clear pitch is found. Used for "size from pitch".
int estimateCycleLength(const AudioData& audio);

// Splits audio into single cycles according to the import options.
std::vector<std::vector<float>> sliceCycles(const AudioData& audio, const WavetableImport& options);

class UserWavetableBank
{
public:
    // ---- Control thread.
    bool load(int slot, const AudioData& audio, const WavetableImport& options, std::string name, std::string& error);
    bool loadFile(int slot, const std::string& wavPath, const WavetableImport& options, std::string& error);
    bool loadCycles(int slot, const std::vector<std::vector<float>>& cycles, WavetableImport::Normalise normalise,
                    std::string name);
    // Loads a zoned table (see UserWavetable::fromZones) built for the current sample rate; false when refused.
    bool loadZones(int slot, const std::vector<WavetableZone>& zones, WavetableImport::Normalise normalise,
                   std::string name);
    // The output rate zoned tables are built for (default 48 kHz). A change rebuilds the zoned tables of every slot
    // and publishes them (control thread, like every load); SynthEngine::prepare calls it.
    void setSampleRate(double rate);
    void clear(int slot);
    bool isLoaded(int slot) const;
    std::string name(int slot) const;
    // The loaded table (for drawing), or nullptr. Valid until the slot is replaced; control thread only.
    const UserWavetable* table(int slot) const;
    // Any thread except the audio thread: runs f on the table of the slot with replacement and collection held off, so
    // it cannot be freed under the caller. False (f not called) for an empty slot. f must be short.
    template <typename F> bool withTable(int slot, F&& f) const
    {
        return slot >= 0 && slot < kUserTables && slots[static_cast<size_t>(slot)].withLatest(std::forward<F>(f));
    }
    // Frees superseded tables once the audio side has let go of them.
    void collectGarbage();

    struct Record
    {
        int slot = -1;
        std::string name;
        std::vector<float> frames; // 64 x 2048, as UserWavetable::baseFrames()
        // A zoned table (2+ zones) also carries its zone sources and normalisation mode (a WavetableImport::Normalise
        // value); deserialise() then rebuilds it with fromZones() for the current rate instead of taking frames verbatim.
        std::vector<WavetableZone> zones;
        int normalise = 0;
    };
    // The zone sources of a record as one float blob (exact: sizes and max_hz are split into 16-bit pieces), and back.
    // unpackZones returns false (record untouched) for a malformed blob. Used by the patch formats.
    static std::vector<float> packZones(const Record& r);
    static bool unpackZones(const std::vector<float>& blob, Record& r);
    std::vector<Record> serialise() const;
    // Clears every slot, then loads the records (frames are taken as-is, without renormalising).
    void deserialise(const std::vector<Record>& records);

    // ---- Audio thread: pin every slot for one render call, then release.
    using Pinned = std::array<const UserWavetable*, kUserTables>;
    void acquire(Pinned& out) noexcept;
    void release() noexcept;

private:
    std::array<RtShared<UserWavetable>, kUserTables> slots;
    double sampleRate = 48000.0;
};

} // namespace crucible
