// SPDX-License-Identifier: MIT
#pragma once

// Multisample zones for the sample oscillator (design: docs/MULTISAMPLE_DESIGN.md). A zone is a sample with a key
// range, a velocity range, a root note, a gain and optional loop points; a note plays the zones whose ranges contain its
// key and velocity. The zones of a slot are an immutable ZoneSet published to the audio thread by ZoneBank.

#include <crucible/AudioFile.h>
#include <crucible/RtShared.h>
#include <crucible/Sample.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace crucible
{

constexpr int kMaxZones = 64;
constexpr int kMaxSamplesPerSlot = 64;               // sample ids 0 (the slot's own sample) .. 63
constexpr int kZoneMaxFrames = kSampleMaxFrames * 4; // frames of all extra samples of one slot
constexpr int kMaxZoneLayers = 4;
constexpr int kMaxZoneSlots = 8;

enum class ZoneRoundRobin
{
    Off = 0,
    Cycle = 1,
    Random = 2
};

struct Zone
{
    int sampleId = 0; // 0 = the slot's sample (SampleBank); n >= 1 = ZoneBank sample n
    std::uint8_t keyLo = 0, keyHi = 127;
    std::uint8_t velLo = 0, velHi = 127;
    float root = 60.0f; // MIDI note units; the pitch at which the sample plays unchanged
    float gainDb = 0.0f;
    float tuneCents = 0.0f;
    float loopStart = 0.0f; // 0..1 along the played region (used when loopMode >= 0)
    float loopEnd = 1.0f;
    int loopMode = -1;      // -1 = the slot's loop parameters; otherwise 0 off, 1 forward, 2 ping-pong
    std::uint8_t group = 0; // overlapping zones of one group are alternatives (round robin); groups layer

    bool operator==(const Zone& o) const;
};

// Immutable once published. The extra samples (ids 1..) are shared with the control side's staging copy, so
// republishing zones does not copy audio.
struct ZoneSet
{
    ZoneSet() = default;
    ZoneSet(const ZoneSet& o) : zones(o.zones), samples(o.samples), cycle(o.cycle.load(std::memory_order_relaxed)) {}
    std::vector<Zone> zones;
    std::vector<std::shared_ptr<const SampleData>> samples; // sample id n is samples[n - 1]
    // Round robin position (audio thread only); it restarts with every published set.
    mutable std::atomic<std::uint32_t> cycle{0};

    int count() const { return static_cast<int>(zones.size()); }
    // The audio of a zone: id 0 is slotSample, others come from `samples`; null when missing.
    const SampleData* sampleFor(const Zone& z, const SampleData* slotSample) const;
};

struct ZoneLayer
{
    int zone = -1;
    float weight = 0.0f; // equal power crossfade weight, 1 away from the zone edges
};

struct ZoneSelect
{
    float xfadeKey = 0.0f; // semitones, 0..12
    float xfadeVel = 0.0f; // 0..0.5 of the velocity range
    ZoneRoundRobin roundRobin = ZoneRoundRobin::Off;
    std::uint32_t seed = 1; // per note, for Random
};

// The zones that sound for a key (0..127) and velocity (0..127): at most kMaxZoneLayers, lowest zone index first.
// Allocation free. Returns the number of layers.
int selectZones(const ZoneSet& set, int key, int velocity, const ZoneSelect& options,
                std::array<ZoneLayer, kMaxZoneLayers>& out);

// Edge weight of x inside [lo, hi] with a crossfade of `width` centred on lo - 0.5 and hi + 0.5. The sum of the squares of
// two zones that meet at an edge is 1. Edges at 0 and 127 stay open.
float zoneEdgeWeight(float x, int lo, int hi, float width);

// Names a zone sample after its file: `Name_C3_v80.wav` = root C3 (C3 is MIDI 60), velocity 80. `_60` is a plain root note.
struct ZoneFileName
{
    float root = -1.0f; // -1 = none given
    int velocity = -1;  // -1 = none given
};
ZoneFileName parseZoneFileName(const std::string& fileName);

// Gives zones sorted by root note the key ranges that split the distance between neighbouring roots (zones of one
// velocity layer; the velocity ranges split the same way). Zones with equal root and velocity keep their order.
// Rewrites keyLo / keyHi / velLo / velHi of `zones`; `velocity` is the velocity each zone was named with (-1 = all).
void assignAutomaticRanges(std::vector<Zone>& zones, const std::vector<int>& velocity);

// Samples and zones of the oscillator slots (the sample of id 0 stays in SampleBank). Control thread except
// acquire / release.
class ZoneBank
{
public:
    // Adds a sample to a slot and returns its id (1..kMaxSamplesPerSlot - 1), or 0 with `error` set.
    int addSample(int slot, const AudioData& audio, const std::string& name, std::string& error);
    // Replaces the zones of a slot; fields are clamped (see sanitiseZone). False for a bad slot. No zones = the slot
    // plays as before.
    bool setZones(int slot, std::vector<Zone> zones);
    std::vector<Zone> zones(int slot) const;          // the published zones; any thread except the audio thread
    int sampleCount(int slot) const;                  // extra samples (ids 1..n)
    const SampleData* sample(int slot, int id) const; // id >= 1
    void clear(int slot);
    void clearAll();
    bool hasZones(int slot) const;

    // Control thread: builds zones from files. A folder of WAVs named Name_C3_v80.wav (key ranges from the roots, or from
    // the detected pitch), or a SFZ-lite text (key, lokey, hikey, lovel, hivel, pitch_keycenter, loop_start,
    // loop_end, sample=file; <region> and <group> headers). Replaces the extra samples and zones of the slot. Returns the
    // number of zones.
    int importFolder(int slot, const std::string& directory, std::string& error);
    int importSfz(int slot, const std::string& sfzPath, std::string& error);

    // Patches: extra samples (original audio) and zones as text.
    struct Record
    {
        int slot = 0;
        int index = 1;
        std::string name;
        double rate = 48000.0;
        int channels = 1;
        std::vector<float> frames;
    };
    std::vector<Record> serialiseSamples() const;
    std::string serialiseZones(int slot) const; // empty when the slot has none
    // Clears every slot, then loads the records and zone texts.
    void deserialise(const std::vector<Record>& samples, const std::vector<std::pair<int, std::string>>& zoneTexts);
    static std::string zonesToText(const std::vector<Zone>& zones);
    static bool zonesFromText(const std::string& text, std::vector<Zone>& out);

    using Pinned = std::array<const ZoneSet*, kMaxZoneSlots>;
    void acquire(Pinned& out) noexcept;
    void release() noexcept;
    void collectGarbage();

private:
    struct Stage
    {
        std::vector<std::shared_ptr<const SampleData>> samples;
        std::vector<Zone> zones;
        int frames = 0;
    };
    void publish(int slot);
    static Zone sanitiseZone(Zone z, int sampleCount);
    std::array<Stage, kMaxZoneSlots> stages;
    std::array<RtShared<ZoneSet>, kMaxZoneSlots> slots;
};

} // namespace crucible
