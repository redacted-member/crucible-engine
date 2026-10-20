// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace crucible
{

// Procedurally generated, mip-mapped wavetables. A table is 64 frames; every frame is stored at several band-limited
// levels. A level keeps only the first `harmonics` partials and is stored with enough samples that the 6-point
// interpolation of the oscillators stays below -120 dB of error (about 6 to 128 samples per cycle of its highest
// partial, more for the highest levels). Level 0 has the most partials. The oscillator picks the level with the
// most partials that keeps every partial below (sampleRate - 20 kHz), so whatever folds over Nyquist lands above
// 20 kHz; above 48 kHz the limit is also capped at 30 kHz (Voice::bandLimit). Neighbouring levels differ by less than
// the 1.4 ratio between 20 kHz and that limit, so the audible bandwidth never drops below 20 kHz at any pitch.
//
// Memory: a table takes about 9 MB (all 14 about 130 MB), so tables are generated the first time something asks for
// them (ensure(), or implicitly frame()) and then stay resident. Engine calls ensure() on the control thread when an
// oscillator is pointed at a table; the audio thread never generates one, and an oscillator whose table is not resident
// yet is silent until service() (or a setParam call) has generated it. EngineConfig::preloadWavetables generates all
// tables up front instead.
class WavetableBank
{
public:
    static constexpr int tableSize = 2048; // full-band reference cycle (analysis, serialisation, base frames)
    static constexpr int numFrames = 64;
    static constexpr int numTables = 14;

    struct Level
    {
        int harmonics;
        int size;
    };
    static constexpr int numMips = 27;
    static constexpr int maxMipSize = 4096;
    // The top four levels serve notes below about 65 Hz and trade interpolation accuracy for memory.
    static constexpr std::array<Level, numMips> levels = {
        {{1023, 2048}, {741, 2048}, {537, 4096}, {389, 4096}, {310, 4096}, {225, 4096}, {165, 4096}, {120, 2048}, {88, 2048},
         {64, 1024},   {47, 1024},  {34, 1024},  {25, 512},   {18, 512},   {13, 256},   {12, 256},   {11, 256},   {10, 256},
         {9, 256},     {8, 256},    {7, 128},    {6, 128},    {5, 128},    {4, 128},    {3, 128},    {2, 128},    {1, 128}}};

    static const WavetableBank& instance();
    static const std::vector<std::string>& tableNames();

    static constexpr int mipSize(int mip) { return levels[static_cast<size_t>(mip)].size; }
    static constexpr int mipHarmonics(int mip) { return levels[static_cast<size_t>(mip)].harmonics; }

    // Samples before index 0 and after index mipSize are valid (two before, three after) for 6-point interpolation.
    static constexpr int guardBefore = 2;
    static constexpr int guardAfter = 3;

    // Returns a pointer to sample 0 of mipSize(mip) samples; the cycle is repeated in the guard samples. Generates
    // the table first if it is not resident.
    const float* frame(int table, int mip, int frameIndex) const
    {
        return tableData(table) + mipOffset(mip) +
               static_cast<size_t>(frameIndex) * static_cast<size_t>(mipSize(mip) + guardBefore + guardAfter) + guardBefore;
    }

    // Generates a table if it is not resident yet (thread safe, idempotent). Out-of-range indices (user tables) are
    // ignored.
    void ensure(int table) const;
    void ensureAll() const;
    bool resident(int table) const
    {
        return table >= 0 && table < numTables &&
               slots[static_cast<size_t>(table)].data.load(std::memory_order_acquire) != nullptr;
    }
    // Bytes of one table (all levels and frames, guard samples included) and of the tables resident now.
    static size_t bytesPerTable() { return perTable() * sizeof(float); }
    size_t residentBytes() const;

    // The limit (see mipForIncrement) of a plain oscillator at the given output rate: partials up to (rate - 20 kHz),
    // above 48 kHz also kept to 30 kHz (never less than 0.2 cycles per sample). Voice::bandLimit(1) is this.
    static float outputBandLimit(double sampleRate)
    {
        const float rate = static_cast<float>(sampleRate);
        return std::min(std::clamp(1.0f - 20000.0f / rate, 0.5f, 0.95f), std::max(30000.0f / rate, 0.2f));
    }

    // The level for an oscillator running at phaseIncrement cycles per sample: the one with the most partials such
    // that partials * phaseIncrement <= limit. limit is 1 - 20 kHz / rate (cycles per sample), at least 0.5.
    static int mipForIncrement(float phaseIncrement, float limit = 0.5f)
    {
        // Levels are sorted by falling partial count: binary search for the first one that fits.
        int lo = 0, hi = numMips - 1;
        while (lo < hi)
        {
            const int mid = (lo + hi) / 2;
            if (static_cast<float>(levels[static_cast<size_t>(mid)].harmonics) * phaseIncrement <= limit)
                hi = mid;
            else
                lo = mid + 1;
        }
        return lo;
    }

private:
    WavetableBank() = default;
    // Start of each level within a table (in samples); the last entry is the size of a table.
    static constexpr std::array<size_t, numMips + 1> offsets = []
    {
        std::array<size_t, numMips + 1> o{};
        for (int m = 0; m < numMips; ++m)
            o[static_cast<size_t>(m) + 1] =
                o[static_cast<size_t>(m)] +
                static_cast<size_t>(numFrames) * static_cast<size_t>(levels[static_cast<size_t>(m)].size + 5);
        return o;
    }();
    static_assert(guardBefore + guardAfter == 5, "level offsets assume five guard samples");
    static size_t mipOffset(int mip) { return offsets[static_cast<size_t>(mip)]; }
    static size_t perTable() { return offsets[numMips]; }
    const float* tableData(int table) const
    {
        const float* p = slots[static_cast<size_t>(table)].data.load(std::memory_order_acquire);
        if (p == nullptr)
        {
            ensure(table);
            p = slots[static_cast<size_t>(table)].data.load(std::memory_order_acquire);
        }
        return p;
    }
    static void generate(int table, float* out);

    struct Slot
    {
        std::atomic<const float*> data{nullptr};
        std::unique_ptr<float[]> storage;
    };
    mutable std::array<Slot, numTables> slots;
    mutable std::mutex generateMutex;
};

} // namespace crucible
