// SPDX-License-Identifier: MIT
#pragma once

// Microtuning: Scala scales (.scl), keyboard mappings (.kbm) and a per-note offset table.
//
// A Tuning turns every MIDI note into a frequency. The engine renders 12-tone equal temperament plus a pitch offset
// per note, so a Tuning is published to the audio thread as a TuningTable of semitone offsets from 12-TET (and a
// "note is mapped" flag: a .kbm can leave keys unmapped, those notes do not sound). The `tune_amount` parameter
// blends between 12-TET (0) and the tuning (1).
//
// Scala semantics: the scale lists degrees 1..N above the 1/1 (degree 0); a pitch with a '.' is cents, otherwise a
// ratio "a/b" or integer. The keyboard map assigns scale degrees to keys around a middle note and fixes the
// reference note / frequency; with the default map (no .kbm) the scale starts on note 60 and the reference is
// note 69 at 440 Hz. Per-note offsets (cents) are added on top of whatever the scale gives.

#include <crucible/RtShared.h>

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace crucible
{

struct TuningTable
{
    std::array<float, 128> semitones{}; // offset from 12-TET
    std::array<unsigned char, 128> mapped{};
    bool identity = true;
    TuningTable() { mapped.fill(1); }
};

class Tuning
{
public:
    Tuning() { resetKeyboardMap(); }

    bool loadScl(const std::string& text, std::string& error);
    bool loadKbm(const std::string& text, std::string& error);
    bool loadSclFile(const std::string& path, std::string& error);
    bool loadKbmFile(const std::string& path, std::string& error);
    void resetScale();       // back to 12-TET
    void resetKeyboardMap(); // middle note 60, reference note 69 = 440 Hz, linear mapping

    // Per-note correction in cents (clamped to +-1200); 0 removes it.
    void setNoteOffsetCents(int note, double cents);
    double noteOffsetCents(int note) const;
    void clearNoteOffsets();

    bool isEqualTemperament() const;
    bool isMapped(int note) const;
    // Frequency in Hz including the per-note offset; unmapped notes return their 12-TET frequency.
    double frequency(int note) const;
    TuningTable table() const;

    const std::string& scaleName() const { return description; }
    int scaleSize() const { return static_cast<int>(degrees.size()) - 1; } // degrees without the 1/1
    // Cents of degree 0..size (degree 0 = 0).
    const std::vector<double>& degreeCents() const { return degrees; }

    // JSON text holding the scale, the map and the offsets (patches store this string).
    std::string serialise() const;
    static bool deserialise(const std::string& text, Tuning& out);

    static double equalFrequency(int note) { return 440.0 * std::pow(2.0, (note - 69) / 12.0); }

private:
    double centsOf(int note, bool& ok) const;

    std::string description = "12-tone equal temperament";
    std::vector<double> degrees; // [0] = 0, then the scale pitches in cents
    int formalOctaveDegree = 12;
    int mapSize = 0; // 0 = linear
    int firstNote = 0, lastNote = 127, middleNote = 60, referenceNote = 69;
    double referenceHz = 440.0;
    std::vector<int> keyMap; // per map position: degree, or -1 for unmapped
    std::array<double, 128> offsets{};
    std::string sclText, kbmText;
};

// Control thread edits, audio thread reads (same scheme as ModRouteStore).
class TuningStore
{
public:
    TuningStore();
    void set(const Tuning& tuning);
    Tuning get() const;
    void edit(const std::function<void(Tuning&)>& change);
    void collectGarbage() { shared.collectGarbage(); }
    // Scale-lock mask derived from the loaded scale (KeyLock.h scaleMaskFromTuning); any thread.
    unsigned scaleMask() const noexcept { return scaleMaskBits.load(std::memory_order_relaxed); }
    const TuningTable* acquire() noexcept { return shared.acquire(); }
    void release() noexcept { shared.release(); }

private:
    mutable std::mutex mutex;
    Tuning tuning;
    RtShared<TuningTable> shared;
    std::atomic<unsigned> scaleMaskBits{0x0FFFU};
    void publishLocked();
};

} // namespace crucible
