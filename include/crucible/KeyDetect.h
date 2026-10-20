// SPDX-License-Identifier: MIT
#pragma once

// Key detection ("learn key"): listens to notes and proposes a root and scale with a confidence value.
//
// The detector keeps a weighted pitch-class histogram (note-on adds its velocity, held notes keep adding while they
// sound when tick() is called, so long notes count more) and correlates it with the Krumhansl-Kessler major and
// minor profiles in all 12 rotations (Pearson correlation). The best of the 24 keys is proposed; the confidence is
// 0 for an empty or flat histogram and grows with the correlation margin over the second-best key that is not the
// relative major / minor (those are musically the same notes) and with the amount of evidence.
//
// Threads: noteOn / noteOff / tick come from the audio (or MIDI) thread and only touch atomics; propose() and
// reset() are for the control thread.

#include <array>
#include <atomic>
#include <string>
#include <vector>

namespace crucible
{

struct MidiFileInfo;
struct TimedMidi;

struct KeyProposal
{
    int root = 0; // pitch class 0..11
    bool minor = false;
    int scale = 1;            // key scale index (Major = 1, Natural Minor = 2), see KeyLock.h
    float correlation = 0.0f; // -1..1 of the best key
    float confidence = 0.0f;  // 0..1
    float evidence = 0.0f;    // total weight collected
    std::string name;         // "A minor"
};

class KeyDetector
{
public:
    void noteOn(int note, float velocity01) noexcept;
    void noteOff(int note) noexcept;
    // Adds weight for the notes held now; call at a steady rate (for example once per block) with the elapsed seconds.
    void tick(float seconds) noexcept;
    void reset() noexcept;

    KeyProposal propose() const;
    // All 24 candidates, best first.
    std::vector<KeyProposal> candidates() const;
    // Feeds a parsed MIDI file (note-ons weighted by velocity and duration). Control thread; adds to the histogram.
    void addMidiFile(const std::vector<TimedMidi>& events);
    // Pure analysis of a histogram, for tests and offline use.
    static std::vector<KeyProposal> analyse(const std::array<float, 12>& histogram);

private:
    std::array<std::atomic<float>, 12> histogram{};
    std::array<std::atomic<float>, 128> held{};
};

} // namespace crucible
