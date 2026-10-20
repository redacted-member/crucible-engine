// SPDX-License-Identifier: MIT
#include <crucible/Arpeggiator.h>

#include "TextFormat.h"

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
float hashRandom01(std::uint32_t seed, std::int64_t k)
{
    auto x = static_cast<std::uint64_t>(k) * 0x9E3779B97F4A7C15ULL ^ (static_cast<std::uint64_t>(seed) << 17U);
    x ^= x >> 33U;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33U;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33U;
    return static_cast<float>(x >> 40U) / 16777216.0f;
}
} // namespace

const std::vector<std::string>& arpModeNames()
{
    static const std::vector<std::string> names = {"Up",     "Down",      "Up-Down", "Down-Up",
                                                   "Random", "As Played", "Chord",   "Converge"};
    return names;
}

const std::vector<std::string>& arpRateNames()
{
    static const std::vector<std::string> names = {"1/1",  "1/2",  "1/2.", "1/2T",  "1/4",   "1/4.", "1/4T",  "1/8",
                                                   "1/8.", "1/8T", "1/16", "1/16.", "1/16T", "1/32", "1/32T", "Free"};
    return names;
}

double arpRateBeats(int rate)
{
    static constexpr double table[] = {4.0,  2.0,       3.0,  4.0 / 3.0, 1.0,       1.5,   2.0 / 3.0,  0.5,
                                       0.75, 1.0 / 3.0, 0.25, 0.375,     1.0 / 6.0, 0.125, 1.0 / 12.0, 0.0};
    return table[std::clamp(rate, 0, 15)];
}

// ---------------------------------------------------------------------------------------------------------------
std::string arpPatternSerialise(const ArpPattern& p)
{
    std::vector<std::string> parts;
    parts.push_back(std::to_string(p.length) + "," + std::to_string(p.direction));
    const ArpStep def;
    for (int i = 0; i < kArpMaxSteps; ++i)
    {
        const auto& s = p.steps[static_cast<size_t>(i)];
        if (s.on == def.on && s.tie == def.tie && s.velocity == def.velocity && s.gate == def.gate &&
            s.octave == def.octave && s.probability == def.probability && s.ratchet == def.ratchet &&
            s.transpose == def.transpose && s.group == def.group)
            continue;
        parts.push_back(std::to_string(i) + "," + std::to_string(s.on ? 1 : 0) + "," + std::to_string(s.tie ? 1 : 0) + "," +
                        std::to_string(static_cast<int>(s.velocity)) + "," + text::fixed(s.gate, 4) + "," +
                        std::to_string(static_cast<int>(s.octave)) + "," + text::fixed(s.probability, 4) + "," +
                        std::to_string(static_cast<int>(s.ratchet)) + "," + std::to_string(static_cast<int>(s.transpose)) +
                        (s.group != 0 ? "," + std::to_string(static_cast<int>(s.group)) : std::string{}));
    }
    return text::join(parts, ';');
}

bool arpPatternDeserialise(const std::string& textIn, ArpPattern& out)
{
    const auto parts = text::split(textIn, ';');
    if (parts.empty())
        return false;
    const auto head = text::split(parts[0], ',');
    if (head.size() < 2)
        return false;
    ArpPattern p;
    p.length = std::clamp(text::toInt(head[0]), 1, kArpMaxSteps);
    p.direction = std::clamp(text::toInt(head[1]), 0, 3);
    for (size_t i = 1; i < parts.size(); ++i)
    {
        const auto f = text::split(parts[i], ',');
        if (f.size() < 9)
            continue;
        const int idx = text::toInt(f[0]);
        if (idx < 0 || idx >= kArpMaxSteps)
            continue;
        auto& s = p.steps[static_cast<size_t>(idx)];
        s.on = text::toInt(f[1]) != 0;
        s.tie = text::toInt(f[2]) != 0;
        s.velocity = static_cast<std::uint8_t>(std::clamp(text::toInt(f[3]), 0, 127));
        s.gate = std::clamp(text::toFloat(f[4]), 0.05f, 2.0f);
        s.octave = static_cast<std::int8_t>(std::clamp(text::toInt(f[5]), -3, 3));
        s.probability = std::clamp(text::toFloat(f[6]), 0.0f, 1.0f);
        s.ratchet = static_cast<std::uint8_t>(std::clamp(text::toInt(f[7]), 1, 4));
        s.transpose = static_cast<std::int8_t>(std::clamp(text::toInt(f[8]), -24, 24));
        s.group =
            f.size() > 9 ? static_cast<std::uint8_t>(std::clamp(text::toInt(f[9]), 0, 8)) : static_cast<std::uint8_t>(0);
    }
    out = p;
    return true;
}

ArpPatternStore::ArpPatternStore()
{
    shared.publish(std::make_unique<ArpPattern>());
}

void ArpPatternStore::edit(const std::function<void(ArpPattern&)>& change)
{
    std::lock_guard<std::mutex> lock(mutex);
    auto copy = std::make_unique<ArpPattern>(*shared.latest());
    change(*copy);
    shared.publish(std::move(copy));
}

void ArpPatternStore::replace(const ArpPattern& p)
{
    std::lock_guard<std::mutex> lock(mutex);
    shared.publish(std::make_unique<ArpPattern>(p));
}

ArpPattern ArpPatternStore::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return *shared.latest();
}

// ---------------------------------------------------------------------------------------------------------------
void Arpeggiator::prepare(double rate)
{
    sampleRate = rate > 0.0 ? rate : 48000.0;
    reset();
}

void Arpeggiator::reset()
{
    now = 0;
    beat = 0.0;
    nextStep = 0;
    running = false;
    held = 0;
    arrival = 0;
    latchArmed = false;
    seqIndex = 0;
    lastNote = -1;
    pendingCount = 0;
    lastStep.store(0);
}

void Arpeggiator::schedule(const Pending& p)
{
    if (pendingCount < static_cast<int>(pending.size()))
        pending[static_cast<size_t>(pendingCount++)] = p;
}

void Arpeggiator::noteInput(const ArpInput& in, const ArpSettings& s)
{
    if (in.panic)
    {
        held = 0;
        latchArmed = false;
        pendingCount = 0;
        lastNote = -1;
        running = false;
        return;
    }
    const int note = std::clamp(in.note, 0, 127);
    int idx = -1;
    for (int i = 0; i < held; ++i)
        if (heldNotes[static_cast<size_t>(i)] == note)
            idx = i;
    if (in.on)
    {
        bool anyDown = false;
        for (int i = 0; i < held; ++i)
            anyDown = anyDown || physicallyDown[static_cast<size_t>(i)];
        if (s.latch && latchArmed && !anyDown)
        {
            held = 0; // a fresh chord replaces the latched one
            latchArmed = false;
        }
        if (idx < 0 && held < maxHeld)
        {
            idx = held++;
            heldNotes[static_cast<size_t>(idx)] = note;
        }
        if (idx >= 0)
        {
            heldVel[static_cast<size_t>(idx)] = std::clamp(in.velocity, 0.0f, 1.0f);
            heldOrder[static_cast<size_t>(idx)] = arrival++;
            physicallyDown[static_cast<size_t>(idx)] = true;
        }
        return;
    }
    if (idx < 0)
        return;
    if (s.latch)
    {
        physicallyDown[static_cast<size_t>(idx)] = false;
        bool anyDown = false;
        for (int i = 0; i < held; ++i)
            anyDown = anyDown || physicallyDown[static_cast<size_t>(i)];
        if (!anyDown)
            latchArmed = true;
        return;
    }
    for (int i = idx; i + 1 < held; ++i)
    {
        heldNotes[static_cast<size_t>(i)] = heldNotes[static_cast<size_t>(i + 1)];
        heldVel[static_cast<size_t>(i)] = heldVel[static_cast<size_t>(i + 1)];
        heldOrder[static_cast<size_t>(i)] = heldOrder[static_cast<size_t>(i + 1)];
        physicallyDown[static_cast<size_t>(i)] = physicallyDown[static_cast<size_t>(i + 1)];
    }
    --held;
}

int Arpeggiator::buildSequence(const ArpSettings& s, int* notes, float* vel) const
{
    // Base list: as played (arrival order) or sorted by pitch.
    std::array<int, maxHeld> idx{};
    for (int i = 0; i < held; ++i)
        idx[static_cast<size_t>(i)] = i;
    const auto mode = static_cast<ArpMode>(std::clamp(s.mode, 0, static_cast<int>(ArpMode::Count) - 1));
    if (mode == ArpMode::AsPlayed)
        std::sort(idx.begin(), idx.begin() + held,
                  [&](int a, int b) { return heldOrder[static_cast<size_t>(a)] < heldOrder[static_cast<size_t>(b)]; });
    else
        std::sort(idx.begin(), idx.begin() + held,
                  [&](int a, int b) { return heldNotes[static_cast<size_t>(a)] < heldNotes[static_cast<size_t>(b)]; });
    const int octaves = std::clamp(s.octaves, 1, 4);
    int n = 0;
    for (int o = 0; o < octaves; ++o)
        for (int i = 0; i < held; ++i)
        {
            notes[n] = heldNotes[static_cast<size_t>(idx[static_cast<size_t>(i)])] + 12 * o;
            vel[n] = heldVel[static_cast<size_t>(idx[static_cast<size_t>(i)])];
            ++n;
        }
    return n;
}

void Arpeggiator::triggerStep(std::int64_t k, std::int64_t at, double stepSamples, const ArpSettings& s,
                              const ArpPattern* grid)
{
    if (held == 0)
        return;
    std::array<int, maxHeld * 4> notes{};
    std::array<float, maxHeld * 4> vels{};
    const int count = buildSequence(s, notes.data(), vels.data());
    if (count == 0)
        return;

    // Grid step for a step counter.
    const bool gridOn = s.useGrid && grid != nullptr;
    const int len = gridOn ? std::clamp(grid->length, 1, kArpMaxSteps) : kArpMaxSteps;
    const auto gridIndexFor = [&](std::int64_t kk)
    {
        const auto local = static_cast<int>(kk % len);
        switch (gridOn ? grid->direction : 0)
        {
        case 1:
            return len - 1 - local;
        case 2:
        {
            if (len <= 1)
                return 0;
            const int m = static_cast<int>(kk % (2 * (len - 1)));
            return m < len ? m : 2 * (len - 1) - m;
        }
        case 3:
            return static_cast<int>(hashRandom01(seed ^ 0x51ed270bU, kk) * static_cast<float>(len)) % len;
        default:
            return local;
        }
    };
    const int gridIndex = gridIndexFor(k);
    const ArpStep* step = gridOn ? &grid->steps[static_cast<size_t>(gridIndex)] : nullptr;
    lastStep.store(gridIndex, std::memory_order_relaxed);

    // A tie step plays nothing: the note before it was scheduled to last through it (see below).
    if (step != nullptr)
    {
        if (step->tie || !step->on)
            return;
        if (step->group == 0)
        {
            if (hashRandom01(seed, k) >= step->probability)
                return;
        }
        else
        {
            // Alternatives: one draw per pass decides which member of the group fires.
            const std::int64_t pass = k / len;
            float total = 0.0f;
            for (int g = 0; g < len; ++g)
            {
                const auto& m = grid->steps[static_cast<size_t>(g)];
                if (m.group == step->group && m.on && !m.tie)
                    total += m.probability;
            }
            const float u =
                hashRandom01(seed ^ (0x9e3779b1U * static_cast<std::uint32_t>(step->group)), pass) * std::max(total, 1.0f);
            float acc = 0.0f;
            int winner = -1;
            for (int g = 0; g < len && winner < 0; ++g)
            {
                const auto& m = grid->steps[static_cast<size_t>(g)];
                if (m.group != step->group || !m.on || m.tie)
                    continue;
                acc += m.probability;
                if (u < acc)
                    winner = g;
            }
            if (winner != gridIndex)
                return;
        }
    }
    // Steps tied to this one extend its last note by whole steps.
    int tied = 0;
    if (gridOn)
        while (tied < len - 1 && grid->steps[static_cast<size_t>(gridIndexFor(k + 1 + tied))].tie)
            ++tied;

    const auto mode = static_cast<ArpMode>(std::clamp(s.mode, 0, static_cast<int>(ArpMode::Count) - 1));
    std::array<int, maxHeld * 4> chosen{};
    std::array<float, maxHeld * 4> chosenVel{};
    int chosenCount = 0;
    const auto pick = [&](int i)
    {
        chosen[static_cast<size_t>(chosenCount)] = notes[static_cast<size_t>(i)];
        chosenVel[static_cast<size_t>(chosenCount)] = vels[static_cast<size_t>(i)];
        ++chosenCount;
    };
    const int i = seqIndex;
    switch (mode)
    {
    case ArpMode::Up:
    case ArpMode::AsPlayed:
        pick(i % count);
        break;
    case ArpMode::Down:
        pick(count - 1 - (i % count));
        break;
    case ArpMode::UpDown:
    case ArpMode::DownUp:
    {
        int j = 0;
        if (count > 1)
        {
            const int period = 2 * (count - 1);
            const int m = i % period;
            j = m < count ? m : period - m;
        }
        pick(mode == ArpMode::UpDown ? j : count - 1 - j);
        break;
    }
    case ArpMode::Random:
        pick(static_cast<int>(hashRandom01(seed ^ 0x2c1b3c6dU, k) * static_cast<float>(count)) % count);
        break;
    case ArpMode::Chord:
    {
        // One octave of the chord per step, cycling through the octave range.
        const int perOctave = held;
        const int oct = perOctave > 0 ? (i % std::max(count / perOctave, 1)) : 0;
        for (int n = 0; n < perOctave; ++n)
            pick(oct * perOctave + n);
        break;
    }
    case ArpMode::Converge:
    {
        const int m = i % count;
        pick(m % 2 == 0 ? m / 2 : count - 1 - m / 2);
        break;
    }
    case ArpMode::Count:
        break;
    }
    ++seqIndex;

    // Velocity.
    const auto vmode = static_cast<ArpVelocity>(std::clamp(s.velocityMode, 0, 2));
    const float fixed = static_cast<float>(std::clamp(s.fixedVelocity, 1, 127)) / 127.0f;
    const bool accented = step != nullptr ? step->velocity >= 100 : (k % 4 == 0);
    const int ratchets = step != nullptr ? std::clamp(static_cast<int>(step->ratchet), 1, 4) : std::clamp(s.ratchet, 1, 4);
    const float gate = std::clamp(s.gate * (step != nullptr ? step->gate : 1.0f), 0.05f, 2.0f);
    const int shift = s.transpose + (step != nullptr ? step->transpose + 12 * step->octave : 0);
    const double sub = stepSamples / ratchets;
    for (int c = 0; c < chosenCount; ++c)
    {
        float v = chosenVel[static_cast<size_t>(c)];
        if (vmode == ArpVelocity::Fixed)
            v = fixed;
        else if (vmode == ArpVelocity::Accent)
            v = accented ? fixed : fixed * (1.0f - std::clamp(s.accent, 0.0f, 1.0f));
        if (step != nullptr && step->velocity > 0 && vmode != ArpVelocity::Accent)
            v = static_cast<float>(step->velocity) / 127.0f;
        const int note = std::clamp(chosen[static_cast<size_t>(c)] + shift, 0, 127);
        for (int r = 0; r < ratchets; ++r)
        {
            const auto onAt = at + std::llround(sub * r);
            const std::int64_t extra = r == ratchets - 1 ? std::llround(stepSamples * tied) : 0;
            const auto offAt = onAt + extra + std::max<std::int64_t>(1, std::llround(sub * gate));
            // An overlapping (gate > 1) earlier copy of this note ends where the new one starts.
            for (int q = 0; q < pendingCount; ++q)
            {
                auto& e = pending[static_cast<size_t>(q)];
                if (!e.on && e.note == note && e.at > onAt)
                    e.at = onAt;
            }
            schedule({onAt, true, note, std::max(v, 1.0f / 127.0f)});
            schedule({offAt, false, note, 0.0f});
        }
        lastNote = note;
    }
}

int Arpeggiator::flush(ArpOutput* out, int cap)
{
    int count = 0;
    for (int i = 0; i < pendingCount; ++i)
    {
        const auto& p = pending[static_cast<size_t>(i)];
        if (!p.on && count < cap)
            out[count++] = {0, false, p.note, 0.0f};
    }
    pendingCount = 0;
    running = false;
    return count;
}

int Arpeggiator::process(const ArpInput* inputs, int numInputs, const ArpSettings& s, const ArpPattern* grid,
                         const SeqHostTime& host, int numSamples, ArpOutput* out, int cap)
{
    int count = 0;
    const double stepBeatsSetting = arpRateBeats(s.rate);
    const bool free = stepBeatsSetting <= 0.0;
    const bool useHost = s.hostSync && host.hasBpm;
    const double bpm = std::clamp(useHost ? host.bpm : static_cast<double>(s.tempo), 20.0, 400.0);
    // In free mode "beats" are steps.
    const double stepBeats = free ? 1.0 : stepBeatsSetting;
    const double beatsPerSample =
        free ? std::clamp(static_cast<double>(s.freeRateHz), 0.05, 100.0) / sampleRate : bpm / 60.0 / sampleRate;
    const double stepSamples = stepBeats / beatsPerSample;
    const double swing = std::clamp(static_cast<double>(s.swingPercent), 0.0, 75.0) / 100.0 * stepBeats;
    const bool transport = useHost && host.hasPpq && host.playing && !free;

    const auto stepTime = [&](std::int64_t k) { return static_cast<double>(k) * stepBeats + ((k & 1) != 0 ? swing : 0.0); };

    int nextInput = 0;
    for (int i = 0; i < numSamples; ++i)
    {
        const double b = transport ? host.ppq + static_cast<double>(i) * beatsPerSample : beat;
        // Inputs at this sample.
        while (nextInput < numInputs && inputs[nextInput].position <= i)
        {
            const bool wasEmpty = held == 0;
            noteInput(inputs[nextInput], s);
            ++nextInput;
            if (wasEmpty && held > 0)
            {
                // First key: start the pattern now (internal clock) or on the next grid line (transport).
                seqIndex = 0;
                lastNote = -1;
                if (transport)
                    nextStep = static_cast<std::int64_t>(std::ceil(b / stepBeats - 1.0e-9));
                else
                {
                    beat = 0.0;
                    nextStep = 0;
                }
                running = true;
            }
        }
        const double bb = transport ? b : beat;
        if (running && held > 0 && bb + 1.0e-9 >= stepTime(nextStep))
        {
            triggerStep(nextStep, now, stepSamples, s, grid);
            ++nextStep;
        }
        if (held == 0)
            running = false;
        // Due events at this sample: note-offs first so a retriggered note is not cut.
        for (int pass = 0; pass < 2; ++pass)
            for (int p = 0; p < pendingCount;)
            {
                auto& e = pending[static_cast<size_t>(p)];
                if (e.at <= now && e.on == (pass == 1))
                {
                    if (count < cap)
                        out[count++] = {i, e.on, e.note, e.velocity};
                    pending[static_cast<size_t>(p)] = pending[static_cast<size_t>(--pendingCount)];
                }
                else
                    ++p;
            }
        if (!transport)
            beat += beatsPerSample;
        ++now;
    }
    if (transport)
        beat = host.ppq + numSamples * beatsPerSample;
    return count;
}

} // namespace crucible
