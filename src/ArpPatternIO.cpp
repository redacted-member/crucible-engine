// SPDX-License-Identifier: MIT
#include <crucible/ArpPatternIO.h>
#include <crucible/Patch.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <map>

#include "Utf8Path.h"

namespace crucible
{

json::Value arpPatternToJsonValue(const ArpPattern& p)
{
    using json::Value;
    Value doc = Value::object();
    doc.set("format", "crucible-arp-pattern");
    doc.set("version", 1);
    doc.set("length", p.length);
    doc.set("direction", p.direction);
    Value steps = Value::array();
    for (int i = 0; i < kArpMaxSteps; ++i)
    {
        const auto& s = p.steps[static_cast<size_t>(i)];
        Value o = Value::object();
        o.set("on", s.on);
        o.set("tie", s.tie);
        o.set("velocity", static_cast<int>(s.velocity));
        o.set("gate", static_cast<double>(s.gate));
        o.set("octave", static_cast<int>(s.octave));
        o.set("probability", static_cast<double>(s.probability));
        o.set("ratchet", static_cast<int>(s.ratchet));
        o.set("transpose", static_cast<int>(s.transpose));
        o.set("group", static_cast<int>(s.group));
        steps.push(o);
    }
    doc.set("steps", steps);
    return doc;
}

std::string arpPatternToJson(const ArpPattern& p)
{
    return arpPatternToJsonValue(p).dump(2) + "\n";
}

bool arpPatternFromJson(const std::string& text, ArpPattern& out, std::string& error)
{
    json::Value doc;
    if (!json::parse(text, doc, error))
        return false;
    const std::string format = doc.isObject() ? doc["format"].asString("") : std::string();
    if (format != "crucible-arp-pattern")
    {
        error = "not an arpeggiator pattern";
        return false;
    }
    if (doc["version"].asNumber(0.0) < 1 || doc["version"].asNumber(0.0) > 1)
    {
        error = "unsupported pattern version";
        return false;
    }
    ArpPattern p;
    p.length = std::clamp(static_cast<int>(doc["length"].asNumber(8.0)), 1, kArpMaxSteps);
    p.direction = std::clamp(static_cast<int>(doc["direction"].asNumber(0.0)), 0, 3);
    const auto& steps = doc["steps"].elements();
    for (size_t i = 0; i < steps.size() && i < static_cast<size_t>(kArpMaxSteps); ++i)
    {
        const auto& o = steps[i];
        auto& s = p.steps[i];
        s.on = o["on"].asBool(true);
        s.tie = o["tie"].asBool(false);
        s.velocity = static_cast<std::uint8_t>(std::clamp(static_cast<int>(o["velocity"].asNumber(0.0)), 0, 127));
        s.gate = std::clamp(static_cast<float>(o["gate"].asNumber(1.0)), 0.05f, 2.0f);
        s.octave = static_cast<std::int8_t>(std::clamp(static_cast<int>(o["octave"].asNumber(0.0)), -3, 3));
        s.probability = std::clamp(static_cast<float>(o["probability"].asNumber(1.0)), 0.0f, 1.0f);
        s.ratchet = static_cast<std::uint8_t>(std::clamp(static_cast<int>(o["ratchet"].asNumber(1.0)), 1, 4));
        s.transpose = static_cast<std::int8_t>(std::clamp(static_cast<int>(o["transpose"].asNumber(0.0)), -24, 24));
        s.group = static_cast<std::uint8_t>(std::clamp(static_cast<int>(o["group"].asNumber(0.0)), 0, 8));
    }
    out = p;
    return true;
}

bool saveArpPatternFile(const std::string& path, const ArpPattern& pattern)
{
    std::ofstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
        return false;
    f << arpPatternToJson(pattern);
    return static_cast<bool>(f);
}

bool loadArpPatternFile(const std::string& path, ArpPattern& out, std::string& error)
{
    std::ifstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
    {
        error = "cannot read " + path;
        return false;
    }
    return arpPatternFromJson(std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()), out, error);
}

namespace
{
void putVar(std::vector<std::uint8_t>& v, std::uint32_t x)
{
    std::uint8_t tmp[5];
    int n = 0;
    tmp[n++] = x & 0x7fU;
    while ((x >>= 7) != 0)
        tmp[n++] = static_cast<std::uint8_t>((x & 0x7fU) | 0x80U);
    while (n > 0)
        v.push_back(tmp[--n]);
}

void put32(std::vector<std::uint8_t>& v, std::uint32_t x)
{
    for (int s = 24; s >= 0; s -= 8)
        v.push_back(static_cast<std::uint8_t>((x >> s) & 0xffU));
}
} // namespace

std::vector<std::uint8_t> arpPatternToMidi(const ArpPattern& p, int baseNote, double bpm, int stepsPerBeat)
{
    constexpr int tpq = 480;
    stepsPerBeat = std::clamp(stepsPerBeat, 1, 16);
    const double stepTicks = static_cast<double>(tpq) / stepsPerBeat;
    struct Note
    {
        std::int64_t on, off;
        int note, velocity;
    };
    std::vector<Note> notes;
    const int len = std::clamp(p.length, 1, kArpMaxSteps);
    for (int i = 0; i < len; ++i)
    {
        const auto& s = p.steps[static_cast<size_t>(i)];
        if (!s.on || s.tie)
            continue;
        int tied = 0;
        while (i + 1 + tied < len && p.steps[static_cast<size_t>(i + 1 + tied)].tie)
            ++tied;
        const int ratchets = std::clamp(static_cast<int>(s.ratchet), 1, 4);
        const int pitch = std::clamp(baseNote + s.transpose + 12 * s.octave, 0, 127);
        const int vel = s.velocity > 0 ? s.velocity : 100;
        for (int r = 0; r < ratchets; ++r)
        {
            const double sub = stepTicks / ratchets;
            const std::int64_t on = std::llround(stepTicks * i + sub * r);
            const double extra = r == ratchets - 1 ? stepTicks * tied : 0.0;
            const std::int64_t off =
                on + std::max<std::int64_t>(1, std::llround(sub * std::clamp(s.gate, 0.05f, 2.0f) + extra));
            notes.push_back({on, off, pitch, vel});
        }
    }
    struct Ev
    {
        std::int64_t tick;
        int order;
        std::uint8_t status, d1, d2;
    };
    std::vector<Ev> evs;
    for (const auto& n : notes)
    {
        evs.push_back({n.on, 1, 0x90, static_cast<std::uint8_t>(n.note), static_cast<std::uint8_t>(n.velocity)});
        evs.push_back({n.off, 0, 0x80, static_cast<std::uint8_t>(n.note), 0});
    }
    std::stable_sort(evs.begin(), evs.end(),
                     [](const Ev& a, const Ev& b) { return a.tick != b.tick ? a.tick < b.tick : a.order < b.order; });

    std::vector<std::uint8_t> track;
    const auto micros = static_cast<std::uint32_t>(std::llround(60000000.0 / std::clamp(bpm, 20.0, 400.0)));
    putVar(track, 0);
    track.insert(track.end(), {0xff, 0x51, 0x03, static_cast<std::uint8_t>(micros >> 16),
                               static_cast<std::uint8_t>(micros >> 8), static_cast<std::uint8_t>(micros)});
    std::int64_t last = 0;
    for (const auto& e : evs)
    {
        putVar(track, static_cast<std::uint32_t>(e.tick - last));
        last = e.tick;
        track.insert(track.end(), {e.status, e.d1, e.d2});
    }
    // The clip ends at the end of the pattern so that a re-import sees the same length.
    putVar(track, static_cast<std::uint32_t>(
                      std::max<std::int64_t>(0, static_cast<std::int64_t>(std::llround(stepTicks * len)) - last)));
    track.insert(track.end(), {0xff, 0x2f, 0x00});

    std::vector<std::uint8_t> file = {'M', 'T', 'h', 'd'};
    put32(file, 6);
    file.insert(file.end(), {0, 0, 0, 1, static_cast<std::uint8_t>(tpq >> 8), static_cast<std::uint8_t>(tpq & 0xff)});
    file.insert(file.end(), {'M', 'T', 'r', 'k'});
    put32(file, static_cast<std::uint32_t>(track.size()));
    file.insert(file.end(), track.begin(), track.end());
    return file;
}

bool arpPatternFromMidi(const std::vector<TimedMidi>& events, double bpm, int stepsPerBeat, int baseNote, ArpPattern& out,
                        std::string& error)
{
    stepsPerBeat = std::clamp(stepsPerBeat, 1, 16);
    const double stepSeconds = 60.0 / std::clamp(bpm, 20.0, 400.0) / stepsPerBeat;
    struct Note
    {
        double start, end;
        int note, velocity;
    };
    std::vector<Note> notes;
    std::map<int, size_t> open;
    double endTime = 0.0;
    for (const auto& e : events)
    {
        const int type = e.status & 0xf0;
        const int n = e.data1 & 0x7f;
        endTime = std::max(endTime, e.seconds);
        if (type == 0x90 && e.data2 > 0)
        {
            open[n] = notes.size();
            notes.push_back({e.seconds, e.seconds, n, e.data2});
        }
        else if (type == 0x80 || type == 0x90)
        {
            const auto it = open.find(n);
            if (it != open.end())
            {
                notes[it->second].end = e.seconds;
                open.erase(it);
            }
        }
    }
    for (const auto& [n, i] : open)
        notes[i].end = std::max(notes[i].start + stepSeconds, endTime);
    if (notes.empty())
    {
        error = "no notes in the clip";
        return false;
    }
    int lowest = 127;
    for (const auto& n : notes)
        lowest = std::min(lowest, n.note);
    const int base = baseNote >= 0 ? baseNote : lowest;

    ArpPattern p;
    for (auto& s : p.steps)
        s.on = false;
    // Group the note-ons by step.
    std::map<int, std::vector<Note>> byStep;
    int lastStep = 0;
    for (const auto& n : notes)
    {
        const int step = static_cast<int>(std::floor(n.start / stepSeconds + 0.1));
        if (step < 0 || step >= kArpMaxSteps)
            continue;
        byStep[step].push_back(n);
        lastStep = std::max(lastStep, step);
    }
    int length = lastStep + 1;
    // The clip's end (last event) tells whether the pattern has trailing rests.
    length = std::max(length, static_cast<int>(std::floor(endTime / stepSeconds + 0.5)));
    p.length = std::clamp(length, 1, kArpMaxSteps);
    for (const auto& [step, group] : byStep)
    {
        if (step >= p.length)
            continue;
        auto sorted = group;
        std::stable_sort(sorted.begin(), sorted.end(), [](const Note& a, const Note& b) { return a.start < b.start; });
        // Notes at (almost) the same time are a chord: keep the lowest. Notes at different times are ratchets.
        std::vector<Note> retriggers;
        for (const auto& n : sorted)
        {
            if (!retriggers.empty() && std::abs(n.start - retriggers.back().start) < stepSeconds * 0.05)
            {
                if (n.note < retriggers.back().note)
                    retriggers.back() = n;
            }
            else
                retriggers.push_back(n);
        }
        auto& s = p.steps[static_cast<size_t>(step)];
        s.on = true;
        const int ratchets = std::clamp(static_cast<int>(retriggers.size()), 1, 4);
        s.ratchet = static_cast<std::uint8_t>(ratchets);
        const Note& first = retriggers.front();
        s.transpose = static_cast<std::int8_t>(std::clamp(first.note - base, -24, 24));
        s.velocity = static_cast<std::uint8_t>(first.velocity);
        const Note& tail = retriggers[static_cast<size_t>(ratchets - 1)];
        const double sub = stepSeconds / ratchets;
        const double duration = tail.end - tail.start;
        const int whole = static_cast<int>(std::floor(duration / stepSeconds + 1.0e-6));
        if (whole >= 2)
        {
            s.gate = 1.0f;
            // The step's note lasts through the next steps: those become ties (when they are free).
            for (int t = 1; t < whole && step + t < p.length; ++t)
                if (byStep.find(step + t) == byStep.end())
                    p.steps[static_cast<size_t>(step + t)].tie = true;
        }
        else
            s.gate = std::clamp(static_cast<float>(duration / (ratchets == 1 ? stepSeconds : sub)), 0.05f, 2.0f);
        if (s.gate > 1.0f && ratchets > 1)
            s.gate = 1.0f;
    }
    // Rest steps stay off; a tie needs a note before it.
    for (int i = 0; i < p.length; ++i)
        if (p.steps[static_cast<size_t>(i)].tie)
            p.steps[static_cast<size_t>(i)].on = true;
    out = p;
    return true;
}

bool arpPatternFromMidiFile(const std::string& path, int stepsPerBeat, int baseNote, ArpPattern& out, std::string& error)
{
    std::vector<TimedMidi> events;
    MidiFileInfo info;
    if (!readMidiFile(path, events, info, error))
        return false;
    return arpPatternFromMidi(events, info.firstTempoBpm, stepsPerBeat, baseNote, out, error);
}

} // namespace crucible
