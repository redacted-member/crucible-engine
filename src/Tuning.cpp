// SPDX-License-Identifier: MIT
#include <crucible/Tuning.h>

#include <crucible/Json.h>
#include <crucible/KeyLock.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>

#include "Utf8Path.h"

namespace crucible
{
namespace
{
std::vector<std::string> contentLines(const std::string& text)
{
    std::vector<std::string> out;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        out.push_back(line);
    }
    return out;
}

std::string trim(const std::string& s)
{
    const auto a = s.find_first_not_of(" \t");
    if (a == std::string::npos)
        return {};
    return s.substr(a, s.find_last_not_of(" \t") - a + 1);
}

// Parses one Scala pitch: cents when it has a '.', otherwise a ratio. Text after the number is ignored.
bool parsePitch(const std::string& raw, double& cents)
{
    const std::string s = trim(raw);
    if (s.empty())
        return false;
    char* end = nullptr;
    if (s.find('.') != std::string::npos)
    {
        cents = std::strtod(s.c_str(), &end);
        return end != s.c_str() && std::isfinite(cents);
    }
    const double num = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || num <= 0.0)
        return false;
    double den = 1.0;
    if (*end == '/')
    {
        char* end2 = nullptr;
        den = std::strtod(end + 1, &end2);
        if (end2 == end + 1 || den <= 0.0)
            return false;
    }
    cents = 1200.0 * std::log2(num / den);
    return std::isfinite(cents);
}

int floorDiv(int a, int b)
{
    int q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0)))
        --q;
    return q;
}

bool readFile(const std::string& path, std::string& out)
{
    std::ifstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
        return false;
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}
} // namespace

bool Tuning::loadScl(const std::string& text, std::string& error)
{
    std::vector<std::string> lines;
    for (const auto& l : contentLines(text))
        if (l.empty() || l[0] != '!')
            lines.push_back(l);
    // The description line may be empty; the count line is the first non-empty line after it.
    if (lines.size() < 2)
    {
        error = "scl: missing header";
        return false;
    }
    const std::string name = trim(lines[0]);
    size_t at = 1;
    while (at < lines.size() && trim(lines[at]).empty())
        ++at;
    if (at >= lines.size())
    {
        error = "scl: missing note count";
        return false;
    }
    char* end = nullptr;
    const long count = std::strtol(trim(lines[at]).c_str(), &end, 10);
    if (end == trim(lines[at]).c_str() || count < 0 || count > 1024)
    {
        error = "scl: bad note count";
        return false;
    }
    std::vector<double> parsed = {0.0};
    for (size_t i = at + 1; i < lines.size() && static_cast<long>(parsed.size()) <= count; ++i)
    {
        if (trim(lines[i]).empty())
            continue;
        double c = 0.0;
        if (!parsePitch(lines[i], c))
        {
            error = "scl: bad pitch '" + trim(lines[i]) + "'";
            return false;
        }
        parsed.push_back(c);
    }
    if (static_cast<long>(parsed.size()) != count + 1)
    {
        error = "scl: expected " + std::to_string(count) + " pitches";
        return false;
    }
    if (count == 0)
    {
        error = "scl: empty scale";
        return false;
    }
    if (parsed.back() <= 0.0)
    {
        error = "scl: the period must be above 1/1";
        return false;
    }
    description = name.empty() ? "Untitled scale" : name;
    degrees = std::move(parsed);
    formalOctaveDegree = static_cast<int>(count);
    sclText = text;
    if (mapSize > 0 && !keyMap.empty())
        for (int& d : keyMap)
            if (d > count)
                d = -1;
    return true;
}

bool Tuning::loadKbm(const std::string& text, std::string& error)
{
    std::vector<std::string> values;
    for (const auto& l : contentLines(text))
    {
        const std::string t = trim(l);
        if (t.empty() || t[0] == '!')
            continue;
        values.push_back(t.substr(0, t.find_first_of(" \t")));
    }
    if (values.size() < 7)
    {
        error = "kbm: missing header";
        return false;
    }
    const auto num = [&](size_t i) { return std::strtod(values[i].c_str(), nullptr); };
    const int size = static_cast<int>(num(0));
    const int first = static_cast<int>(num(1));
    const int last = static_cast<int>(num(2));
    const int middle = static_cast<int>(num(3));
    const int refNote = static_cast<int>(num(4));
    const double refHz = num(5);
    const int formal = static_cast<int>(num(6));
    if (size < 0 || size > 1024 || first < 0 || last > 127 || first > last || middle < 0 || middle > 127 || refNote < 0 ||
        refNote > 127 || !(refHz > 0.0) || formal < 0)
    {
        error = "kbm: value out of range";
        return false;
    }
    std::vector<int> map;
    for (int i = 0; i < size; ++i)
    {
        const size_t at = 7 + static_cast<size_t>(i);
        if (at >= values.size())
        {
            error = "kbm: expected " + std::to_string(size) + " map entries";
            return false;
        }
        const std::string& v = values[at];
        map.push_back(v == "x" || v == "X" ? -1 : static_cast<int>(std::strtol(v.c_str(), nullptr, 10)));
    }
    mapSize = size;
    firstNote = first;
    lastNote = last;
    middleNote = middle;
    referenceNote = refNote;
    referenceHz = refHz;
    keyMap = std::move(map);
    if (formal > 0)
        formalOctaveDegree = std::min(formal, scaleSize());
    kbmText = text;
    return true;
}

bool Tuning::loadSclFile(const std::string& path, std::string& error)
{
    std::string text;
    if (!readFile(path, text))
    {
        error = "cannot read " + path;
        return false;
    }
    return loadScl(text, error);
}

bool Tuning::loadKbmFile(const std::string& path, std::string& error)
{
    std::string text;
    if (!readFile(path, text))
    {
        error = "cannot read " + path;
        return false;
    }
    return loadKbm(text, error);
}

void Tuning::resetScale()
{
    description = "12-tone equal temperament";
    degrees.clear();
    for (int i = 0; i <= 12; ++i)
        degrees.push_back(100.0 * i);
    formalOctaveDegree = 12;
    sclText.clear();
}

void Tuning::resetKeyboardMap()
{
    if (degrees.empty())
        resetScale();
    mapSize = 0;
    firstNote = 0;
    lastNote = 127;
    middleNote = 60;
    referenceNote = 69;
    referenceHz = 440.0;
    keyMap.clear();
    formalOctaveDegree = scaleSize();
    kbmText.clear();
}

void Tuning::setNoteOffsetCents(int note, double cents)
{
    if (note >= 0 && note < 128)
        offsets[static_cast<size_t>(note)] = std::isfinite(cents) ? std::clamp(cents, -1200.0, 1200.0) : 0.0;
}

double Tuning::noteOffsetCents(int note) const
{
    return note >= 0 && note < 128 ? offsets[static_cast<size_t>(note)] : 0.0;
}

void Tuning::clearNoteOffsets()
{
    offsets.fill(0.0);
}

double Tuning::centsOf(int note, bool& ok) const
{
    ok = true;
    const int n = scaleSize();
    const double period = degrees[static_cast<size_t>(std::clamp(formalOctaveDegree, 1, n))];
    const int k = note - middleNote;
    if (mapSize == 0)
    {
        const int oct = floorDiv(k, n);
        const int deg = k - oct * n;
        return oct * degrees[static_cast<size_t>(n)] + degrees[static_cast<size_t>(deg)];
    }
    const int oct = floorDiv(k, mapSize);
    const int pos = k - oct * mapSize;
    const int deg = keyMap[static_cast<size_t>(pos)];
    if (deg < 0 || deg > n)
    {
        ok = false;
        return 0.0;
    }
    return oct * period + degrees[static_cast<size_t>(deg)];
}

bool Tuning::isMapped(int note) const
{
    if (note < 0 || note > 127)
        return false;
    if (note < firstNote || note > lastNote)
        return false;
    bool ok = true;
    centsOf(note, ok);
    return ok;
}

double Tuning::frequency(int note) const
{
    note = std::clamp(note, 0, 127);
    bool ok = true;
    const double c = centsOf(note, ok);
    if (!isMapped(note) || !ok)
        return equalFrequency(note);
    bool refOk = true;
    const double cr = centsOf(referenceNote, refOk);
    const double ref = refOk ? cr : 0.0;
    return referenceHz * std::pow(2.0, (c - ref + offsets[static_cast<size_t>(note)]) / 1200.0);
}

bool Tuning::isEqualTemperament() const
{
    for (int n = 0; n < 128; ++n)
        if (std::abs(frequency(n) / equalFrequency(n) - 1.0) > 1.0e-9 || !isMapped(n))
            return false;
    return true;
}

TuningTable Tuning::table() const
{
    TuningTable t;
    t.identity = true;
    for (int n = 0; n < 128; ++n)
    {
        const auto i = static_cast<size_t>(n);
        t.mapped[i] = isMapped(n) ? 1 : 0;
        t.semitones[i] = static_cast<float>(12.0 * std::log2(frequency(n) / equalFrequency(n)));
        if (t.mapped[i] == 0 || std::abs(t.semitones[i]) > 1.0e-6f)
            t.identity = false;
    }
    return t;
}

std::string Tuning::serialise() const
{
    json::Value o = json::Value::object();
    o.set("scl", sclText);
    o.set("kbm", kbmText);
    json::Value off = json::Value::object();
    for (int n = 0; n < 128; ++n)
        if (offsets[static_cast<size_t>(n)] != 0.0)
            off.set(std::to_string(n), offsets[static_cast<size_t>(n)]);
    o.set("offsets", off);
    return o.dump(0);
}

bool Tuning::deserialise(const std::string& text, Tuning& out)
{
    Tuning t;
    if (trim(text).empty())
    {
        out = t;
        return true;
    }
    json::Value doc;
    std::string error;
    if (!json::parse(text, doc, error) || !doc.isObject())
        return false;
    const std::string scl = doc["scl"].asString("");
    const std::string kbm = doc["kbm"].asString("");
    if (!scl.empty() && !t.loadScl(scl, error))
        return false;
    if (!kbm.empty() && !t.loadKbm(kbm, error))
        return false;
    for (const auto& [k, v] : doc["offsets"].entries())
        t.setNoteOffsetCents(std::atoi(k.c_str()), v.asNumber(0.0));
    out = std::move(t);
    return true;
}

TuningStore::TuningStore()
{
    shared.publish(std::make_unique<TuningTable>());
}

void TuningStore::set(const Tuning& t)
{
    std::lock_guard<std::mutex> lock(mutex);
    tuning = t;
    publishLocked();
}

Tuning TuningStore::get() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return tuning;
}

void TuningStore::edit(const std::function<void(Tuning&)>& change)
{
    std::lock_guard<std::mutex> lock(mutex);
    change(tuning);
    publishLocked();
}

void TuningStore::publishLocked()
{
    scaleMaskBits.store(scaleMaskFromTuning(tuning), std::memory_order_relaxed);
    shared.publish(std::make_unique<TuningTable>(tuning.table()));
}

} // namespace crucible
