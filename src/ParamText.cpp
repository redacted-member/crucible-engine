// SPDX-License-Identifier: MIT
#include <crucible/ParamText.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace crucible
{
namespace
{
struct FormatName
{
    TextFormat format;
    const char* name;
};
const FormatName kFormats[] = {
    {TextFormat::Auto, "auto"},       {TextFormat::Number, "number"},   {TextFormat::Hertz, "hz"},
    {TextFormat::Seconds, "s"},       {TextFormat::Milliseconds, "ms"}, {TextFormat::Decibels, "db"},
    {TextFormat::Percent, "percent"}, {TextFormat::Semitones, "st"},    {TextFormat::Cents, "cents"},
    {TextFormat::NoteName, "note"},   {TextFormat::Ratio, "ratio"},     {TextFormat::Bpm, "bpm"},
    {TextFormat::Degrees, "deg"},     {TextFormat::Octaves, "oct"},     {TextFormat::Bits, "bit"},
    {TextFormat::Bars, "bars"},       {TextFormat::Choice, "choice"},   {TextFormat::Toggle, "toggle"},
};
const char* const kHints[] = {"auto", "knob", "slider", "switch", "menu", "xy", "drawbar"};
const char* const kPolarities[] = {"auto", "unipolar", "bipolar"};

std::string lower(std::string s)
{
    for (auto& c : s)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string trim(const std::string& s)
{
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a])))
        ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1])))
        --b;
    return s.substr(a, b - a);
}

// Six significant digits, no exponent, trailing zeros trimmed.
std::string number(double v)
{
    if (v == 0.0 || !std::isfinite(v))
        return "0";
    const int mag = static_cast<int>(std::floor(std::log10(std::fabs(v))));
    const int decimals = std::clamp(5 - mag, 0, 12);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    std::string s = buf;
    if (s.find('.') != std::string::npos)
    {
        while (s.back() == '0')
            s.pop_back();
        if (s.back() == '.')
            s.pop_back();
    }
    return s == "-0" ? "0" : s;
}

std::string signedNumber(double v)
{
    const std::string n = number(v);
    return (v > 0.0 && n != "0" ? "+" : "") + n;
}

const char* const kNotes[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

std::string noteName(int n)
{
    const int pc = ((n % 12) + 12) % 12;
    const int oct = (n - pc) / 12 - 1;
    return std::string(kNotes[pc]) + std::to_string(oct);
}

bool parseNote(const std::string& t, int& out)
{
    if (t.empty() ||
        !(std::toupper(static_cast<unsigned char>(t[0])) >= 'A' && std::toupper(static_cast<unsigned char>(t[0])) <= 'G'))
        return false;
    static const int base[7] = {9, 11, 0, 2, 4, 5, 7}; // A B C D E F G
    int pc = base[std::toupper(static_cast<unsigned char>(t[0])) - 'A'];
    size_t i = 1;
    if (i < t.size() && (t[i] == '#' || t[i] == 'b'))
    {
        pc += t[i] == '#' ? 1 : -1;
        ++i;
    }
    char* end = nullptr;
    const long oct = std::strtol(t.c_str() + i, &end, 10);
    if (end == t.c_str() + i || *end != '\0')
        return false;
    out = static_cast<int>((oct + 1) * 12 + pc);
    return true;
}

// Splits "12.5 kHz" into the number and the lower-case suffix.
bool split(const std::string& text, double& value, std::string& suffix)
{
    const std::string t = trim(text);
    if (t.empty())
        return false;
    char* end = nullptr;
    value = std::strtod(t.c_str(), &end);
    if (end == t.c_str() || !std::isfinite(value))
        return false;
    suffix = lower(trim(std::string(end)));
    return true;
}

bool oneOf(const std::string& s, std::initializer_list<const char*> options)
{
    for (const char* o : options)
        if (s == o)
            return true;
    return false;
}
} // namespace

const char* textFormatName(TextFormat format)
{
    for (const auto& f : kFormats)
        if (f.format == format)
            return f.name;
    return "auto";
}

bool textFormatFromName(const std::string& name, TextFormat& out)
{
    for (const auto& f : kFormats)
        if (name == f.name)
        {
            out = f.format;
            return true;
        }
    return false;
}

const char* uiHintName(UiHint hint)
{
    return kHints[static_cast<int>(hint)];
}

bool uiHintFromName(const std::string& name, UiHint& out)
{
    for (int i = 0; i < 7; ++i)
        if (name == kHints[i])
        {
            out = static_cast<UiHint>(i);
            return true;
        }
    return false;
}

const char* polarityName(Polarity polarity)
{
    return kPolarities[static_cast<int>(polarity)];
}

bool polarityFromName(const std::string& name, Polarity& out)
{
    for (int i = 0; i < 3; ++i)
        if (name == kPolarities[i])
        {
            out = static_cast<Polarity>(i);
            return true;
        }
    return false;
}

namespace ParamText
{
TextFormat resolveFormat(const ParamDef& d)
{
    if (d.textFormat != TextFormat::Auto)
        return d.textFormat;
    if (d.type == ParamType::Choice)
        return TextFormat::Choice;
    if (d.type == ParamType::Bool)
        return TextFormat::Toggle;
    const std::string& u = d.unit;
    if (u == "Hz")
        return TextFormat::Hertz;
    if (u == "s")
        return TextFormat::Seconds;
    if (u == "ms")
        return TextFormat::Milliseconds;
    if (u == "dB")
        return TextFormat::Decibels;
    if (u == "%")
        return TextFormat::Percent;
    if (u == "st")
        return TextFormat::Semitones;
    if (u == "ct")
        return TextFormat::Cents;
    if (u == "note")
        return TextFormat::NoteName;
    if (u == ":1")
        return TextFormat::Ratio;
    if (u == "BPM")
        return TextFormat::Bpm;
    if (u == "deg")
        return TextFormat::Degrees;
    if (u == "oct")
        return TextFormat::Octaves;
    if (u == "bit")
        return TextFormat::Bits;
    if (u == "bars")
        return TextFormat::Bars;
    return TextFormat::Number;
}

std::string format(const ParamDef& d, float value)
{
    float v = std::isnan(value) ? d.defaultValue : std::clamp(value, d.min, d.max);
    if (d.type != ParamType::Float)
        v = std::clamp(std::round(v), d.min, d.max);
    const double x = v;
    switch (resolveFormat(d))
    {
    case TextFormat::Choice:
    {
        const auto i = static_cast<size_t>(x);
        return i < d.choices.size() ? d.choices[i] : number(x);
    }
    case TextFormat::Toggle:
        return x > 0.5 ? "On" : "Off";
    case TextFormat::Hertz:
        return number(x) + " Hz";
    case TextFormat::Seconds:
        return number(x) + " s";
    case TextFormat::Milliseconds:
        return number(x) + " ms";
    case TextFormat::Decibels:
        return number(x) + " dB";
    case TextFormat::Percent:
        return number(x) + " %";
    case TextFormat::Semitones:
        return signedNumber(x) + " st";
    case TextFormat::Cents:
        return signedNumber(x) + " ct";
    case TextFormat::NoteName:
        return noteName(static_cast<int>(std::lround(x)));
    case TextFormat::Ratio:
        return number(x) + ":1";
    case TextFormat::Bpm:
        return number(x) + " BPM";
    case TextFormat::Degrees:
        return number(x) + " deg";
    case TextFormat::Octaves:
        return number(x) + " oct";
    case TextFormat::Bits:
        return number(x) + " bit";
    case TextFormat::Bars:
        return number(x) + (x == 1.0 ? " bar" : " bars");
    case TextFormat::Number:
    case TextFormat::Auto:
        break;
    }
    return number(x) + (d.unit.empty() ? "" : " " + d.unit);
}

bool parse(const ParamDef& d, const std::string& text, float& out)
{
    const TextFormat f = resolveFormat(d);
    const std::string t = trim(text);
    double v = 0.0;
    if (f == TextFormat::Choice)
    {
        for (size_t i = 0; i < d.choices.size(); ++i)
            if (d.choices[i] == t)
            {
                out = static_cast<float>(i);
                return true;
            }
        const std::string lt = lower(t);
        for (size_t i = 0; i < d.choices.size(); ++i)
            if (lower(d.choices[i]) == lt)
            {
                out = static_cast<float>(i);
                return true;
            }
        std::string suffix;
        if (split(t, v, suffix) && suffix.empty() && v >= 0.0 && v < static_cast<double>(d.choices.size()) &&
            v == std::floor(v))
        {
            out = static_cast<float>(v);
            return true;
        }
        // Last: the text against each choice name without its trailing " (...)". More than one match is refused.
        int found = -1;
        for (size_t i = 0; i < d.choices.size(); ++i)
        {
            const std::string& name = d.choices[i];
            const size_t open = name.rfind(" (");
            if (open == std::string::npos || name.back() != ')' || lower(name.substr(0, open)) != lt)
                continue;
            if (found >= 0)
                return false;
            found = static_cast<int>(i);
        }
        if (found < 0)
            return false;
        out = static_cast<float>(found);
        return true;
    }
    if (f == TextFormat::Toggle)
    {
        const std::string lt = lower(t);
        if (oneOf(lt, {"on", "true", "yes", "1"}))
            out = 1.0f;
        else if (oneOf(lt, {"off", "false", "no", "0"}))
            out = 0.0f;
        else
            return false;
        return true;
    }
    if (f == TextFormat::NoteName)
    {
        int n = 0;
        if (parseNote(t, n))
            v = n;
        else
        {
            std::string suffix;
            if (!split(t, v, suffix) || !suffix.empty())
                return false;
        }
    }
    else
    {
        std::string s;
        if (!split(t, v, s))
            return false;
        bool ok = false;
        switch (f)
        {
        case TextFormat::Hertz:
            if (oneOf(s, {"", "hz"}))
                ok = true;
            else if (oneOf(s, {"k", "khz"}))
            {
                v *= 1000.0;
                ok = true;
            }
            break;
        case TextFormat::Seconds:
            if (oneOf(s, {"", "s", "sec"}))
                ok = true;
            else if (s == "ms")
            {
                v /= 1000.0;
                ok = true;
            }
            break;
        case TextFormat::Milliseconds:
            if (oneOf(s, {"", "ms"}))
                ok = true;
            else if (oneOf(s, {"s", "sec"}))
            {
                v *= 1000.0;
                ok = true;
            }
            break;
        case TextFormat::Decibels:
            ok = oneOf(s, {"", "db"});
            break;
        case TextFormat::Percent:
            ok = oneOf(s, {"", "%"});
            break;
        case TextFormat::Semitones:
            ok = oneOf(s, {"", "st", "semi"});
            break;
        case TextFormat::Cents:
            ok = oneOf(s, {"", "ct", "cents"});
            break;
        case TextFormat::Ratio:
            ok = oneOf(s, {"", ":1", ":", "x"});
            break;
        case TextFormat::Bpm:
            ok = oneOf(s, {"", "bpm"});
            break;
        case TextFormat::Degrees:
            ok = oneOf(s, {"", "deg", "\xC2\xB0"});
            break;
        case TextFormat::Octaves:
            ok = oneOf(s, {"", "oct"});
            break;
        case TextFormat::Bits:
            ok = oneOf(s, {"", "bit", "bits"});
            break;
        case TextFormat::Bars:
            ok = oneOf(s, {"", "bar", "bars"});
            break;
        default:
            ok = s.empty() || lower(d.unit) == s;
            break;
        }
        if (!ok)
            return false;
    }
    float r = std::clamp(static_cast<float>(v), d.min, d.max);
    if (d.type != ParamType::Float)
        r = std::clamp(std::round(r), d.min, d.max);
    out = r;
    return true;
}

std::string shortNameOf(const std::string& name)
{
    std::vector<std::string> words;
    std::string cur;
    for (const char c : name)
    {
        if (c == ' ' || c == '/' || c == '-')
        {
            if (!cur.empty())
                words.push_back(cur);
            cur.clear();
        }
        else if (static_cast<unsigned char>(c) < 128 && std::isprint(static_cast<unsigned char>(c)))
            cur += c;
    }
    if (!cur.empty())
        words.push_back(cur);
    const auto total = [&]
    {
        size_t n = 0;
        for (const auto& w : words)
            n += w.size();
        return n;
    };
    if (words.size() == 1 && words[0].size() <= 6)
        return words[0];
    if (name.size() <= 6 && words.size() >= 1)
    {
        std::string s;
        for (const auto& w : words)
            s += w;
        if (s.size() <= 6 && name.find(' ') == std::string::npos)
            return s;
    }
    while (total() > 6)
    {
        size_t longest = 0;
        for (size_t i = 1; i < words.size(); ++i)
            if (words[i].size() > words[longest].size())
                longest = i;
        auto& w = words[longest];
        bool removed = false;
        for (size_t i = w.size(); i-- > 1;)
            if (std::strchr("aeiou", w[i]) != nullptr)
            {
                w.erase(i, 1);
                removed = true;
                break;
            }
        if (!removed)
            w.pop_back();
    }
    std::string s;
    for (const auto& w : words)
        s += w;
    return s.empty() ? "?" : s;
}
} // namespace ParamText
} // namespace crucible
