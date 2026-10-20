// SPDX-License-Identifier: MIT
#pragma once

// Locale-independent number formatting and parsing for the plain-text state formats (scenes, patterns).
// Message-thread helpers only: they allocate.

#include <cstdlib>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace crucible::text
{

// Fixed notation with the given number of decimals ("0.500000").
inline std::string fixed(double v, int decimals)
{
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os.setf(std::ios_base::fixed);
    os.precision(decimals);
    os << v;
    return os.str();
}

// Splits on a single separator character; empty tokens are dropped.
inline std::vector<std::string> split(const std::string& s, char sep)
{
    std::vector<std::string> out;
    std::string cur;
    for (const char c : s)
    {
        if (c == sep)
        {
            if (!cur.empty())
                out.push_back(cur);
            cur.clear();
        }
        else
        {
            cur.push_back(c);
        }
    }
    if (!cur.empty())
        out.push_back(cur);
    return out;
}

inline std::string join(const std::vector<std::string>& parts, char sep)
{
    std::string out;
    for (size_t i = 0; i < parts.size(); ++i)
    {
        if (i > 0)
            out.push_back(sep);
        out += parts[i];
    }
    return out;
}

// Leading integer of the string, 0 when there is none.
inline int toInt(const std::string& s)
{
    return static_cast<int>(std::strtol(s.c_str(), nullptr, 10));
}

// Leading decimal number of the string, 0 when there is none.
inline float toFloat(const std::string& s)
{
    std::istringstream is(s);
    is.imbue(std::locale::classic());
    double v = 0.0;
    if (!(is >> v))
        return 0.0f;
    return static_cast<float>(v);
}

} // namespace crucible::text
