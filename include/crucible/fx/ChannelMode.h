// SPDX-License-Identifier: MIT
#pragma once

// Channel modes of stereo processors (docs/plans/ENGINE-FX-PLAN.md, section 4.6): one place for left / right,
// linked, unlinked, mid, side and mid / side operation so that every module offers the same set.
//
// A processor sees two "lanes" a and b. encode() turns left / right into lanes for the mode, the processor works on
// the lanes it is meant to touch (laneActive()), decode() turns the lanes back into left / right. Mid and side use
// M = (L + R) / 2 and S = (L - R) / 2, so decode(encode(x)) is the identity up to float rounding and a mid-only
// gain of g changes a mono signal by exactly g.

#include <algorithm>

namespace crucible::fx
{

enum class ChannelMode
{
    StereoLinked,   // left and right, one shared detector
    StereoUnlinked, // left and right, independent detectors
    Mid,            // mid processed, side untouched
    Side,           // side processed, mid untouched
    MidSide,        // mid and side processed independently (dual)
    Left,           // left processed, right untouched
    Right,          // right processed, left untouched
    Count
};

inline const char* channelModeName(ChannelMode mode)
{
    switch (mode)
    {
    case ChannelMode::StereoLinked:
        return "Stereo linked";
    case ChannelMode::StereoUnlinked:
        return "Stereo unlinked";
    case ChannelMode::Mid:
        return "Mid";
    case ChannelMode::Side:
        return "Side";
    case ChannelMode::MidSide:
        return "Mid / Side";
    case ChannelMode::Left:
        return "Left";
    case ChannelMode::Right:
        return "Right";
    default:
        return "?";
    }
}

inline bool usesMidSide(ChannelMode mode)
{
    return mode == ChannelMode::Mid || mode == ChannelMode::Side || mode == ChannelMode::MidSide;
}

// Whether the processor works on lane 0 (a) or lane 1 (b) in this mode; an inactive lane passes through unchanged.
inline bool laneActive(ChannelMode mode, int lane)
{
    switch (mode)
    {
    case ChannelMode::Mid:
    case ChannelMode::Left:
        return lane == 0;
    case ChannelMode::Side:
    case ChannelMode::Right:
        return lane == 1;
    default:
        return true;
    }
}

// Detector link amount actually used: linked forces 1, unlinked forces 0, the others use the parameter (0..1).
inline float effectiveLink(ChannelMode mode, float link)
{
    if (mode == ChannelMode::StereoLinked)
        return 1.0f;
    if (mode == ChannelMode::StereoUnlinked)
        return 0.0f;
    return std::clamp(link, 0.0f, 1.0f);
}

// Detector levels of the two lanes after linking: each lane moves toward the larger of the two by `link`.
// Works on any monotonic level (linear or dB).
inline void linkLevels(float& a, float& b, float link)
{
    const float peak = std::max(a, b);
    a += link * (peak - a);
    b += link * (peak - b);
}

inline void encode(ChannelMode mode, float& l, float& r)
{
    if (usesMidSide(mode))
    {
        const float m = 0.5f * (l + r), s = 0.5f * (l - r);
        l = m;
        r = s;
    }
}

inline void decode(ChannelMode mode, float& a, float& b)
{
    if (usesMidSide(mode))
    {
        const float l = a + b, r = a - b;
        a = l;
        b = r;
    }
}

inline void encode(ChannelMode mode, float* l, float* r, int n)
{
    if (!usesMidSide(mode))
        return;
    for (int i = 0; i < n; ++i)
        encode(mode, l[i], r[i]);
}

inline void decode(ChannelMode mode, float* a, float* b, int n)
{
    if (!usesMidSide(mode))
        return;
    for (int i = 0; i < n; ++i)
        decode(mode, a[i], b[i]);
}

} // namespace crucible::fx
