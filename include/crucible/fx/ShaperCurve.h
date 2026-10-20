// SPDX-License-Identifier: MIT
#pragma once

// The transfer curve of fx.shaper: a user-drawn point curve with a curvature per segment.
//
// The user gives 2..64 points (x in -1..1 strictly ascending, y in -1..1) and a curvature per point in -1..1 that bends
// the segment from that point to the next (the last point's curvature is unused). A segment between (x0, y0) and
// (x1, y1) is
//
//   y = y0 + (y1 - y0) * t^p,   t = (x - x0) / (x1 - x0),   p = 4^curve
//
// so curve 0 is the straight line, a positive curvature bows the segment towards the start value (slow start, fast
// finish) and a negative one the other way; p runs from 1/4 to 4. Outside the first and last point the curve holds the end
// value (a curve that ends at +-1 clips there), with one exception: the identity curve (every point on y = x, no
// curvature, the ends at +-1, which is the default) is the unbounded straight line, so the default module never clips.
//
// Symmetric mode: the points draw the positive half only (x in 0..1) and the curve is odd, f(-x) = -f(x). The curve passes
// through the origin: when the first point is right of 0 an implicit point (0, 0) with curvature 0 is added, and a point at
// x = 0 must have y = 0.
//
// A table is plain data: built on the control thread by makeShaperCurveTable() and copied into the audio thread's state at a
// block boundary, so evaluating it never allocates.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace crucible
{

constexpr int kMaxShaperPoints = 64;

struct ShaperPoint
{
    float x = 0.0f, y = 0.0f, curve = 0.0f;
};

struct ShaperCurveTable
{
    // One more than the user's limit: the implicit origin of the symmetric mode.
    static constexpr int kMax = kMaxShaperPoints + 1;

    int n = 2;
    bool symmetric = false;
    bool identity = true;
    std::uint64_t serial = 0; // set by the module that publishes the table, to tell a new one from the one in use
    std::array<float, kMax> x{-1.0f, 1.0f};
    std::array<float, kMax> y{-1.0f, 1.0f};
    std::array<float, kMax> dy{}; // y[i + 1] - y[i]
    std::array<float, kMax> inv{};// 1 / (x[i + 1] - x[i])
    std::array<float, kMax> p{};  // exponent of the segment after point i
    std::array<float, kMax> curve{};

    ShaperCurveTable()
    {
        dy[0] = 2.0f;
        inv[0] = 0.5f;
        p[0] = 1.0f;
    }

    float eval(float in) const
    {
        if (identity)
            return in;
        const float a = symmetric ? std::fabs(in) : in;
        float out;
        if (a <= x[0])
            out = y[0];
        else if (a >= x[static_cast<size_t>(n - 1)])
            out = y[static_cast<size_t>(n - 1)];
        else
        {
            int lo = 0, hi = n - 1; // x[lo] < a < x[hi]
            while (hi - lo > 1)
            {
                const int mid = (lo + hi) >> 1;
                if (x[static_cast<size_t>(mid)] <= a)
                    lo = mid;
                else
                    hi = mid;
            }
            const auto i = static_cast<size_t>(lo);
            const float t = (a - x[i]) * inv[i];
            out = y[i] + dy[i] * (p[i] == 1.0f ? t : std::pow(t, p[i]));
        }
        return symmetric ? std::copysign(out, in) : out;
    }
};

// Validates the points and builds the table; false with the reason in error. Rules: 2..64 finite points, x within -1..1 and
// strictly ascending, y and curve within -1..1; symmetric mode: x >= 0 and y = 0 at x = 0.
bool makeShaperCurveTable(const ShaperPoint* points, int count, bool symmetric, ShaperCurveTable& out, std::string& error);

} // namespace crucible
