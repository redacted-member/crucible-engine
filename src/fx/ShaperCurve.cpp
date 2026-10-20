// SPDX-License-Identifier: MIT
#include <crucible/fx/ShaperCurve.h>

namespace crucible
{

bool makeShaperCurveTable(const ShaperPoint* points, int count, bool symmetric, ShaperCurveTable& out, std::string& error)
{
    if (points == nullptr || count < 2 || count > kMaxShaperPoints)
    {
        error = "a shaper curve takes 2 to " + std::to_string(kMaxShaperPoints) + " points";
        return false;
    }
    for (int i = 0; i < count; ++i)
    {
        const ShaperPoint& q = points[i];
        const std::string at = "point " + std::to_string(i) + ": ";
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.curve))
        {
            error = at + "x, y and curve must be finite";
            return false;
        }
        if (q.x < -1.0f || q.x > 1.0f)
        {
            error = at + "x must be within -1..1";
            return false;
        }
        if (q.y < -1.0f || q.y > 1.0f)
        {
            error = at + "y must be within -1..1";
            return false;
        }
        if (q.curve < -1.0f || q.curve > 1.0f)
        {
            error = at + "curve must be within -1..1";
            return false;
        }
        if (i > 0 && !(q.x > points[i - 1].x))
        {
            error = at + "x must be strictly ascending";
            return false;
        }
        if (symmetric && q.x < 0.0f)
        {
            error = at + "a symmetric curve draws x in 0..1";
            return false;
        }
        if (symmetric && q.x == 0.0f && q.y != 0.0f)
        {
            error = at + "a symmetric curve passes through the origin (y must be 0 at x = 0)";
            return false;
        }
    }
    ShaperCurveTable t;
    int n = 0;
    if (symmetric && points[0].x > 0.0f)
    {
        t.x[0] = 0.0f;
        t.y[0] = 0.0f;
        t.curve[0] = 0.0f;
        n = 1;
    }
    for (int i = 0; i < count; ++i, ++n)
    {
        t.x[static_cast<size_t>(n)] = points[i].x;
        t.y[static_cast<size_t>(n)] = points[i].y;
        t.curve[static_cast<size_t>(n)] = points[i].curve;
    }
    t.n = n;
    t.symmetric = symmetric;
    bool identity = true;
    for (int i = 0; i < n; ++i)
    {
        const auto s = static_cast<size_t>(i);
        if (i + 1 < n)
        {
            t.dy[s] = t.y[s + 1] - t.y[s];
            t.inv[s] = 1.0f / (t.x[s + 1] - t.x[s]);
            t.p[s] = t.curve[s] == 0.0f ? 1.0f : std::exp2(2.0f * t.curve[s]);
            identity = identity && t.curve[s] == 0.0f;
        }
        identity = identity && t.y[s] == t.x[s];
    }
    // The ends must be at +-1 (the symmetric half only at +1), otherwise the straight line would clip there.
    identity = identity && t.x[static_cast<size_t>(n - 1)] == 1.0f && (symmetric || t.x[0] == -1.0f);
    t.identity = identity;
    out = t;
    return true;
}

} // namespace crucible
