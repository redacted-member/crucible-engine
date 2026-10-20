// SPDX-License-Identifier: MIT
#include <crucible/Saturation.h>

#include "SaturationAvx.h"
#if CRUCIBLE_SATURATION_AVX
#include <intrin.h>
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <type_traits>

namespace crucible::saturation
{
namespace
{
constexpr double kPiD = 3.14159265358979323846;
constexpr double kAdaaEpsilon = 1.0e-5;

// tanh with a Pade approximant: no exp, accurate to about 1e-7 where it matters; saturates past |x| = 4.97.
CRUCIBLE_ALWAYS_INLINE double fastTanh(double x)
{
    if (x > 4.97)
        return 1.0;
    if (x < -4.97)
        return -1.0;
    const double x2 = x * x;
    const double num = x * (135135.0 + x2 * (17325.0 + x2 * (378.0 + x2)));
    const double den = 135135.0 + x2 * (62370.0 + x2 * (3150.0 + x2 * 28.0));
    return num / den;
}

// ------------------------------------------------------------------------------------------------------------
// Memoryless curves with closed-form antiderivatives. Each has f'(0) = 1 and F(0) = 0.

struct SoftCurve
{
    static double f(double x) { return std::tanh(x); }
    static double F(double x)
    {
        const double a = std::abs(x);
        return a + std::log1p(std::exp(-2.0 * a)) - 0.69314718055994531; // ln cosh x
    }
};

struct HardCurve
{
    static double f(double x) { return std::clamp(x, -1.0, 1.0); }
    static double F(double x)
    {
        const double a = std::abs(x);
        return a <= 1.0 ? 0.5 * x * x : a - 0.5;
    }
};

// Op-amp rail: a cubic knee that meets the rail with zero slope (output limit 1, knee between 0 and 1.5).
struct RailCurve
{
    static double f(double x)
    {
        const double u = std::clamp(x / 1.5, -1.0, 1.0);
        return 1.5 * (u - u * u * u / 3.0);
    }
    static double F(double x)
    {
        const double a = std::abs(x);
        if (a >= 1.5)
            return 0.9375 + (a - 1.5);
        const double u = a / 1.5;
        return 2.25 * (0.5 * u * u - u * u * u * u / 12.0);
    }
};

// Diode clipper: exponential conduction on both sides, the negative side clips earlier (0.7 of the positive side);
// the slope is 1 on both sides of the origin.
struct DiodeCurve
{
    static constexpr double c = 0.7;
    static double f(double x) { return x >= 0.0 ? 1.0 - std::exp(-x) : -c * (1.0 - std::exp(x / c)); }
    static double F(double x) { return x >= 0.0 ? x + std::exp(-x) - 1.0 : -c * x + c * c * (std::exp(x / c) - 1.0); }
};

// Triangle foldback with period 4 and peak 1.
struct FoldCurve
{
    CRUCIBLE_ALWAYS_INLINE static double wrap(double x) { return x - 4.0 * std::floor((x + 1.0) * 0.25); } // in [-1, 3)
    CRUCIBLE_ALWAYS_INLINE static double f(double x)
    {
        const double t = wrap(x);
        return t <= 1.0 ? t : 2.0 - t;
    }
    CRUCIBLE_ALWAYS_INLINE static double F(double x)
    {
        const double t = wrap(x);
        return t <= 1.0 ? 0.5 * t * t : 2.0 * t - 0.5 * t * t - 1.0; // periodic, mean 1/2
    }
    // Antiderivative of F: 2 per period (the integral of F over one) plus a piecewise cubic.
    CRUCIBLE_ALWAYS_INLINE static double F2(double x)
    {
        const double k = std::floor((x + 1.0) * 0.25);
        const double t = x - 4.0 * k;
        const double g = t <= 1.0 ? (t * t * t + 1.0) / 6.0 : 0.5 + t * t - t * t * t / 6.0 - t;
        return g + 2.0 * k;
    }
};

// Appended shapers. Soft's ln cosh is the antiderivative of tanh.
struct RectHalfCurve
{
    static double f(double x) { return x > 0.0 ? std::tanh(x) : 0.0; }
    static double F(double x) { return x > 0.0 ? SoftCurve::F(x) : 0.0; }
};

struct RectFullCurve
{
    static double f(double x) { return std::abs(std::tanh(x)); }
    static double F(double x) { return x >= 0.0 ? SoftCurve::F(x) : -SoftCurve::F(x); }
};

// tanh(8 x): slope 8 at the origin, so a small input already gives a near-square wave.
struct ZeroSquareCurve
{
    static constexpr double k = 8.0;
    static double f(double x) { return std::tanh(k * x); }
    static double F(double x) { return SoftCurve::F(k * x) / k; }
};

// The positive half is tanh x; the negative half saturates at 0.4 (slope 1 at the origin on both sides).
struct AsymCurve
{
    static constexpr double c = 0.4;
    static double f(double x) { return x >= 0.0 ? std::tanh(x) : c * std::tanh(x / c); }
    static double F(double x) { return x >= 0.0 ? SoftCurve::F(x) : c * c * SoftCurve::F(x / c); }
};

struct SineFoldCurve
{
    static double f(double x) { return std::sin(x); }
    static double F(double x) { return 1.0 - std::cos(x); }
    static double F2(double x) { return x - std::sin(x); }
};

// The triangle foldback at twice the input: f(x) = Fold(2 x), so the first fold is at |x| = 0.5.
struct LinFoldCurve
{
    static double f(double x) { return FoldCurve::f(2.0 * x); }
    static double F(double x) { return 0.5 * FoldCurve::F(2.0 * x); }
    static double F2(double x) { return 0.25 * FoldCurve::F2(2.0 * x); }
};

// ------------------------------------------------------------------------------------------------------------
// Tabulated curves

ShapeTable buildTable(double range, int cells, const std::function<double(double)>& fn)
{
    ShapeTable t;
    t.range = range;
    t.step = 2.0 * range / cells;
    t.f.resize(static_cast<size_t>(cells) + 1);
    t.F.resize(static_cast<size_t>(cells) + 1);
    t.F2.resize(static_cast<size_t>(cells) + 1);
    for (int i = 0; i <= cells; ++i)
        t.f[static_cast<size_t>(i)] = fn(-range + i * t.step);
    t.F[0] = 0.0;
    for (int i = 0; i < cells; ++i)
        t.F[static_cast<size_t>(i) + 1] =
            t.F[static_cast<size_t>(i)] + 0.5 * (t.f[static_cast<size_t>(i)] + t.f[static_cast<size_t>(i) + 1]) * t.step;
    const double f0 = t.F[static_cast<size_t>(cells / 2)];
    for (auto& v : t.F)
        v -= f0;
    // F2 integrates the piecewise quadratic F exactly: F_i h + f_i h^2 / 2 + slope h^3 / 6 over a cell.
    t.F2[0] = 0.0;
    for (int i = 0; i < cells; ++i)
    {
        const auto k = static_cast<size_t>(i);
        const double h = t.step, slope = (t.f[k + 1] - t.f[k]) / h;
        t.F2[k + 1] = t.F2[k] + t.F[k] * h + 0.5 * t.f[k] * h * h + slope * h * h * h / 6.0;
    }
    const double g0 = t.F2[static_cast<size_t>(cells / 2)];
    for (auto& v : t.F2)
        v -= g0;
    t.cells = cells;
    t.invStep = 1.0 / t.step;
    t.node.resize(static_cast<size_t>(cells) + 3);
    for (int i = 0; i < cells; ++i)
    {
        const auto k = static_cast<size_t>(i);
        t.node[k] = {t.f[k], t.F[k], t.F2[k], (t.f[k + 1] - t.f[k]) / t.step};
    }
    t.node[static_cast<size_t>(cells)] = {t.f[static_cast<size_t>(cells)], t.F[static_cast<size_t>(cells)],
                                          t.F2[static_cast<size_t>(cells)], 0.0};
    t.node[static_cast<size_t>(cells) + 1] = {t.f.front(), t.F.front(), t.F2.front(), 0.0};
    t.node[static_cast<size_t>(cells) + 2] = {t.f.back(), t.F.back(), t.F2.back(), 0.0};
    return t;
}

double softplus(double x)
{
    return x > 30.0 ? x : std::log1p(std::exp(x));
}

// Koren's triode: plate current in mA from grid and plate voltage.
double triodeCurrent(double vg, double vp)
{
    constexpr double mu = 100.0, kx = 1.4, kg1 = 1060.0, kp = 600.0, kvb = 300.0; // a 12AX7-class fit
    const double e1 = vp / kp * softplus(kp * (1.0 / mu + vg / std::sqrt(kvb + vp * vp)));
    return e1 > 0.0 ? 2.0 * std::pow(e1, kx) / kg1 : 0.0;
}

// Koren's pentode (power pentode class: mu 11, kp 60, kvb 24, screen 250 V).
double pentodeCurrent(double vg, double vp)
{
    constexpr double mu = 11.0, kx = 1.35, kg1 = 1460.0, kp = 60.0, kvb = 24.0, vg2 = 250.0;
    const double e1 = vg2 / kp * softplus(kp * (1.0 / mu + vg / vg2));
    return e1 > 0.0 ? 2.0 * std::pow(e1, kx) / kg1 * std::atan(vp / kvb) : 0.0;
}

// Grid current (mA) of a conducting grid; the knee sits at the cathode voltage.
double gridCurrent(double vgk, double scale)
{
    constexpr double cg = 13.5, ex = 1.5;
    return std::pow(softplus(cg * vgk) / cg, ex) * scale;
}

struct TubeStage
{
    std::function<double(double, double)> plate; // (vg, vp) -> mA
    double supply = 250.0, loadK = 100.0;        // volts, kilo-ohms
    double biasV = -1.4;                         // quiescent grid voltage
    double voltsPerUnit = 1.4;                   // grid volts per unit of input
    double sourceK = 33.0;                       // grid series resistance, kilo-ohms
    double gridScale = 0.1;                      // grid current scale, mA

    // Plate voltage for an input of x units, with grid conduction through the source resistance and the load line.
    double plateVoltage(double x) const
    {
        const double vs = biasV + voltsPerUnit * x;
        double lo = vs - 25.0, hi = vs;
        for (int i = 0; i < 80; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            if (mid + sourceK * gridCurrent(mid - 0.0, gridScale) - vs > 0.0)
                hi = mid;
            else
                lo = mid;
        }
        const double vg = 0.5 * (lo + hi);
        double a = 0.0, b = supply;
        for (int i = 0; i < 80; ++i)
        {
            const double mid = 0.5 * (a + b);
            if (mid - (supply - loadK * plate(vg, mid)) > 0.0)
                b = mid;
            else
                a = mid;
        }
        return 0.5 * (a + b);
    }

    // Non-inverting transfer, slope 1 at the origin, zero at the origin.
    std::function<double(double)> curve() const
    {
        const double h = 1.0e-3;
        const double slope = -(plateVoltage(h) - plateVoltage(-h)) / (2.0 * h);
        const double v0 = plateVoltage(0.0);
        const TubeStage self = *this;
        return [self, slope, v0](double x) { return -(self.plateVoltage(x) - v0) / slope; };
    }
};

struct TubeTables
{
    ShapeTable triode, pushPull, pentode;
    TubeTables()
    {
        constexpr double range = 12.0;
        constexpr int cells = 4800;
        TubeStage t;
        t.plate = triodeCurrent;
        const auto tf = t.curve();
        triode = buildTable(range, cells, tf);
        pushPull = buildTable(range, cells, [&](double x) { return 0.5 * (tf(x) - tf(-x)); });

        TubeStage p;
        p.plate = pentodeCurrent;
        p.supply = 300.0;
        p.loadK = 3.0;
        p.biasV = -14.0;
        p.voltsPerUnit = 14.0;
        p.sourceK = 20.0;
        p.gridScale = 0.5;
        pentode = buildTable(range, cells, p.curve());
    }
};

const TubeTables& tubeTables()
{
    static const TubeTables tables; // built on first prepare(), a control-thread call
    return tables;
}

// The memoryless curves as tables for the second-order ADAA (the same functions as the closed forms above, to the
// accuracy of the 0.005 grid; the knees of Hard and Rail sit on grid nodes).
struct CurveTables
{
    ShapeTable soft, hard, rail, diode, rectHalf, rectFull, zeroSquare, asym;
    CurveTables()
    {
        soft = buildTable(8.0, 3200, [](double x) { return SoftCurve::f(x); });
        hard = buildTable(2.0, 800, [](double x) { return HardCurve::f(x); });
        rail = buildTable(2.0, 800, [](double x) { return RailCurve::f(x); });
        diode = buildTable(16.0, 6400, [](double x) { return DiodeCurve::f(x); });
        rectHalf = buildTable(8.0, 3200, [](double x) { return RectHalfCurve::f(x); });
        rectFull = buildTable(8.0, 3200, [](double x) { return RectFullCurve::f(x); });
        zeroSquare = buildTable(2.0, 4000, [](double x) { return ZeroSquareCurve::f(x); });
        asym = buildTable(8.0, 3200, [](double x) { return AsymCurve::f(x); });
    }
};

const CurveTables& curveTables()
{
    static const CurveTables tables; // built on first prepare(), a control-thread call
    return tables;
}

// ------------------------------------------------------------------------------------------------------------
// Jiles-Atherton hysteresis in normalised units: saturation magnetisation 1, anhysteretic knee `a` (tanh form,
// the J = 1/2 Brillouin function), pinning `k`, domain coupling `alpha`, reversibility `c`.

inline JaParams jaParams(double hysteresis)
{
    JaParams p;
    p.k = 0.01 + 0.08 * std::clamp(hysteresis, 0.0, 1.0);
    p.invA = 1.0 / p.a;
    return p;
}

// Slope dM/dH. Single precision is enough for the slope (it only scales a small step); the state stays in double.
inline double jaSlope(const JaParams& p, double m, double h, double delta)
{
    const float he = static_cast<float>(h + p.alpha * m);
    float x = he * static_cast<float>(p.invA);
    x = std::clamp(x, -4.97f, 4.97f);
    const float x2 = x * x;
    const float t =
        x * (135135.0f + x2 * (17325.0f + x2 * (378.0f + x2))) / (135135.0f + x2 * (62370.0f + x2 * (3150.0f + x2 * 28.0f)));
    const auto c = static_cast<float>(p.c);
    const auto alpha = static_cast<float>(p.alpha);
    const float dMan = (1.0f - t * t) * static_cast<float>(p.invA);
    const float dd = static_cast<float>(delta) * (t - static_cast<float>(m));
    float slope;
    if (dd > 0.0f)
    {
        const float d2 = std::max(static_cast<float>(p.k) - alpha * dd, 1.0e-4f);
        const float num = (1.0f - c) * dd + c * dMan * d2;
        const float den = d2 * (1.0f - alpha * c * dMan) - alpha * (1.0f - c) * dd;
        slope = num / std::max(den, 0.05f * d2);
    }
    else
    {
        slope = c * dMan / std::max(1.0f - alpha * c * dMan, 0.05f);
    }
    return std::max(slope, 0.0f);
}

// One integration step of the hysteresis from h0 to h1 (a single RK2 or RK4 step); returns the new magnetisation.
inline double jaSubStep(const JaParams& p, double m, double h0, double h1, int order)
{
    const double dh = h1 - h0;
    if (std::abs(dh) < 1.0e-12)
        return m;
    const double d = dh > 0.0 ? 1.0 : -1.0;
    double out;
    if (order >= 3)
    {
        const double k1 = jaSlope(p, m, h0, d);
        const double k2 = jaSlope(p, m + 0.5 * dh * k1, h0 + 0.5 * dh, d);
        const double k3 = jaSlope(p, m + 0.5 * dh * k2, h0 + 0.5 * dh, d);
        const double k4 = jaSlope(p, m + dh * k3, h1, d);
        out = m + dh * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
    }
    else if (order == 2)
    {
        const double k1 = jaSlope(p, m, h0, d);
        const double k2 = jaSlope(p, m + 0.5 * dh * k1, h0 + 0.5 * dh, d);
        out = m + dh * k2;
    }
    else
    {
        out = m + dh * jaSlope(p, m, h0, d);
    }
    return std::clamp(out, -1.0, 1.0);
}

// A sample step is split into pieces of at most kMaxFieldStep (up to kMaxSubSteps of them) so that a hard-driven
// input, which moves the field by many knee widths per sample, still integrates accurately.
constexpr double kMaxFieldStep = 0.25;
constexpr int kMaxSubSteps = 4;

inline double jaStep(const JaParams& p, double m, double h0, double h1, int order)
{
    const double dh = h1 - h0;
    if (std::abs(dh) <= kMaxFieldStep)
        return jaSubStep(p, m, h0, h1, order);
    const int pieces = std::clamp(static_cast<int>(std::ceil(std::abs(dh) / kMaxFieldStep)), 1, kMaxSubSteps);
    double h = h0;
    for (int i = 1; i <= pieces; ++i)
    {
        const double hn = h0 + dh * i / pieces;
        m = jaSubStep(p, m, h, hn, order);
        h = hn;
    }
    return m;
}

// Gain of the hysteresis for a -40 dBFS sine, so that the model is transparent at small signals.
double calibrateTape(double hysteresis, int order)
{
    const JaParams p = jaParams(hysteresis);
    constexpr double amp = 0.01;
    constexpr int perPeriod = 96, periods = 10;
    double m = 0.0, hPrev = 0.0, re = 0.0, im = 0.0;
    for (int n = 1; n <= perPeriod * periods; ++n)
    {
        const double ph = 2.0 * kPiD * n / perPeriod;
        const double h = amp * std::sin(ph);
        m = jaStep(p, m, hPrev, h, order);
        hPrev = h;
        if (n > perPeriod * (periods - 4))
        {
            re += m * std::sin(ph);
            im += m * std::cos(ph);
        }
    }
    const double mag = 2.0 * std::sqrt(re * re + im * im) / (perPeriod * 4);
    return mag > 1.0e-12 ? amp / mag : 1.0;
}

// Relaxation form of the hysteresis, for the block kernel. The magnetisation follows the anhysteretic curve
// Man(h) = tanh(h) in two parts per step of the field: the reversible share c of its change, and an irreversible
// relaxation toward Man that is active only while the magnetisation lags behind it in the direction of travel, with the
// pinning k setting how far the field must move for it to catch up (m -> Man + (m - Man) exp(-(1 - c) |dh| / k), the
// exponential written as a reciprocal of its series so that any step of the field is stable). The state is the same pair
// (m, previous field) as the Jiles-Atherton form, but a step needs no division and its chain is a max and a fused
// multiply-add.
CRUCIBLE_ALWAYS_INLINE float fastTanhF(float x)
{
    x = std::clamp(x, -4.97f, 4.97f);
    const float x2 = x * x;
    return x * (135135.0f + x2 * (17325.0f + x2 * (378.0f + x2))) /
           (135135.0f + x2 * (62370.0f + x2 * (3150.0f + x2 * 28.0f)));
}

struct TapeFastCoef
{
    float c, gScale;
    explicit TapeFastCoef(const JaParams& p) : c(static_cast<float>(p.c)), gScale(static_cast<float>((1.0 - p.c) / p.k)) {}
};

CRUCIBLE_ALWAYS_INLINE float relaxWeight(float g)
{
    return 1.0f - 1.0f / (1.0f + g * (1.0f + g * (0.5f + g * (1.0f / 6.0f))));
}

CRUCIBLE_ALWAYS_INLINE double tapeFastScalar(const TapeFastCoef& q, double m, double h0, double h1)
{
    const double dh = h1 - h0;
    const float delta = dh >= 0.0 ? 1.0f : -1.0f;
    const float man0 = fastTanhF(static_cast<float>(h0)), man1 = fastTanhF(static_cast<float>(h1));
    const float cm = q.c * (man1 - man0);
    const float w = relaxWeight(q.gScale * static_cast<float>(std::abs(dh)));
    const float mm = static_cast<float>(m);
    const float p = mm + cm;
    const float s = std::max(delta * (man1 - p), 0.0f);
    return static_cast<double>(p + w * delta * s);
}

double calibrateTapeFast(double hysteresis)
{
    const TapeFastCoef q(jaParams(hysteresis));
    constexpr double amp = 0.01;
    constexpr int perPeriod = 96, periods = 10;
    double m = 0.0, hPrev = 0.0, re = 0.0, im = 0.0;
    for (int n = 1; n <= perPeriod * periods; ++n)
    {
        const double ph = 2.0 * kPiD * n / perPeriod;
        const double h = amp * std::sin(ph);
        m = tapeFastScalar(q, m, hPrev, h);
        hPrev = h;
        if (n > perPeriod * (periods - 4))
        {
            re += m * std::sin(ph);
            im += m * std::cos(ph);
        }
    }
    const double mag = 2.0 * std::sqrt(re * re + im * im) / (perPeriod * 4);
    return mag > 1.0e-12 ? amp / mag : 1.0;
}

struct TableCurve
{
    const ShapeTable* t;
    double f(double x) const { return t->fAt(x); }
    double F(double x) const { return t->FAt(x); }
    double F2(double x) const { return t->F2At(x); }
};

// The same curve for the block kernels of the second order, read through the packed cells of the table.
struct FastTableCurve
{
    const ShapeTable* t;
    CRUCIBLE_ALWAYS_INLINE double f(double x) const
    {
        double u;
        const ShapeTable::Node& n = t->locate(x, u);
        return n.f + n.slope * u;
    }
    CRUCIBLE_ALWAYS_INLINE double F(double x) const
    {
        double u;
        const ShapeTable::Node& n = t->locate(x, u);
        return n.F + u * (n.f + 0.5 * n.slope * u);
    }
    CRUCIBLE_ALWAYS_INLINE double F2(double x) const
    {
        double u;
        const ShapeTable::Node& n = t->locate(x, u);
        return n.F2 + u * (n.F + u * (0.5 * n.f + u * n.slope * (1.0 / 6.0)));
    }
};

template <class C> struct StaticCurve
{
    CRUCIBLE_ALWAYS_INLINE double f(double x) const { return C::f(x); }
    CRUCIBLE_ALWAYS_INLINE double F(double x) const { return C::F(x); }
    CRUCIBLE_ALWAYS_INLINE double F2(double x) const { return C::F2(x); }
};
} // namespace

// ------------------------------------------------------------------------------------------------------------
double ShapeTable::fAt(double x) const
{
    if (x >= range)
        return f.back();
    if (x <= -range)
        return f.front();
    const double u = (x + range) / step;
    const auto i = static_cast<size_t>(u);
    return f[i] + (f[i + 1] - f[i]) * (u - static_cast<double>(i));
}

double ShapeTable::FAt(double x) const
{
    if (x >= range)
        return F.back() + f.back() * (x - range);
    if (x <= -range)
        return F.front() + f.front() * (x + range);
    const double u = (x + range) / step;
    const auto i = static_cast<size_t>(u);
    const double t = (u - static_cast<double>(i)) * step;
    const double slope = (f[i + 1] - f[i]) / step;
    return F[i] + f[i] * t + 0.5 * slope * t * t;
}

double ShapeTable::F2At(double x) const
{
    if (x >= range)
    {
        const double d = x - range;
        return F2.back() + F.back() * d + 0.5 * f.back() * d * d;
    }
    if (x <= -range)
    {
        const double d = x + range;
        return F2.front() + F.front() * d + 0.5 * f.front() * d * d;
    }
    const double u = (x + range) / step;
    const auto i = static_cast<size_t>(u);
    const double t = (u - static_cast<double>(i)) * step;
    const double slope = (f[i + 1] - f[i]) / step;
    return F2[i] + t * (F[i] + t * (0.5 * f[i] + t * slope / 6.0));
}

const std::string& modelName(Model m)
{
    return modelNames()[static_cast<size_t>(m)];
}

const std::vector<std::string>& modelNames()
{
    static const std::vector<std::string> names = {"Soft",   "Hard",      "Rail",    "Diode", "Foldback",
                                                   "Triode", "Push-Pull", "Pentode", "Tape",  "Transformer",
                                                   "RectHalf", "RectFull", "SineFold", "LinFold", "ZeroSq", "Asym"};
    return names;
}

int recommendedOversampling(Model m, Quality q, bool shaperNode)
{
    const int qi = q == Quality::Draft ? 0 : (q == Quality::Standard ? 1 : 2);
    if (shaperNode)
    {
        static constexpr int node[3] = {2, 4, 8};
        return node[qi];
    }
    // The transformer saturates the flux, which is band-limited by construction, so it needs half of the others.
    if (m == Model::Transformer)
    {
        static constexpr int flux[3] = {1, 2, 4};
        return flux[qi];
    }
    // The hysteresis is the most expensive kernel; the plan gives it 2x / 2x / 4x.
    if (m == Model::Tape)
    {
        static constexpr int tape[3] = {2, 2, 4};
        return tape[qi];
    }
    static constexpr int other[3] = {2, 4, 8};
    return other[qi];
}

// ------------------------------------------------------------------------------------------------------------
thread_local long gPrepareCalls = 0;

long Saturator::prepareCalls()
{
    return gPrepareCalls;
}

void Saturator::prepare(double rate)
{
    ++gPrepareCalls;
    sr = rate;
    (void)tubeTables();
    (void)curveTables();
    tapeCalibrated = false;
    driveCoef = 1.0 - std::exp(-1.0 / (0.005 * sr));
    sagAtk = 1.0 - std::exp(-1.0 / (0.003 * sr));
    sagRel = 1.0 - std::exp(-1.0 / (0.12 * sr));
    set(cfg);
    driveGain = driveTarget;
    reset();
}

void Saturator::reset()
{
    xPrev = FPrev = 0.0;
    xPrev2 = F2Prev = divPrev = 0.0;
    history = 0;
    havePrev = false;
    dc = Dc{};
    sagEnv = 0.0;
    tapeM = tapeHprev = 0.0;
    fluxState = 0.0;
}

void Saturator::set(const Settings& s)
{
    const bool modelChanged = s.model != cfg.model;
    const bool tapeChanged =
        s.model == Model::Tape && (modelChanged || !tapeCalibrated || std::abs(s.hysteresis - cfg.hysteresis) > 1.0e-6f ||
                                   s.integrator != cfg.integrator || s.tapeFast != cfg.tapeFast);
    cfg = s;
    cfg.driveDb = std::clamp(cfg.driveDb, 0.0f, 48.0f);
    cfg.bias = std::clamp(cfg.bias, -1.0f, 1.0f);
    cfg.sag = std::clamp(cfg.sag, 0.0f, 1.0f);
    driveTarget = std::pow(10.0, cfg.driveDb / 20.0);
    biasOffset = 0.5 * cfg.bias;
    dcG = cfg.dcCornerHz > 0.0f ? std::tan(kPiD * std::min<double>(cfg.dcCornerHz, 0.45 * sr) / sr) : 0.0;
    fluxG = std::tan(kPiD * std::clamp<double>(cfg.cornerHz, 0.5, 0.45 * sr) / sr);
    switch (cfg.model)
    {
    case Model::TriodeSE:
        table = &tubeTables().triode;
        break;
    case Model::TriodePP:
        table = &tubeTables().pushPull;
        break;
    case Model::Pentode:
        table = &tubeTables().pentode;
        break;
    default:
        table = nullptr;
        break;
    }
    if (tapeChanged)
    {
        ja = jaParams(cfg.hysteresis);
        tapeNorm = cfg.tapeFast ? calibrateTapeFast(cfg.hysteresis) : calibrateTape(cfg.hysteresis, cfg.integrator);
        tapeCalibrated = true;
    }
    if (modelChanged)
        reset();
}

void Saturator::adoptState(const Saturator& o)
{
    driveGain = o.driveGain;
    sagEnv = o.sagEnv;
    dc.s = o.dc.s;
    tapeM = o.tapeM;
    tapeHprev = o.tapeHprev;
    fluxState = o.fluxState;
    xPrev = xPrev2 = FPrev = F2Prev = divPrev = 0.0;
    history = 0;
    havePrev = false;
}

float Saturator::latencySamples() const
{
    const bool memoryless = cfg.model != Model::Tape && cfg.model != Model::Transformer;
    if (!memoryless || !cfg.adaa)
        return 0.0f;
    return cfg.adaaOrder >= 2 ? 1.0f : 0.5f;
}

template <class Curve> double Saturator::adaaStep(double x, const Curve& c)
{
    const double F1 = c.F(x);
    double y;
    if (!havePrev)
    {
        havePrev = true;
        y = c.f(x);
    }
    else
    {
        const double dx = x - xPrev;
        y = std::abs(dx) > kAdaaEpsilon ? (F1 - FPrev) / dx : c.f(0.5 * (x + xPrev));
    }
    xPrev = x;
    FPrev = F1;
    return y;
}

// Second-order ADAA: y = 2 F2[x0, x1, x2], the second divided difference of the second antiderivative F2, which is the
// curve smoothed by two cascaded one-sample boxes (for a linear curve y is the mean of the three samples); the output
// is delayed by one sample. Where samples nearly coincide the divided differences are replaced by their expansions,
// good to the square of kEps (Bilbao et al. give the limits; the correction d (f0 - f1) / 24 is the next term). The
// regular case needs one division: with p = d01 d02 the two reciprocals are d02 / p and d01 / p.
struct Adaa2State
{
    double xPrev = 0.0, xPrev2 = 0.0, F2Prev = 0.0, divPrev = 0.0;
    int history = 0;
    // The last f(x) the expansions looked up: a run of nearly equal samples asks for f at the previous sample next.
    double fAtX = std::numeric_limits<double>::quiet_NaN(), fValue = 0.0;
};

template <class Curve> CRUCIBLE_ALWAYS_INLINE double adaa2Update(Adaa2State& st, double x, const Curve& t)
{
    constexpr double eps = 1.0e-3;
    const double g0 = t.F2(x);
    // F2[a, b] = mean of F1 over [a, b]
    const auto firstDifference = [&](double xa, double ga, double xb, double gb)
    {
        const double d = xa - xb;
        if (std::abs(d) > eps)
            return (ga - gb) / d;
        const double fa = t.f(xa);
        const double fb = xb == st.fAtX ? st.fValue : t.f(xb);
        st.fAtX = xa;
        st.fValue = fa;
        return t.F(0.5 * (xa + xb)) + d * (fa - fb) * (1.0 / 24.0);
    };
    double y;
    if (st.history >= 2)
    {
        const double d01 = x - st.xPrev, d02 = x - st.xPrev2;
        const double dg = g0 - st.F2Prev;
        double a;
        if (std::abs(d01) > eps && std::abs(d02) > eps) [[likely]]
        {
            const double inv = 1.0 / (d01 * d02);
            a = dg * d02 * inv;
            y = 2.0 * (a - st.divPrev) * d01 * inv;
        }
        else
        {
            a = firstDifference(x, g0, st.xPrev, st.F2Prev); // F2[x0, x1]
            if (std::abs(d02) > eps)
            {
                y = 2.0 * (a - st.divPrev) / d02;
            }
            else
            {
                const double xb = 0.5 * (x + st.xPrev2), d = xb - st.xPrev;
                if (std::abs(d) > eps)
                    y = 2.0 * (t.F(xb) - (t.F2(xb) - st.F2Prev) / d) / d;
                else
                    y = t.f((x + st.xPrev + st.xPrev2) * (1.0 / 3.0));
            }
        }
        st.divPrev = a;
    }
    else if (st.history == 0)
    {
        y = t.f(x);
        st.history = 1;
    }
    else
    {
        y = t.f(0.5 * (x + st.xPrev));
        st.divPrev = firstDifference(x, g0, st.xPrev, st.F2Prev);
        st.history = 2;
    }
    st.xPrev2 = st.xPrev;
    st.xPrev = x;
    st.F2Prev = g0;
    return y;
}

template <class Curve> double Saturator::adaa2Step(double x, const Curve& t)
{
    Adaa2State st{xPrev, xPrev2, F2Prev, divPrev, history};
    const double y = adaa2Update(st, x, t);
    xPrev = st.xPrev;
    xPrev2 = st.xPrev2;
    F2Prev = st.F2Prev;
    divPrev = st.divPrev;
    history = st.history;
    return y;
}

// The block form of process() for the second-order models, in two passes over tiles: the first makes the driven,
// biased (and, for the tubes, sagged) input, the second runs the antiderivative step. The sag follows the level of the
// curve at the input before the shift, which keeps the feedback of the envelope off the long chain of the second pass.
// The recursive state lives in locals for the length of a tile.
constexpr int kAdaaTile = 128;

template <class Curve> void Saturator::adaa2Drive(const Curve& c, const float* in, double* xs, int len, bool withSag)
{
    double gain = driveGain;
    const double target = driveTarget, coef = driveCoef, bias = biasOffset;
    // Once the drive glide has settled (to 1e-12) the gain is a constant and the pass has no recursion left.
    if (std::abs(target - gain) <= 1.0e-12 * target)
    {
        gain = target;
        for (int i = 0; i < len; ++i)
            xs[i] = static_cast<double>(in[i]) * gain + bias;
    }
    else
    {
        for (int i = 0; i < len; ++i)
        {
            gain += (target - gain) * coef;
            xs[i] = static_cast<double>(in[i]) * gain + bias;
        }
    }
    driveGain = gain;
    if (withSag && hasSag())
        adaa2Sag(c, xs, len);
}

// The sag follows the level of the curve at the input before the shift (attack 3 ms, release 120 ms) and shifts the
// input by it. Taking the level before the shift lets the envelope run as a recursion of its own over the tile.
template <class Curve> void Saturator::adaa2Sag(const Curve& c, double* xs, int len)
{
    const double sagDepth = 0.6 * static_cast<double>(cfg.sag);
    double level[kAdaaTile];
    for (int i = 0; i < len; ++i)
        level[i] = std::abs(c.f(xs[i]));
    double env = sagEnv;
    for (int i = 0; i < len; ++i)
    {
        const double a = level[i];
        xs[i] -= sagDepth * env;
        env += (a - env) * (a > env ? sagAtk : sagRel);
    }
    sagEnv = env;
}

template <class Curve> void Saturator::adaa2Run(const Curve& c, const double* xs, float* out, int len)
{
    // The same one-pole high-pass as Dc::step in a form whose state update is a single fused multiply-add.
    const bool blockDc = dcG > 0.0;
    const double k = dcG / (1.0 + dcG), dcKeep = 1.0 - 2.0 * k, dcIn = 2.0 * k, dcOut = 1.0 - k;
    Adaa2State st{xPrev, xPrev2, F2Prev, divPrev, history};
    double hp = dc.s;
    for (int i = 0; i < len; ++i)
    {
        double y = adaa2Update(st, xs[i], c);
        if (blockDc)
        {
            const double s = hp;
            hp = s * dcKeep + dcIn * y;
            y = dcOut * (y - s);
        }
        out[i] = static_cast<float>(y);
    }
    xPrev = st.xPrev;
    xPrev2 = st.xPrev2;
    F2Prev = st.F2Prev;
    divPrev = st.divPrev;
    history = st.history;
    dc.s = hp;
}

template <class Curve> void Saturator::adaa2Block(const Curve& c, const float* in, float* out, int n)
{
    double xs[kAdaaTile];
    for (int base = 0; base < n; base += kAdaaTile)
    {
        const int len = std::min(kAdaaTile, n - base);
        adaa2Drive(c, in + base, xs, len);
        adaa2Run(c, xs, out + base, len);
    }
}

// The transformer in the same two-pass form: the flux recursion is short, so the passes are cheap.
void Saturator::transformerBlock(const float* in, float* out, int n)
{
    const bool blockDc = dcG > 0.0;
    const double dk = dcG / (1.0 + dcG), dcKeep = 1.0 - 2.0 * dk, dcIn = 2.0 * dk, dcOut = 1.0 - dk;
    const double fk = fluxG / (1.0 + fluxG);
    double xs[kAdaaTile];
    double s = fluxState, hp = dc.s;
    for (int base = 0; base < n; base += kAdaaTile)
    {
        const int len = std::min(kAdaaTile, n - base);
        adaa2Drive(StaticCurve<HardCurve>{}, in + base, xs, len);
        for (int i = 0; i < len; ++i)
        {
            const double x = xs[i];
            const double v = (x - s) * fk;
            const double flux = v + s;
            s = flux + v;
            const double t = fastTanh(flux);
            double y = (1.0 - t * t) * (x - flux);
            if (blockDc)
            {
                const double h = hp;
                hp = h * dcKeep + dcIn * y;
                y = dcOut * (y - h);
            }
            out[base + i] = static_cast<float>(y);
        }
    }
    fluxState = s;
    dc.s = hp;
}

#if CRUCIBLE_SIMD_NEON && defined(__aarch64__)
// The second antiderivative of the curve in the two lanes of a float64x2: the tabulated curves read the packed cells of
// the table lane by lane, the wavefolder evaluates its piecewise cubic.
struct TablePairF2
{
    const ShapeTable* t;
    float64x2_t range, invStep, step, top, zero, half, sixth;
    size_t lowNode, highNode;
    const ShapeTable::Node* nodes;
    explicit TablePairF2(const ShapeTable& tab)
        : t(&tab), range(vdupq_n_f64(tab.range)), invStep(vdupq_n_f64(tab.invStep)), step(vdupq_n_f64(tab.step)),
          top(vdupq_n_f64(static_cast<double>(tab.cells))), zero(vdupq_n_f64(0.0)), half(vdupq_n_f64(0.5)),
          sixth(vdupq_n_f64(1.0 / 6.0)), lowNode(static_cast<size_t>(tab.cells) + 1),
          highNode(static_cast<size_t>(tab.cells) + 2), nodes(tab.node.data())
    {
    }
    // The curve itself in both lanes (the level the sag follows).
    float64x2_t f(float64x2_t x) const
    {
        const float64x2_t u = vmulq_f64(vaddq_f64(x, range), invStep);
        const uint64x2_t low = vcleq_f64(u, zero), high = vcgeq_f64(u, top);
        const float64x2_t fl = vrndmq_f64(u);
        const float64x2_t inside = vmulq_f64(vsubq_f64(u, fl), step);
        const float64x2_t tv = vbslq_f64(low, vaddq_f64(x, range), vbslq_f64(high, vsubq_f64(x, range), inside));
        const size_t i0 = vgetq_lane_u64(low, 0)
                              ? lowNode
                              : (vgetq_lane_u64(high, 0) ? highNode : static_cast<size_t>(vgetq_lane_f64(fl, 0)));
        const size_t i1 = vgetq_lane_u64(low, 1)
                              ? lowNode
                              : (vgetq_lane_u64(high, 1) ? highNode : static_cast<size_t>(vgetq_lane_f64(fl, 1)));
        const float64x2_t n0 = vld1q_f64(&nodes[i0].f), n1 = vld1q_f64(&nodes[i1].f);
        const float64x2_t s0 = vld1q_f64(&nodes[i0].F2), s1 = vld1q_f64(&nodes[i1].F2);
        return vfmaq_f64(vzip1q_f64(n0, n1), tv, vzip2q_f64(s0, s1)); // f + t slope
    }
    // FastTableCurve::F: n.F + u * (n.f + 0.5 * n.slope * u)
    float64x2_t F(float64x2_t x) const
    {
        const float64x2_t u = vmulq_f64(vaddq_f64(x, range), invStep);
        const uint64x2_t low = vcleq_f64(u, zero), high = vcgeq_f64(u, top);
        const float64x2_t fl = vrndmq_f64(u);
        const float64x2_t inside = vmulq_f64(vsubq_f64(u, fl), step);
        const float64x2_t tv = vbslq_f64(low, vaddq_f64(x, range), vbslq_f64(high, vsubq_f64(x, range), inside));
        const size_t i0 = vgetq_lane_u64(low, 0)
                              ? lowNode
                              : (vgetq_lane_u64(high, 0) ? highNode : static_cast<size_t>(vgetq_lane_f64(fl, 0)));
        const size_t i1 = vgetq_lane_u64(low, 1)
                              ? lowNode
                              : (vgetq_lane_u64(high, 1) ? highNode : static_cast<size_t>(vgetq_lane_f64(fl, 1)));
        const float64x2_t n0a = vld1q_f64(&nodes[i0].f), n0b = vld1q_f64(&nodes[i0].F2);
        const float64x2_t n1a = vld1q_f64(&nodes[i1].f), n1b = vld1q_f64(&nodes[i1].F2);
        const float64x2_t f = vzip1q_f64(n0a, n1a), F = vzip2q_f64(n0a, n1a), slope = vzip2q_f64(n0b, n1b);
        const float64x2_t inner = vfmaq_f64(f, vmulq_f64(half, slope), tv);
        return vfmaq_f64(F, tv, inner);
    }
    float64x2_t g2(float64x2_t x) const
    {
        const float64x2_t u = vmulq_f64(vaddq_f64(x, range), invStep);
        const uint64x2_t low = vcleq_f64(u, zero), high = vcgeq_f64(u, top);
        const float64x2_t fl = vrndmq_f64(u);
        const float64x2_t inside = vmulq_f64(vsubq_f64(u, fl), step);
        const float64x2_t tv = vbslq_f64(low, vaddq_f64(x, range), vbslq_f64(high, vsubq_f64(x, range), inside));
        // node of each lane: the cell index, or an extension node
        const size_t i0 = vgetq_lane_u64(low, 0)
                              ? lowNode
                              : (vgetq_lane_u64(high, 0) ? highNode : static_cast<size_t>(vgetq_lane_f64(fl, 0)));
        const size_t i1 = vgetq_lane_u64(low, 1)
                              ? lowNode
                              : (vgetq_lane_u64(high, 1) ? highNode : static_cast<size_t>(vgetq_lane_f64(fl, 1)));
        const float64x2_t n0a = vld1q_f64(&nodes[i0].f), n0b = vld1q_f64(&nodes[i0].F2);
        const float64x2_t n1a = vld1q_f64(&nodes[i1].f), n1b = vld1q_f64(&nodes[i1].F2);
        const float64x2_t f = vzip1q_f64(n0a, n1a), F = vzip2q_f64(n0a, n1a), F2 = vzip1q_f64(n0b, n1b),
                          slope = vzip2q_f64(n0b, n1b);
        float64x2_t g0 = vfmaq_f64(vmulq_f64(half, f), vmulq_f64(tv, slope), sixth); // 0.5 f + t slope / 6
        g0 = vfmaq_f64(F, tv, g0);                                                   // F + t (...)
        return vfmaq_f64(F2, tv, g0);                                                // F2 + t (...)
    }
};

struct FoldPairF2
{
    float64x2_t one = vdupq_n_f64(1.0), quarter = vdupq_n_f64(0.25), four = vdupq_n_f64(4.0), sixth = vdupq_n_f64(1.0 / 6.0),
                half = vdupq_n_f64(0.5), two = vdupq_n_f64(2.0);
    float64x2_t g2(float64x2_t x) const
    {
        const float64x2_t k = vrndmq_f64(vmulq_f64(vaddq_f64(x, one), quarter));
        const float64x2_t t = vfmsq_f64(x, k, four); // x - 4 k, in [-1, 3)
        const float64x2_t t2 = vmulq_f64(t, t), t3 = vmulq_f64(t2, t);
        const float64x2_t rise = vmulq_f64(vaddq_f64(t3, one), sixth);                    // (t^3 + 1) / 6
        const float64x2_t fall = vsubq_f64(vfmsq_f64(vaddq_f64(half, t2), t3, sixth), t); // 1/2 + t^2 - t^3 / 6 - t
        return vfmaq_f64(vbslq_f64(vcleq_f64(t, one), rise, fall), k, two);
    }
};

inline TablePairF2 makePairF2(const FastTableCurve& c)
{
    return TablePairF2(*c.t);
}
inline FoldPairF2 makePairF2(const StaticCurve<FoldCurve>&)
{
    return FoldPairF2{};
}

// Both channels' sag in the lanes of float64x2: the level of the curve at the unshifted input, the shift by the envelope
// before the update, and the envelope recursion, one sample at a time.
template <class Curve>
void Saturator::adaa2SagPair(Saturator& a, Saturator& b, const Curve& curve, double* xs0, double* xs1, int len)
{
    const TablePairF2 ev(*curve.t);
    const float64x2_t depth = {0.6 * static_cast<double>(a.cfg.sag), 0.6 * static_cast<double>(b.cfg.sag)};
    const float64x2_t atk = {a.sagAtk, b.sagAtk}, rel = {a.sagRel, b.sagRel};
    float64x2_t env = {a.sagEnv, b.sagEnv};
    for (int i = 0; i < len; ++i)
    {
        const float64x2_t x = {xs0[i], xs1[i]};
        const float64x2_t level = vabsq_f64(ev.f(x));
        const float64x2_t shifted = vfmsq_f64(x, depth, env);
        xs0[i] = vgetq_lane_f64(shifted, 0);
        xs1[i] = vgetq_lane_f64(shifted, 1);
        env = vfmaq_f64(env, vsubq_f64(level, env), vbslq_f64(vcgtq_f64(level, env), atk, rel));
    }
    a.sagEnv = vgetq_lane_f64(env, 0);
    b.sagEnv = vgetq_lane_f64(env, 1);
}

// Two channels in the lanes of float64x2: the same arithmetic as adaa2Update, with the table gathers done lane by lane.
// A sample for which either lane needs one of the expansions (nearly equal samples) goes through the scalar update for
// both lanes.
template <class Curve>
void Saturator::adaa2PairRun(Saturator& a, Saturator& b, const Curve& curve, const double* xs0, const double* xs1,
                             float* out0, float* out1, int len)
{
    constexpr double eps = 1.0e-3;
    const auto ev = makePairF2(curve);
    Adaa2State sa{a.xPrev, a.xPrev2, a.F2Prev, a.divPrev, a.history};
    Adaa2State sb{b.xPrev, b.xPrev2, b.F2Prev, b.divPrev, b.history};
    const bool blockDc = a.dcG > 0.0;
    const double k = a.dcG / (1.0 + a.dcG);
    const float64x2_t dcKeep = vdupq_n_f64(1.0 - 2.0 * k), dcIn = vdupq_n_f64(2.0 * k), dcOut = vdupq_n_f64(1.0 - k);
    float64x2_t hp = {a.dc.s, b.dc.s};
    float64x2_t xp1 = {sa.xPrev, sb.xPrev}, xp2 = {sa.xPrev2, sb.xPrev2}, g1 = {sa.F2Prev, sb.F2Prev},
                dv = {sa.divPrev, sb.divPrev};
    const float64x2_t two = vdupq_n_f64(2.0), epsilon = vdupq_n_f64(eps);
    constexpr bool isTable = std::is_same_v<Curve, FastTableCurve>;
    // Both lanes past their first two samples with the three newest samples of each within eps of each other: the
    // case adaa2Update answers with its expansions only (the tests are the negations of its > eps tests, so NaN lands here
    // as it does there).
    const auto nearBoth = [&](int ha, int hb, float64x2_t x, float64x2_t p1, float64x2_t p2, float64x2_t e)
    {
        if (ha < 2 || hb < 2)
            return false;
        const float64x2_t xb = vmulq_f64(vdupq_n_f64(0.5), vaddq_f64(x, p2));
        const uint64x2_t far = vorrq_u64(vorrq_u64(vcagtq_f64(vsubq_f64(x, p1), e), vcagtq_f64(vsubq_f64(x, p2), e)),
                                         vcagtq_f64(vsubq_f64(xb, p1), e));
        return (vgetq_lane_u64(far, 0) | vgetq_lane_u64(far, 1)) == 0;
    };
    for (int i = 0; i < len; ++i)
    {
        const float64x2_t x = {xs0[i], xs1[i]};
        const float64x2_t g0 = ev.g2(x);
        const float64x2_t d01 = vsubq_f64(x, xp1), d02 = vsubq_f64(x, xp2);
        const uint64x2_t ok = vandq_u64(vcagtq_f64(d01, epsilon), vcagtq_f64(d02, epsilon));
        double y0, y1;
        if (sa.history >= 2 && sb.history >= 2 && (vgetq_lane_u64(ok, 0) & vgetq_lane_u64(ok, 1)))
        {
            const float64x2_t inv = vdivq_f64(vdupq_n_f64(1.0), vmulq_f64(d01, d02));
            const float64x2_t dg = vsubq_f64(g0, g1);
            const float64x2_t av = vmulq_f64(vmulq_f64(dg, d02), inv);
            const float64x2_t yv = vmulq_f64(vmulq_f64(vmulq_f64(two, vsubq_f64(av, dv)), d01), inv);
            dv = av;
            xp2 = xp1;
            xp1 = x;
            g1 = g0;
            if (blockDc)
            {
                const float64x2_t s = hp;
                hp = vfmaq_f64(vmulq_f64(dcIn, yv), s, dcKeep);
                const float64x2_t yo = vmulq_f64(dcOut, vsubq_f64(yv, s));
                y0 = vgetq_lane_f64(yo, 0);
                y1 = vgetq_lane_f64(yo, 1);
            }
            else
            {
                y0 = vgetq_lane_f64(yv, 0);
                y1 = vgetq_lane_f64(yv, 1);
            }
        }
        else if (isTable && nearBoth(sa.history, sb.history, x, xp1, xp2, epsilon))
        {
          if constexpr (isTable)
          {
            // All three samples of both lanes within eps of each other (a quiet band): the expansions of adaa2Update,
            // in the lanes, with the same operations per lane.
            const float64x2_t fx = ev.f(x);
            const float64x2_t fb = ev.f(xp1);
            const float64x2_t d01 = vsubq_f64(x, xp1);
            const float64x2_t mid = vmulq_f64(vdupq_n_f64(0.5), vaddq_f64(x, xp1));
            const float64x2_t av = vfmaq_f64(ev.F(mid), vmulq_f64(d01, vsubq_f64(fx, fb)), vdupq_n_f64(1.0 / 24.0));
            const float64x2_t mean = vmulq_f64(vaddq_f64(vaddq_f64(x, xp1), xp2), vdupq_n_f64(1.0 / 3.0));
            const float64x2_t yv = ev.f(mean);
            dv = av;
            xp2 = xp1;
            xp1 = x;
            g1 = g0;
            if (blockDc)
            {
                const float64x2_t s = hp;
                hp = vfmaq_f64(vmulq_f64(dcIn, yv), s, dcKeep);
                const float64x2_t yo = vmulq_f64(dcOut, vsubq_f64(yv, s));
                y0 = vgetq_lane_f64(yo, 0);
                y1 = vgetq_lane_f64(yo, 1);
            }
            else
            {
                y0 = vgetq_lane_f64(yv, 0);
                y1 = vgetq_lane_f64(yv, 1);
            }
          }
        }
        else
        {
            sa.xPrev = vgetq_lane_f64(xp1, 0);
            sb.xPrev = vgetq_lane_f64(xp1, 1);
            sa.xPrev2 = vgetq_lane_f64(xp2, 0);
            sb.xPrev2 = vgetq_lane_f64(xp2, 1);
            sa.F2Prev = vgetq_lane_f64(g1, 0);
            sb.F2Prev = vgetq_lane_f64(g1, 1);
            sa.divPrev = vgetq_lane_f64(dv, 0);
            sb.divPrev = vgetq_lane_f64(dv, 1);
            y0 = adaa2Update(sa, xs0[i], curve);
            y1 = adaa2Update(sb, xs1[i], curve);
            xp1 = float64x2_t{sa.xPrev, sb.xPrev};
            xp2 = float64x2_t{sa.xPrev2, sb.xPrev2};
            g1 = float64x2_t{sa.F2Prev, sb.F2Prev};
            dv = float64x2_t{sa.divPrev, sb.divPrev};
            if (blockDc)
            {
                const float64x2_t yv = {y0, y1};
                const float64x2_t s = hp;
                hp = vfmaq_f64(vmulq_f64(dcIn, yv), s, dcKeep);
                const float64x2_t yo = vmulq_f64(dcOut, vsubq_f64(yv, s));
                y0 = vgetq_lane_f64(yo, 0);
                y1 = vgetq_lane_f64(yo, 1);
            }
        }
        out0[i] = static_cast<float>(y0);
        out1[i] = static_cast<float>(y1);
    }
    a.xPrev = vgetq_lane_f64(xp1, 0);
    b.xPrev = vgetq_lane_f64(xp1, 1);
    a.xPrev2 = vgetq_lane_f64(xp2, 0);
    b.xPrev2 = vgetq_lane_f64(xp2, 1);
    a.F2Prev = vgetq_lane_f64(g1, 0);
    b.F2Prev = vgetq_lane_f64(g1, 1);
    a.divPrev = vgetq_lane_f64(dv, 0);
    b.divPrev = vgetq_lane_f64(dv, 1);
    a.history = sa.history;
    b.history = sb.history;
    a.dc.s = vgetq_lane_f64(hp, 0);
    b.dc.s = vgetq_lane_f64(hp, 1);
}
#elif CRUCIBLE_SIMD_SSE_EXACT
#if CRUCIBLE_SATURATION_AVX
// AVX in the CPU and its register state saved by the OS (CPUID.1:ECX bits 27 and 28, XCR0 bits 1 and 2), checked once.
namespace
{
bool cpuHasAvx()
{
    int regs[4];
    __cpuid(regs, 1);
    const bool osSaves = (regs[2] & (1 << 27)) != 0, avxBit = (regs[2] & (1 << 28)) != 0;
    return osSaves && avxBit && (_xgetbv(0) & 6U) == 6U;
}
const bool kSaturationAvx = cpuHasAvx();
} // namespace
#endif
// x86 without FMA: the two channels in the lanes of an __m128d with exactly the scalar operations of adaa2Sag and
// adaa2Run per lane (a multiply and an add where the scalar code has them, no contraction), so the output is that of
// the two scalar runs bit for bit; the two independent chains of dependent operations overlap.
namespace
{
CRUCIBLE_ALWAYS_INLINE __m128d selectPd(__m128d mask, __m128d a, __m128d b)
{
    return _mm_or_pd(_mm_and_pd(mask, a), _mm_andnot_pd(mask, b));
}

// ShapeTable::locate in both lanes: the node of each lane and the offset in the cell.
struct TablePairSse
{
    const ShapeTable::Node* nodes;
    __m128d range, invStep, step, top, zero;
    int lowNode, highNode;
    explicit TablePairSse(const ShapeTable& t)
        : nodes(t.node.data()), range(_mm_set1_pd(t.range)), invStep(_mm_set1_pd(t.invStep)), step(_mm_set1_pd(t.step)),
          top(_mm_set1_pd(static_cast<double>(t.cells))), zero(_mm_setzero_pd()), lowNode(t.cells + 1), highNode(t.cells + 2)
    {
    }
    // locate(): u = (x + range) / step; inside (0 < u < cells) the cell is trunc(u) and the offset (u - cell) step;
    // below (u <= 0) and above (u >= cells) the extension nodes with x + range and x - range; otherwise (NaN) cell 0.
    CRUCIBLE_ALWAYS_INLINE __m128d locate(__m128d x, const ShapeTable::Node*& n0, const ShapeTable::Node*& n1) const
    {
        const __m128d xr = _mm_add_pd(x, range);
        const __m128d u = _mm_mul_pd(xr, invStep);
        const __m128d low = _mm_cmple_pd(u, zero), high = _mm_cmpge_pd(u, top);
        const __m128d inside = _mm_and_pd(_mm_cmpgt_pd(u, zero), _mm_cmplt_pd(u, top));
        const __m128i cell = _mm_and_si128(
            _mm_cvttpd_epi32(u),
            _mm_castps_si128(_mm_shuffle_ps(_mm_castpd_ps(inside), _mm_castpd_ps(inside), _MM_SHUFFLE(3, 3, 2, 0))));
        const __m128d offset = _mm_mul_pd(_mm_sub_pd(u, _mm_cvtepi32_pd(cell)), step);
        const __m128d t = selectPd(low, xr, selectPd(high, _mm_sub_pd(x, range), offset));
        const int lowMask = _mm_movemask_pd(low), highMask = _mm_movemask_pd(high);
        const int c0 = _mm_cvtsi128_si32(cell), c1 = _mm_cvtsi128_si32(_mm_shuffle_epi32(cell, _MM_SHUFFLE(1, 1, 1, 1)));
        n0 = nodes + ((lowMask & 1) ? lowNode : ((highMask & 1) ? highNode : c0));
        n1 = nodes + ((lowMask & 2) ? lowNode : ((highMask & 2) ? highNode : c1));
        return t;
    }
    // FastTableCurve::f: n.f + n.slope * u
    CRUCIBLE_ALWAYS_INLINE __m128d f(__m128d x) const
    {
        const ShapeTable::Node *n0, *n1;
        const __m128d u = locate(x, n0, n1);
        const __m128d a = _mm_loadu_pd(&n0->f), b = _mm_loadu_pd(&n1->f);   // f, F
        const __m128d c = _mm_loadu_pd(&n0->F2), d = _mm_loadu_pd(&n1->F2); // F2, slope
        return _mm_add_pd(_mm_unpacklo_pd(a, b), _mm_mul_pd(_mm_unpackhi_pd(c, d), u));
    }
    // FastTableCurve::F: n.F + u * (n.f + 0.5 * n.slope * u)
    CRUCIBLE_ALWAYS_INLINE __m128d F(__m128d x) const
    {
        const ShapeTable::Node *n0, *n1;
        const __m128d u = locate(x, n0, n1);
        const __m128d a = _mm_loadu_pd(&n0->f), b = _mm_loadu_pd(&n1->f);
        const __m128d c = _mm_loadu_pd(&n0->F2), d = _mm_loadu_pd(&n1->F2);
        const __m128d nf = _mm_unpacklo_pd(a, b), nF = _mm_unpackhi_pd(a, b), slope = _mm_unpackhi_pd(c, d);
        const __m128d inner = _mm_add_pd(nf, _mm_mul_pd(_mm_mul_pd(_mm_set1_pd(0.5), slope), u));
        return _mm_add_pd(nF, _mm_mul_pd(u, inner));
    }
    // FastTableCurve::F2: n.F2 + u * (n.F + u * (0.5 * n.f + u * n.slope * (1 / 6)))
    CRUCIBLE_ALWAYS_INLINE __m128d g2(__m128d x) const
    {
        const ShapeTable::Node *n0, *n1;
        const __m128d u = locate(x, n0, n1);
        const __m128d a = _mm_loadu_pd(&n0->f), b = _mm_loadu_pd(&n1->f);
        const __m128d c = _mm_loadu_pd(&n0->F2), d = _mm_loadu_pd(&n1->F2);
        const __m128d nf = _mm_unpacklo_pd(a, b), nF = _mm_unpackhi_pd(a, b), nF2 = _mm_unpacklo_pd(c, d),
                      slope = _mm_unpackhi_pd(c, d);
        __m128d g = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.5), nf), _mm_mul_pd(_mm_mul_pd(u, slope), _mm_set1_pd(1.0 / 6.0)));
        g = _mm_add_pd(nF, _mm_mul_pd(u, g));
        return _mm_add_pd(nF2, _mm_mul_pd(u, g));
    }
};

// Curves without a table (the wavefolder): F2 lane by lane through the scalar function.
template <class Curve> struct ScalarPairSse
{
    Curve c;
    CRUCIBLE_ALWAYS_INLINE __m128d g2(__m128d x) const
    {
        alignas(16) double v[2];
        _mm_store_pd(v, x);
        return _mm_set_pd(c.F2(v[1]), c.F2(v[0]));
    }
};

inline TablePairSse makePairSse(const FastTableCurve& c)
{
    return TablePairSse(*c.t);
}
template <class C> inline ScalarPairSse<StaticCurve<C>> makePairSse(const StaticCurve<C>& c)
{
    return ScalarPairSse<StaticCurve<C>>{c};
}
} // namespace

// adaa2Sag for both channels: the level of the curve at the unshifted input for the whole tile, then the shift by the
// envelope before its update and the envelope recursion, x - depth env and env + (a - env) (a > env ? attack : release).
template <class Curve>
void Saturator::adaa2SagPair(Saturator& a, Saturator& b, const Curve& curve, double* xs0, double* xs1, int len)
{
    // One loop: the level does not depend on the envelope, so its look-up overlaps the envelope's chain.
    const TablePairSse ev(*curve.t);
    const __m128d signMask = _mm_set1_pd(-0.0);
    const __m128d depth = _mm_set_pd(0.6 * static_cast<double>(b.cfg.sag), 0.6 * static_cast<double>(a.cfg.sag));
    const __m128d atk = _mm_set_pd(b.sagAtk, a.sagAtk), rel = _mm_set_pd(b.sagRel, a.sagRel);
    __m128d env = _mm_set_pd(b.sagEnv, a.sagEnv);
#if CRUCIBLE_SATURATION_AVX
    // On CPUs with AVX the level of two samples at a time (SaturationAvx.h), the same operations.
    if (kSaturationAvx)
    {
        avx::SagLanes sl;
        _mm_storeu_pd(sl.depth, depth);
        _mm_storeu_pd(sl.atk, atk);
        _mm_storeu_pd(sl.rel, rel);
        _mm_storeu_pd(sl.env, env);
        const avx::TableView view{reinterpret_cast<const double*>(curve.t->node.data()), curve.t->range, curve.t->invStep,
                                  curve.t->step, curve.t->cells};
        avx::sagPairTable(view, sl, xs0, xs1, len);
        a.sagEnv = sl.env[0];
        b.sagEnv = sl.env[1];
        return;
    }
#endif
    for (int i = 0; i < len; ++i)
    {
        const __m128d x = _mm_set_pd(xs1[i], xs0[i]);
        const __m128d lv = _mm_andnot_pd(signMask, ev.f(x));
        const __m128d shifted = _mm_sub_pd(x, _mm_mul_pd(depth, env));
        _mm_storel_pd(xs0 + i, shifted);
        _mm_storeh_pd(xs1 + i, shifted);
        env = _mm_add_pd(env, _mm_mul_pd(_mm_sub_pd(lv, env), selectPd(_mm_cmpgt_pd(lv, env), atk, rel)));
    }
    _mm_storel_pd(&a.sagEnv, env);
    _mm_storeh_pd(&b.sagEnv, env);
}

// adaa2Run for both channels: the regular step (both lanes past their first two samples and away from the
// expansions) runs in the lanes; any other sample goes through adaa2Update lane by lane, from the same state.
template <class Curve>
void Saturator::adaa2PairRun(Saturator& a, Saturator& b, const Curve& curve, const double* xs0, const double* xs1,
                             float* out0, float* out1, int len)
{
    if (a.dcG != b.dcG)
    {
        a.adaa2Run(curve, xs0, out0, len);
        b.adaa2Run(curve, xs1, out1, len);
        return;
    }
    constexpr double eps = 1.0e-3;
    const auto ev = makePairSse(curve);
    Adaa2State sa{a.xPrev, a.xPrev2, a.F2Prev, a.divPrev, a.history};
    Adaa2State sb{b.xPrev, b.xPrev2, b.F2Prev, b.divPrev, b.history};
    const bool blockDc = a.dcG > 0.0;
    const double k = a.dcG / (1.0 + a.dcG), keep = 1.0 - 2.0 * k, in = 2.0 * k, outGain = 1.0 - k;
    const __m128d dcKeep = _mm_set1_pd(keep), dcIn = _mm_set1_pd(in), dcOut = _mm_set1_pd(outGain);
    __m128d hp = _mm_set_pd(b.dc.s, a.dc.s);
    __m128d xp1 = _mm_set_pd(sb.xPrev, sa.xPrev), xp2 = _mm_set_pd(sb.xPrev2, sa.xPrev2),
            g1 = _mm_set_pd(sb.F2Prev, sa.F2Prev), dv = _mm_set_pd(sb.divPrev, sa.divPrev);
    const __m128d epsilon = _mm_set1_pd(eps), signMask = _mm_set1_pd(-0.0), one = _mm_set1_pd(1.0), two = _mm_set1_pd(2.0);
    bool settled = sa.history >= 2 && sb.history >= 2;
    constexpr bool isTable = std::is_same_v<Curve, FastTableCurve>;
    // The three newest samples of both lanes within eps of each other: what adaa2Update answers with its expansions only
    // (the tests are the negations of its > eps tests, so NaN lands here as it does there).
    const auto nearBoth = [&](__m128d x, __m128d p1, __m128d p2)
    {
        const __m128d xb = _mm_mul_pd(_mm_set1_pd(0.5), _mm_add_pd(x, p2));
        const __m128d far = _mm_or_pd(_mm_or_pd(_mm_cmpgt_pd(_mm_andnot_pd(signMask, _mm_sub_pd(x, p1)), epsilon),
                                                _mm_cmpgt_pd(_mm_andnot_pd(signMask, _mm_sub_pd(x, p2)), epsilon)),
                                      _mm_cmpgt_pd(_mm_andnot_pd(signMask, _mm_sub_pd(xb, p1)), epsilon));
        return _mm_movemask_pd(far) == 0;
    };
#if CRUCIBLE_SATURATION_AVX
    // On CPUs with AVX, runs of regular samples go two at a time through the four-lane kernel (SaturationAvx.h, the same
    // operations per lane); the pair of samples where it stops comes here, then it takes over again.
    constexpr bool avxCurve = std::is_same_v<Curve, FastTableCurve> || std::is_same_v<Curve, StaticCurve<FoldCurve>>;
    const avx::DcCoefficients dcAvx{blockDc, keep, in, outGain};
    int avxResume = 0;
#endif
    for (int i = 0; i < len; ++i)
    {
#if CRUCIBLE_SATURATION_AVX
        if constexpr (avxCurve)
            if (kSaturationAvx && settled && i >= avxResume && i + 2 <= len)
            {
                avx::PairState ps;
                _mm_storeu_pd(ps.xPrev, xp1);
                _mm_storeu_pd(ps.xPrev2, xp2);
                _mm_storeu_pd(ps.F2Prev, g1);
                _mm_storeu_pd(ps.divPrev, dv);
                _mm_storeu_pd(ps.hp, hp);
                if constexpr (std::is_same_v<Curve, FastTableCurve>)
                {
                    const avx::TableView view{reinterpret_cast<const double*>(curve.t->node.data()), curve.t->range,
                                              curve.t->invStep, curve.t->step, curve.t->cells};
                    i = avx::adaa2PairTable(view, ps, dcAvx, xs0, xs1, out0, out1, i, len);
                }
                else
                    i = avx::adaa2PairFold(ps, dcAvx, xs0, xs1, out0, out1, i, len);
                xp1 = _mm_loadu_pd(ps.xPrev);
                xp2 = _mm_loadu_pd(ps.xPrev2);
                g1 = _mm_loadu_pd(ps.F2Prev);
                dv = _mm_loadu_pd(ps.divPrev);
                hp = _mm_loadu_pd(ps.hp);
                avxResume = i + 2;
                if (i >= len)
                    break;
            }
#endif
        const __m128d x = _mm_set_pd(xs1[i], xs0[i]);
        const __m128d d01 = _mm_sub_pd(x, xp1), d02 = _mm_sub_pd(x, xp2);
        const __m128d ok = _mm_and_pd(_mm_cmpgt_pd(_mm_andnot_pd(signMask, d01), epsilon),
                                      _mm_cmpgt_pd(_mm_andnot_pd(signMask, d02), epsilon));
        __m128d yv;
        if (settled && _mm_movemask_pd(ok) == 3) [[likely]]
        {
            const __m128d g0 = ev.g2(x);
            const __m128d inv = _mm_div_pd(one, _mm_mul_pd(d01, d02));
            const __m128d av = _mm_mul_pd(_mm_mul_pd(_mm_sub_pd(g0, g1), d02), inv);
            yv = _mm_mul_pd(_mm_mul_pd(_mm_mul_pd(two, _mm_sub_pd(av, dv)), d01), inv);
            dv = av;
            xp2 = xp1;
            xp1 = x;
            g1 = g0;
        }
        else if (isTable && settled && nearBoth(x, xp1, xp2))
        {
          if constexpr (isTable)
          {
            // All three samples of both lanes within eps of each other (a quiet band): the expansions of adaa2Update, in
            // the lanes, with the same operations per lane.
            const __m128d g0 = ev.g2(x);
            const __m128d fx = ev.f(x), fb = ev.f(xp1);
            const __m128d mid = _mm_mul_pd(_mm_set1_pd(0.5), _mm_add_pd(x, xp1));
            const __m128d av = _mm_add_pd(ev.F(mid), _mm_mul_pd(_mm_mul_pd(d01, _mm_sub_pd(fx, fb)), _mm_set1_pd(1.0 / 24.0)));
            yv = ev.f(_mm_mul_pd(_mm_add_pd(_mm_add_pd(x, xp1), xp2), _mm_set1_pd(1.0 / 3.0)));
            dv = av;
            xp2 = xp1;
            xp1 = x;
            g1 = g0;
          }
        }
        else
        {
            _mm_storel_pd(&sa.xPrev, xp1);
            _mm_storeh_pd(&sb.xPrev, xp1);
            _mm_storel_pd(&sa.xPrev2, xp2);
            _mm_storeh_pd(&sb.xPrev2, xp2);
            _mm_storel_pd(&sa.F2Prev, g1);
            _mm_storeh_pd(&sb.F2Prev, g1);
            _mm_storel_pd(&sa.divPrev, dv);
            _mm_storeh_pd(&sb.divPrev, dv);
            const double y0 = adaa2Update(sa, xs0[i], curve);
            const double y1 = adaa2Update(sb, xs1[i], curve);
            yv = _mm_set_pd(y1, y0);
            xp1 = _mm_set_pd(sb.xPrev, sa.xPrev);
            xp2 = _mm_set_pd(sb.xPrev2, sa.xPrev2);
            g1 = _mm_set_pd(sb.F2Prev, sa.F2Prev);
            dv = _mm_set_pd(sb.divPrev, sa.divPrev);
            settled = sa.history >= 2 && sb.history >= 2;
        }
        if (blockDc)
        {
            const __m128d s = hp;
            hp = _mm_add_pd(_mm_mul_pd(s, dcKeep), _mm_mul_pd(dcIn, yv));
            yv = _mm_mul_pd(dcOut, _mm_sub_pd(yv, s));
        }
        const __m128 yf = _mm_cvtpd_ps(yv);
        _mm_store_ss(out0 + i, yf);
        _mm_store_ss(out1 + i, _mm_shuffle_ps(yf, yf, _MM_SHUFFLE(1, 1, 1, 1)));
    }
    _mm_storel_pd(&a.xPrev, xp1);
    _mm_storeh_pd(&b.xPrev, xp1);
    _mm_storel_pd(&a.xPrev2, xp2);
    _mm_storeh_pd(&b.xPrev2, xp2);
    _mm_storel_pd(&a.F2Prev, g1);
    _mm_storeh_pd(&b.F2Prev, g1);
    _mm_storel_pd(&a.divPrev, dv);
    _mm_storeh_pd(&b.divPrev, dv);
    a.history = sa.history;
    b.history = sb.history;
    _mm_storel_pd(&a.dc.s, hp);
    _mm_storeh_pd(&b.dc.s, hp);
}
#else
template <class Curve>
void Saturator::adaa2SagPair(Saturator& a, Saturator& b, const Curve& curve, double* xs0, double* xs1, int len)
{
    a.adaa2Sag(curve, xs0, len);
    b.adaa2Sag(curve, xs1, len);
}

template <class Curve>
void Saturator::adaa2PairRun(Saturator& a, Saturator& b, const Curve& curve, const double* xs0, const double* xs1,
                             float* out0, float* out1, int len)
{
    a.adaa2Run(curve, xs0, out0, len);
    b.adaa2Run(curve, xs1, out1, len);
}
#endif

template <class Curve> void Saturator::adaa2PairBlock(Saturator& a, Saturator& b, const Curve& c, float* x, float* y, int n)
{
    double xs0[kAdaaTile], xs1[kAdaaTile];
    for (int base = 0; base < n; base += kAdaaTile)
    {
        const int len = std::min(kAdaaTile, n - base);
        const bool sag = a.hasSag() && b.hasSag();
        a.adaa2Drive(c, x + base, xs0, len, !sag);
        b.adaa2Drive(c, y + base, xs1, len, !sag);
        if (sag)
        {
            if constexpr (std::is_same_v<Curve, FastTableCurve>)
                adaa2SagPair(a, b, c, xs0, xs1, len);
            else
            {
                a.adaa2Sag(c, xs0, len);
                b.adaa2Sag(c, xs1, len);
            }
        }
        adaa2PairRun(a, b, c, xs0, xs1, x + base, y + base, len);
    }
}

double Saturator::tapeFastStep(double h)
{
    tapeM = tapeFastScalar(TapeFastCoef(ja), tapeM, tapeHprev, h);
    tapeHprev = h;
    return tapeM * tapeNorm;
}

// Both channels (or one) of the relaxation hysteresis in tiles: a vector pass makes the per-sample coefficients from the
// driven field, then the only recursion left (m <- p + wd max(q - delta m, 0), with p = m + c dMan) runs for the lanes side
// by side, and a last pass scales and blocks the DC.
template <int N>
void Saturator::tapeFastBlock(Saturator* const (&s)[N], const float* const (&in)[N], float* const (&out)[N], int n)
{
    constexpr int tile = kAdaaTile;
#if CRUCIBLE_SIMD_SSE_EXACT
    // Two channels on x86 without FMA: the same float and double operations per lane as the loops below, with the
    // coefficient pass four values (two samples of both channels) at a time, and the recursion, the scaling and the DC
    // blocker of both channels in one loop, so that the chains overlap. MSVC left the coefficient pass scalar (two
    // divisions per sample) and ran the DC blocker of the channels one after the other.
    if constexpr (N == 2)
        if ((s[0]->dcG > 0.0) == (s[1]->dcG > 0.0))
        {
            const TapeFastCoef q(s[0]->ja);
            alignas(16) double xb[2][tile + 2];
            alignas(16) float cm2[2 * tile + 4], qd2[2 * tile + 4], wd2[2 * tile + 4], dl2[2 * tile + 4];
            const __m128 lo = _mm_set1_ps(-4.97f), hi = _mm_set1_ps(4.97f);
            const auto tanh4 = [&](__m128 x)
            {
                x = _mm_max_ps(lo, _mm_min_ps(hi, x)); // std::clamp(x, -4.97f, 4.97f)
                const __m128 x2 = _mm_mul_ps(x, x);
                const __m128 num = _mm_mul_ps(
                    x, _mm_add_ps(_mm_set1_ps(135135.0f),
                                  _mm_mul_ps(x2, _mm_add_ps(_mm_set1_ps(17325.0f),
                                                            _mm_mul_ps(x2, _mm_add_ps(_mm_set1_ps(378.0f), x2))))));
                const __m128 den =
                    _mm_add_ps(_mm_set1_ps(135135.0f),
                               _mm_mul_ps(x2, _mm_add_ps(_mm_set1_ps(62370.0f),
                                                         _mm_mul_ps(x2, _mm_add_ps(_mm_set1_ps(3150.0f),
                                                                                   _mm_mul_ps(x2, _mm_set1_ps(28.0f)))))));
                return _mm_div_ps(num, den);
            };
            const __m128 qc = _mm_set1_ps(q.c), qg = _mm_set1_ps(q.gScale), one = _mm_set1_ps(1.0f);
            const __m128d zeroD = _mm_setzero_pd(), oneD = _mm_set1_pd(1.0), minusOneD = _mm_set1_pd(-1.0),
                          signD = _mm_set1_pd(-0.0);
            // Coefficients of two consecutive samples of both channels; x01 holds (ch0, ch1) of the first, x23 of the
            // second, p01 / p23 the field one sample earlier, manPrev the curve one sample earlier in the same layout.
            const auto coefficients = [&](__m128d x01, __m128d x23, __m128d p01, __m128d p23, __m128 man, __m128 manPrev,
                                          float* c, float* qdOut, float* wdOut, float* dlOut)
            {
                const __m128d d01 = _mm_sub_pd(x01, p01), d23 = _mm_sub_pd(x23, p23);
                const __m128d s01 = _mm_cmpge_pd(d01, zeroD), s23 = _mm_cmpge_pd(d23, zeroD);
                const __m128 delta =
                    _mm_movelh_ps(_mm_cvtpd_ps(_mm_or_pd(_mm_and_pd(s01, oneD), _mm_andnot_pd(s01, minusOneD))),
                                  _mm_cvtpd_ps(_mm_or_pd(_mm_and_pd(s23, oneD), _mm_andnot_pd(s23, minusOneD))));
                const __m128 adh =
                    _mm_movelh_ps(_mm_cvtpd_ps(_mm_andnot_pd(signD, d01)), _mm_cvtpd_ps(_mm_andnot_pd(signD, d23)));
                const __m128 cv = _mm_mul_ps(qc, _mm_sub_ps(man, manPrev));
                const __m128 g = _mm_mul_ps(qg, adh);
                // relaxWeight: 1 - 1 / (1 + g (1 + g (0.5 + g / 6)))
                const __m128 poly = _mm_add_ps(
                    one, _mm_mul_ps(g, _mm_add_ps(one, _mm_mul_ps(g, _mm_add_ps(_mm_set1_ps(0.5f),
                                                                                _mm_mul_ps(g, _mm_set1_ps(1.0f / 6.0f)))))));
                const __m128 w = _mm_sub_ps(one, _mm_div_ps(one, poly));
                _mm_storeu_ps(c, cv);
                _mm_storeu_ps(qdOut, _mm_mul_ps(delta, _mm_sub_ps(man, cv)));
                _mm_storeu_ps(wdOut, _mm_mul_ps(w, delta));
                _mm_storeu_ps(dlOut, delta);
            };
            Saturator& a = *s[0];
            Saturator& b = *s[1];
            const bool blockDc = a.dcG > 0.0;
            const double ka = a.dcG / (1.0 + a.dcG), kb = b.dcG / (1.0 + b.dcG);
            const __m128d dcKeep = _mm_set_pd(1.0 - 2.0 * kb, 1.0 - 2.0 * ka), dcIn = _mm_set_pd(2.0 * kb, 2.0 * ka),
                          dcOut = _mm_set_pd(1.0 - kb, 1.0 - ka), norm = _mm_set_pd(b.tapeNorm, a.tapeNorm);
            const __m128 zero = _mm_setzero_ps();
            for (int base = 0; base < n; base += tile)
            {
                const int len = std::min(tile, n - base);
                double* x0 = xb[0] + 1;
                double* x1 = xb[1] + 1;
                x0[-1] = a.tapeHprev;
                x1[-1] = b.tapeHprev;
                a.adaa2Drive(FastTableCurve{nullptr}, in[0] + base, x0, len, false);
                b.adaa2Drive(FastTableCurve{nullptr}, in[1] + base, x1, len, false);
                // the curve one sample before the tile, in the upper half (lanes 2, 3)
                __m128 manLast = tanh4(_mm_movelh_ps(zero, _mm_cvtpd_ps(_mm_set_pd(x1[-1], x0[-1]))));
                int i = 0;
                for (; i + 2 <= len; i += 2)
                {
                    const __m128d x01 = _mm_set_pd(x1[i], x0[i]), x23 = _mm_set_pd(x1[i + 1], x0[i + 1]);
                    const __m128d p01 = _mm_set_pd(x1[i - 1], x0[i - 1]);
                    const __m128 man = tanh4(_mm_movelh_ps(_mm_cvtpd_ps(x01), _mm_cvtpd_ps(x23)));
                    const __m128 manPrev = _mm_shuffle_ps(manLast, man, _MM_SHUFFLE(1, 0, 3, 2));
                    coefficients(x01, x23, p01, x01, man, manPrev, cm2 + 2 * i, qd2 + 2 * i, wd2 + 2 * i, dl2 + 2 * i);
                    manLast = man;
                }
                if (i < len) // the last sample of an odd tile (the upper lanes are computed and not used)
                {
                    const __m128d x01 = _mm_set_pd(x1[i], x0[i]);
                    const __m128d p01 = _mm_set_pd(x1[i - 1], x0[i - 1]);
                    const __m128 man = tanh4(_mm_movelh_ps(_mm_cvtpd_ps(x01), _mm_cvtpd_ps(x01)));
                    const __m128 manPrev = _mm_shuffle_ps(manLast, man, _MM_SHUFFLE(1, 0, 3, 2));
                    coefficients(x01, x01, p01, x01, man, manPrev, cm2 + 2 * i, qd2 + 2 * i, wd2 + 2 * i, dl2 + 2 * i);
                }
                // m <- (m + c) + wd max(qd - dl m, 0) for both channels, then y = m norm through the DC blocker
                __m128 m = _mm_unpacklo_ps(_mm_set_ss(static_cast<float>(a.tapeM)), _mm_set_ss(static_cast<float>(b.tapeM)));
                __m128d hp = _mm_set_pd(b.dc.s, a.dc.s);
                const auto pairLoad = [](const float* p)
                { return _mm_castpd_ps(_mm_load_sd(reinterpret_cast<const double*>(p))); };
                for (int j = 0; j < len; ++j)
                {
                    const __m128 p = _mm_add_ps(m, pairLoad(cm2 + 2 * j));
                    const __m128 r =
                        _mm_max_ps(zero, _mm_sub_ps(pairLoad(qd2 + 2 * j), _mm_mul_ps(pairLoad(dl2 + 2 * j), m)));
                    m = _mm_add_ps(p, _mm_mul_ps(pairLoad(wd2 + 2 * j), r));
                    __m128d y = _mm_mul_pd(_mm_cvtps_pd(m), norm);
                    if (blockDc)
                    {
                        const __m128d st = hp;
                        hp = _mm_add_pd(_mm_mul_pd(st, dcKeep), _mm_mul_pd(dcIn, y));
                        y = _mm_mul_pd(dcOut, _mm_sub_pd(y, st));
                    }
                    const __m128 yf = _mm_cvtpd_ps(y);
                    _mm_store_ss(out[0] + base + j, yf);
                    _mm_store_ss(out[1] + base + j, _mm_shuffle_ps(yf, yf, _MM_SHUFFLE(1, 1, 1, 1)));
                }
                a.tapeM = static_cast<double>(_mm_cvtss_f32(m));
                b.tapeM = static_cast<double>(_mm_cvtss_f32(_mm_shuffle_ps(m, m, _MM_SHUFFLE(1, 1, 1, 1))));
                a.tapeHprev = x0[len - 1];
                b.tapeHprev = x1[len - 1];
                if (blockDc)
                {
                    _mm_storel_pd(&a.dc.s, hp);
                    _mm_storeh_pd(&b.dc.s, hp);
                }
            }
            return;
        }
#endif
    alignas(16) double xs[N][tile];
    alignas(16) float man[N][tile], cm[N][tile], qd[N][tile], wd[N][tile], dl[N][tile], mm[N][tile];
    const TapeFastCoef coef(s[0]->ja);
    for (int base = 0; base < n; base += tile)
    {
        const int len = std::min(tile, n - base);
        float manPrev[N];
        double hPrev[N];
        for (int k = 0; k < N; ++k)
        {
            s[k]->adaa2Drive(FastTableCurve{nullptr}, in[k] + base, xs[k], len, false);
            hPrev[k] = s[k]->tapeHprev;
            manPrev[k] = fastTanhF(static_cast<float>(hPrev[k]));
            for (int i = 0; i < len; ++i)
                man[k][i] = fastTanhF(static_cast<float>(xs[k][i]));
            for (int i = 0; i < len; ++i)
            {
                const double dh = xs[k][i] - (i > 0 ? xs[k][i - 1] : hPrev[k]);
                const float delta = dh >= 0.0 ? 1.0f : -1.0f;
                const float m0 = i > 0 ? man[k][i - 1] : manPrev[k];
                const float c = coef.c * (man[k][i] - m0);
                cm[k][i] = c;
                qd[k][i] = delta * (man[k][i] - c);
                wd[k][i] = relaxWeight(coef.gScale * static_cast<float>(std::abs(dh))) * delta;
                dl[k][i] = delta;
            }
        }
        float m[N];
        for (int k = 0; k < N; ++k)
            m[k] = static_cast<float>(s[k]->tapeM);
        for (int i = 0; i < len; ++i)
            for (int k = 0; k < N; ++k)
            {
                const float p = m[k] + cm[k][i];
                const float r = std::max(qd[k][i] - dl[k][i] * m[k], 0.0f);
                m[k] = p + wd[k][i] * r;
                mm[k][i] = m[k];
            }
        for (int k = 0; k < N; ++k)
        {
            Saturator& t = *s[k];
            t.tapeM = static_cast<double>(m[k]);
            t.tapeHprev = xs[k][len - 1];
            const bool blockDc = t.dcG > 0.0;
            const double kk = t.dcG / (1.0 + t.dcG), dcKeep = 1.0 - 2.0 * kk, dcIn = 2.0 * kk, dcOut = 1.0 - kk;
            double hp = t.dc.s;
            const double norm = t.tapeNorm;
            for (int i = 0; i < len; ++i)
            {
                double y = static_cast<double>(mm[k][i]) * norm;
                if (blockDc)
                {
                    const double st = hp;
                    hp = st * dcKeep + dcIn * y;
                    y = dcOut * (y - st);
                }
                out[k][base + i] = static_cast<float>(y);
            }
            t.dc.s = hp;
        }
    }
}

double Saturator::tapeStep(double h)
{
    if (cfg.tapeFast)
        return tapeFastStep(h);
    tapeM = jaStep(ja, tapeM, tapeHprev, h, cfg.integrator);
    tapeHprev = h;
    return tapeM * tapeNorm;
}

double Saturator::transformerStep(double x)
{
    const double v = (x - fluxState) * fluxG / (1.0 + fluxG);
    const double flux = v + fluxState;
    fluxState = flux + v;
    const double t = fastTanh(flux);
    return (1.0 - t * t) * (x - flux);
}

double Saturator::model(double x)
{
    switch (cfg.model)
    {
    case Model::Soft:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{&curveTables().soft});
        return cfg.adaa ? adaaStep(x, StaticCurve<SoftCurve>{}) : SoftCurve::f(x);
    case Model::Hard:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{&curveTables().hard});
        return cfg.adaa ? adaaStep(x, StaticCurve<HardCurve>{}) : HardCurve::f(x);
    case Model::Rail:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{&curveTables().rail});
        return cfg.adaa ? adaaStep(x, StaticCurve<RailCurve>{}) : RailCurve::f(x);
    case Model::Diode:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{&curveTables().diode});
        return cfg.adaa ? adaaStep(x, StaticCurve<DiodeCurve>{}) : DiodeCurve::f(x);
    case Model::Fold:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, StaticCurve<FoldCurve>{});
        return cfg.adaa ? adaaStep(x, StaticCurve<FoldCurve>{}) : FoldCurve::f(x);
    case Model::RectHalf:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{&curveTables().rectHalf});
        return cfg.adaa ? adaaStep(x, StaticCurve<RectHalfCurve>{}) : RectHalfCurve::f(x);
    case Model::RectFull:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{&curveTables().rectFull});
        return cfg.adaa ? adaaStep(x, StaticCurve<RectFullCurve>{}) : RectFullCurve::f(x);
    case Model::ZeroSquare:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{&curveTables().zeroSquare});
        return cfg.adaa ? adaaStep(x, StaticCurve<ZeroSquareCurve>{}) : ZeroSquareCurve::f(x);
    case Model::Asym:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{&curveTables().asym});
        return cfg.adaa ? adaaStep(x, StaticCurve<AsymCurve>{}) : AsymCurve::f(x);
    case Model::SineFold:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, StaticCurve<SineFoldCurve>{});
        return cfg.adaa ? adaaStep(x, StaticCurve<SineFoldCurve>{}) : SineFoldCurve::f(x);
    case Model::LinFold:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, StaticCurve<LinFoldCurve>{});
        return cfg.adaa ? adaaStep(x, StaticCurve<LinFoldCurve>{}) : LinFoldCurve::f(x);
    case Model::TriodeSE:
    case Model::TriodePP:
    case Model::Pentode:
        if (cfg.adaa && cfg.adaaOrder >= 2)
            return adaa2Step(x, TableCurve{table});
        return cfg.adaa ? adaaStep(x, TableCurve{table}) : table->fAt(x);
    case Model::Tape:
        return tapeStep(x);
    case Model::Transformer:
        return transformerStep(x);
    default:
        return x;
    }
}

float Saturator::process(float in)
{
    driveGain += (driveTarget - driveGain) * driveCoef;
    double x = static_cast<double>(in) * driveGain + biasOffset;
    const bool tube = cfg.model == Model::TriodeSE || cfg.model == Model::TriodePP || cfg.model == Model::Pentode;
    if (tube && cfg.sag > 0.0f)
        x -= 0.6 * cfg.sag * sagEnv;
    double y = model(x);
    if (tube && cfg.sag > 0.0f)
    {
        const double a = std::abs(y);
        sagEnv += (a - sagEnv) * (a > sagEnv ? sagAtk : sagRel);
    }
    if (dcG > 0.0)
        y = dc.step(y, dcG);
    return static_cast<float>(y);
}

void Saturator::process(const float* in, float* out, int n)
{
    if (cfg.adaa && cfg.adaaOrder >= 2)
    {
        switch (cfg.model)
        {
        case Model::Soft:
            adaa2Block(FastTableCurve{&curveTables().soft}, in, out, n);
            return;
        case Model::Hard:
            adaa2Block(FastTableCurve{&curveTables().hard}, in, out, n);
            return;
        case Model::Rail:
            adaa2Block(FastTableCurve{&curveTables().rail}, in, out, n);
            return;
        case Model::Diode:
            adaa2Block(FastTableCurve{&curveTables().diode}, in, out, n);
            return;
        case Model::Fold:
            adaa2Block(StaticCurve<FoldCurve>{}, in, out, n);
            return;
        case Model::RectHalf:
            adaa2Block(FastTableCurve{&curveTables().rectHalf}, in, out, n);
            return;
        case Model::RectFull:
            adaa2Block(FastTableCurve{&curveTables().rectFull}, in, out, n);
            return;
        case Model::ZeroSquare:
            adaa2Block(FastTableCurve{&curveTables().zeroSquare}, in, out, n);
            return;
        case Model::Asym:
            adaa2Block(FastTableCurve{&curveTables().asym}, in, out, n);
            return;
        case Model::SineFold:
            adaa2Block(StaticCurve<SineFoldCurve>{}, in, out, n);
            return;
        case Model::LinFold:
            adaa2Block(StaticCurve<LinFoldCurve>{}, in, out, n);
            return;
        case Model::TriodeSE:
        case Model::TriodePP:
        case Model::Pentode:
            adaa2Block(FastTableCurve{table}, in, out, n);
            return;
        case Model::Transformer:
            transformerBlock(in, out, n);
            return;
        default:
            break;
        }
    }
    if (cfg.model == Model::Tape && cfg.tapeFast)
    {
        Saturator* const s[1] = {this};
        const float* const i1[1] = {in};
        float* const o1[1] = {out};
        tapeFastBlock<1>(s, i1, o1, n);
        return;
    }
    for (int i = 0; i < n; ++i)
        out[i] = process(in[i]);
}

void Saturator::processPair(Saturator& a, Saturator& b, float* x, float* y, int n)
{
    if (a.cfg.model != b.cfg.model || a.cfg.adaa != b.cfg.adaa || a.cfg.adaaOrder != b.cfg.adaaOrder)
    {
        a.process(x, x, n);
        b.process(y, y, n);
        return;
    }
    if (a.cfg.adaa && a.cfg.adaaOrder >= 2)
    {
        switch (a.cfg.model)
        {
        case Model::Soft:
            adaa2PairBlock(a, b, FastTableCurve{&curveTables().soft}, x, y, n);
            return;
        case Model::Hard:
            adaa2PairBlock(a, b, FastTableCurve{&curveTables().hard}, x, y, n);
            return;
        case Model::Rail:
            adaa2PairBlock(a, b, FastTableCurve{&curveTables().rail}, x, y, n);
            return;
        case Model::Diode:
            adaa2PairBlock(a, b, FastTableCurve{&curveTables().diode}, x, y, n);
            return;
        case Model::Fold:
            adaa2PairBlock(a, b, StaticCurve<FoldCurve>{}, x, y, n);
            return;
        case Model::RectHalf:
            adaa2PairBlock(a, b, FastTableCurve{&curveTables().rectHalf}, x, y, n);
            return;
        case Model::RectFull:
            adaa2PairBlock(a, b, FastTableCurve{&curveTables().rectFull}, x, y, n);
            return;
        case Model::ZeroSquare:
            adaa2PairBlock(a, b, FastTableCurve{&curveTables().zeroSquare}, x, y, n);
            return;
        case Model::Asym:
            adaa2PairBlock(a, b, FastTableCurve{&curveTables().asym}, x, y, n);
            return;
        case Model::TriodeSE:
        case Model::TriodePP:
        case Model::Pentode:
            adaa2PairBlock(a, b, FastTableCurve{a.table}, x, y, n);
            return;
        default:
            break;
        }
    }
    if (a.cfg.model == Model::Tape && a.cfg.tapeFast && b.cfg.tapeFast)
    {
        Saturator* const s[2] = {&a, &b};
        const float* const in[2] = {x, y};
        float* const out[2] = {x, y};
        tapeFastBlock<2>(s, in, out, n);
        return;
    }
    if (a.cfg.model == Model::Tape)
    {
        for (int i = 0; i < n; ++i)
        {
            x[i] = a.process(x[i]);
            y[i] = b.process(y[i]);
        }
        return;
    }
    a.process(x, x, n);
    b.process(y, y, n);
}

simd::Float4 Saturator4::process(simd::Float4 x)
{
    alignas(16) float v[4];
    x.store(v);
    for (size_t i = 0; i < 4; ++i)
        v[i] = lane[i].process(v[i]);
    return simd::Float4::load(v);
}

// ------------------------------------------------------------------------------------------------------------
void OversampledSaturator::prepare(double rate, int factor, OversamplingFilter filter, Quality quality, int maxBlock)
{
    os = factor >= 8 ? 8 : (factor >= 4 ? 4 : (factor >= 2 ? 2 : 1));
    sat.prepare(rate * os);
    if (os > 1)
    {
        over.prepare(os == 2 ? 1 : (os == 4 ? 2 : 3), filter, quality, maxBlock);
        lat = std::round(over.latency());
        hi.assign(static_cast<size_t>(maxBlock) * static_cast<size_t>(os), 0.0f);
    }
    else
    {
        lat = 0.0f;
        hi.clear();
    }
    reset();
}

void OversampledSaturator::reset()
{
    sat.reset();
    if (os > 1)
        over.reset();
}

void OversampledSaturator::process(float* x, int n)
{
    if (os == 1)
    {
        sat.process(x, x, n);
        return;
    }
    over.up(x, n, hi.data());
    sat.process(hi.data(), hi.data(), n * os);
    over.down(hi.data(), n, x);
}

} // namespace crucible::saturation
