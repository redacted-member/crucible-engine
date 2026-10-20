// SPDX-License-Identifier: MIT
#pragma once

// Clipping curve with first-order antiderivative anti-aliasing (Parker, Zavalishin and Le Bivic, "Reducing the aliasing of
// nonlinear waveshaping using continuous-time convolution", DAFx 2016), shared by the maximiser's soft-clip stage and the
// clipper effect (docs/plans/ENGINE-FX-PLAN.md, section 3.2).
//
// One curve family in units of the ceiling (the output never exceeds 1 in magnitude):
//
//   f(x) = x                                  |x| <= a
//   f(x) = a + w tanh((|x| - a) / w)          |x| >  a,   a = 1 - w
//
// w = 0 is the hard clipper, w = 1 is tanh(x); in between, the knee widens. The slope is continuous at the knee. With
// F the antiderivative (F(x) = x^2 / 2 inside, a^2 / 2 + a (|x| - a) + w^2 ln cosh((|x| - a) / w) outside) the ADAA output
// is (F(x[n]) - F(x[n-1])) / (x[n] - x[n-1]), the mean of f over the last sample step: it sits half a sample behind the
// input. The second-order form uses the second antiderivative F2 and sits one sample behind; it removes about 25 dB more
// aliasing of a hard clip at the same oversampling.
//
// ADAA filters the curve's output with a short kernel before sampling it, so the linear part is low-passed as well: a
// two-point mean for the first order, a three-point mean for the second (-0.45 dB / -1.3 dB at 20 kHz at 4x, more at lower
// factors). Inside the linear part (|x| <= a for the newest samples) the output is exactly that mean and the stage skips the
// divisions and table reads. ClipperEffect equalises the droop in the oversampled domain (designEqualiser there). tanh, ln
// cosh and G come from cubic Hermite tables (error below 1e-9), so nothing here calls libm in the audio path.

#include <crucible/Simd.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace crucible::fx
{

// Shared tables of tanh and ln cosh over [0, 10]. instance() builds them on first use: call it from a control thread
// (prepare()) before the audio thread needs it.
class LogCoshTable
{
public:
    static const LogCoshTable& instance();

    static constexpr int kPerUnit = 32;
    static constexpr int kRange = 10;

    // tanh(u) and ln cosh(u) for u >= 0. The per-sample helpers here and in ClipCurve / AdaaClipper are forced inline:
    // MSVC otherwise keeps every one an out-of-line call per sample (clang inlines them on its own).
    CRUCIBLE_ALWAYS_INLINE double tanhAt(double u) const
    {
        if (u >= kRange)
            return 1.0;
        return hermite(tanhValue, tanhSlope, u);
    }
    CRUCIBLE_ALWAYS_INLINE double lnCoshAt(double u) const
    {
        if (u >= kRange)
            return u - 0.69314718055994531;
        return hermite(lnValue, lnSlope, u);
    }
    // Integral of ln cosh from 0 to u.
    CRUCIBLE_ALWAYS_INLINE double lnCoshIntegralAt(double u) const
    {
        if (u >= kRange)
            return 0.5 * u * u - 0.69314718055994531 * u + 0.41123351478979;
        return hermite(intValue, lnValue, u);
    }

private:
    LogCoshTable();
    CRUCIBLE_ALWAYS_INLINE static double hermite(const std::vector<double>& v, const std::vector<double>& m, double u)
    {
        const double s = u * kPerUnit;
        const int j = static_cast<int>(s);
        const double t = s - j;
        const double h = 1.0 / kPerUnit;
        const double t2 = t * t, t3 = t2 * t;
        const size_t k = static_cast<size_t>(j);
        return (2.0 * t3 - 3.0 * t2 + 1.0) * v[k] + (t3 - 2.0 * t2 + t) * h * m[k] + (-2.0 * t3 + 3.0 * t2) * v[k + 1] +
               (t3 - t2) * h * m[k + 1];
    }
    std::vector<double> tanhValue, tanhSlope, lnValue, lnSlope, intValue;
};

class ClipCurve
{
public:
    // softness w in [0, 1]. Cheap; call per block.
    void setSoftness(double w)
    {
        width = w < 0.0 ? 0.0 : (w > 1.0 ? 1.0 : w);
        knee = 1.0 - width;
        hard = width < 1.0e-6;
    }
    double softness() const { return width; }

    CRUCIBLE_ALWAYS_INLINE double f(double x) const
    {
        const double ax = std::abs(x);
        if (ax <= knee)
            return x;
        const double y = hard ? 1.0 : knee + width * table->tanhAt((ax - knee) / width);
        return x < 0.0 ? -y : y;
    }
    // Antiderivative, even, F(0) = 0.
    CRUCIBLE_ALWAYS_INLINE double F(double x) const
    {
        const double ax = std::abs(x);
        if (ax <= knee)
            return 0.5 * ax * ax;
        if (hard)
            return ax - 0.5;
        const double u = (ax - knee) / width;
        return 0.5 * knee * knee + knee * (ax - knee) + width * width * table->lnCoshAt(u);
    }
    // Second antiderivative, odd, F2(0) = 0.
    CRUCIBLE_ALWAYS_INLINE double F2(double x) const
    {
        const double ax = std::abs(x);
        double v;
        if (ax <= knee)
            v = ax * ax * ax / 6.0;
        else if (hard)
            v = 1.0 / 6.0 + 0.5 * (ax * ax - 1.0) - 0.5 * (ax - 1.0);
        else
        {
            const double d = ax - knee;
            v = knee * knee * knee / 6.0 + 0.5 * knee * knee * d + 0.5 * knee * d * d +
                width * width * width * table->lnCoshIntegralAt(d / width);
        }
        return x < 0.0 ? -v : v;
    }

    double kneePoint() const { return knee; }

private:
    const LogCoshTable* table = &LogCoshTable::instance(); // built when the curve is created, on a control thread
    double width = 0.0, knee = 1.0;
    bool hard = true;
};

// One channel of ADAA clipping.
class AdaaClipper
{
public:
    void reset()
    {
        x1 = x2 = 0.0;
        f1 = g1 = g2 = 0.0;
        lastOrder = 2;
    }
    const ClipCurve& curve() const { return shape; }
    // Changes the curve. The antiderivatives of the last samples depend on it, so they are recomputed: a stale value would
    // make the next difference quotient a spike.
    void setSoftness(double w)
    {
        if (w == shape.softness())
            return;
        shape.setSoftness(w);
        f1 = shape.F(x1);
        g1 = shape.F2(x1);
        g2 = shape.F2(x2);
    }
    // order 0: the plain memoryless curve (for comparison); 1: antiderivative anti-aliasing of the first order (the audio
    // sits half a sample behind the input); 2: of the second order (one sample behind).
    CRUCIBLE_ALWAYS_INLINE float process(float xin, int order = 2)
    {
        const double x = (xin == xin && std::abs(xin) < 1.0e18f) ? static_cast<double>(xin) : 0.0;
        if (order != lastOrder)
        {
            // The cached antiderivatives belong to the order that last ran: bring them up to date with the last samples.
            lastOrder = order;
            f1 = shape.F(x1);
            g1 = shape.F2(x1);
            g2 = shape.F2(x2);
        }
        if (order <= 0)
        {
            x2 = x1;
            x1 = x;
            return static_cast<float>(shape.f(x));
        }
        const double a = shape.kneePoint();
        double y;
        if (order == 1)
        {
            if (std::abs(x) <= a && std::abs(x1) <= a)
            {
                y = 0.5 * (x + x1);
                f1 = 0.5 * x * x;
            }
            else
            {
                const double fx = shape.F(x), dx = x - x1;
                y = std::abs(dx) < 1.0e-7 ? shape.f(0.5 * (x + x1)) : (fx - f1) / dx;
                f1 = fx;
            }
        }
        else
        {
            if (std::abs(x) <= a && std::abs(x1) <= a && std::abs(x2) <= a)
            {
                y = (x + x1 + x2) * (1.0 / 3.0);
                g2 = x1 * x1 * x1 / 6.0;
                g1 = x * x * x / 6.0;
            }
            else
            {
                const double gx = shape.F2(x);
                const double d01 = x - x1, d12 = x1 - x2, d02 = x - x2;
                // F2 grows like x^2, so the quotient loses about eps * x^2 / (d01 * d02): the threshold scales with the
                // level. Pairwise max (the same value as std::max over the list; the list form is an out-of-line call on
                // MSVC).
                const double eps = 1.0e-5 * std::max(std::max(1.0, std::abs(x)), std::max(std::abs(x1), std::abs(x2)));
                if (std::abs(d02) < eps || std::abs(d01) < eps || std::abs(d12) < eps)
                    y = shape.f(0.25 *
                                (x + 2.0 * x1 + x2)); // two of the points (nearly) coincide: the mean of f over the span
                else
                    y = 2.0 / d02 * ((gx - g1) / d01 - (g1 - g2) / d12);
                g2 = g1;
                g1 = gx;
            }
        }
        x2 = x1;
        x1 = x;
        return static_cast<float>(y);
    }

private:
    ClipCurve shape;
    double x1 = 0.0, x2 = 0.0, f1 = 0.0, g1 = 0.0, g2 = 0.0;
    int lastOrder = 2;
};

} // namespace crucible::fx
