// SPDX-License-Identifier: MIT
// Matched second-order sections, band prototypes and the TPT state-variable filter. See AnalogMatchedFilters.h for the
// design; control-thread and audio-thread safe (no allocation).

#include <crucible/fx/AnalogMatchedFilters.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>

namespace crucible::fx
{
namespace
{
using cd = std::complex<double>;
constexpr double kPi = 3.14159265358979323846;
constexpr double kNeperPerDb = 0.11512925464970229; // ln(10) / 20

cd polyAt(const double c[3], cd s)
{
    return c[0] + s * (c[1] + s * c[2]);
}

// ---------------------------------------------------------------------------------------------------------------
// Closed form

struct Roots
{
    cd r[2];
    int count = 0;
};

Roots rootsOf(const double c[3], int order)
{
    Roots out;
    if (order <= 1 || std::abs(c[2]) < 1.0e-300)
    {
        if (std::abs(c[1]) > 1.0e-300)
        {
            out.r[0] = -c[0] / c[1];
            out.count = 1;
        }
        return out;
    }
    const double disc = c[1] * c[1] - 4.0 * c[2] * c[0];
    const cd root = std::sqrt(cd(disc, 0.0));
    out.r[0] = (-c[1] + root) / (2.0 * c[2]);
    out.r[1] = (-c[1] - root) / (2.0 * c[2]);
    out.count = 2;
    return out;
}

// 1 - exp(x + jy) without cancellation for small x, y.
cd oneMinusExp(double x, double y)
{
    const double ex = std::exp(x);
    const double s = std::sin(0.5 * y);
    return cd(-std::expm1(x) + 2.0 * ex * s * s, -ex * std::sin(y));
}

// Reflects the zeros of b0 + b1 z^-1 + b2 z^-2 that lie outside the unit circle to their conjugate-reciprocal positions
// (the magnitude response is kept and the section becomes minimum phase).
void minimumPhase(double& b0, double& b1, double& b2, int order)
{
    if (order <= 1 || std::abs(b2) < 1.0e-300)
    {
        if (std::abs(b0) < 1.0e-300)
            return;
        const double r = -b1 / b0; // b0 + b1 z^-1 = b0 (1 - r z^-1)
        if (std::abs(r) > 1.0)
        {
            b0 *= std::abs(r);
            b1 = -b0 / r;
        }
        b2 = 0.0;
        return;
    }
    if (std::abs(b0) < 1.0e-14 * (std::abs(b1) + std::abs(b2)))
        return;
    // z^2 + (b1/b0) z + b2/b0 = (z - r1)(z - r2)
    const double p = b1 / b0, q = b2 / b0;
    const cd root = std::sqrt(cd(p * p - 4.0 * q, 0.0));
    cd r1 = 0.5 * (-p + root), r2 = 0.5 * (-p - root);
    double gain = b0;
    const auto reflect = [&](cd& r)
    {
        const double m = std::abs(r);
        if (m > 1.0)
        {
            r = r / (m * m); // 1 / conj(r)
            gain *= m;
        }
    };
    reflect(r1);
    reflect(r2);
    // b0 (1 - r1 q)(1 - r2 q) in q = z^-1; roots are real or a conjugate pair, so the products are real.
    b0 = gain;
    b1 = -gain * (r1 + r2).real();
    b2 = gain * (r1 * r2).real();
}

// Pre-warped bilinear transform of a normalised analog section (the fallback seed).
Biquad bilinear(const AnalogSection& s, double w0T)
{
    const double k = 1.0 / std::tan(0.5 * w0T);
    const double k2 = k * k;
    const auto map = [&](const double c[3], double out[3])
    {
        out[0] = c[0] + c[1] * k + c[2] * k2;
        out[1] = 2.0 * (c[0] - c[2] * k2);
        out[2] = c[0] - c[1] * k + c[2] * k2;
    };
    double n[3], d[3];
    map(s.b, n);
    map(s.a, d);
    Biquad out;
    out.b0 = n[0] / d[0];
    out.b1 = n[1] / d[0];
    out.b2 = n[2] / d[0];
    out.a1 = d[1] / d[0];
    out.a2 = d[2] / d[0];
    return out;
}

// Matched poles, numerator from the analog magnitude at DC, the centre and Nyquist. False when no real numerator exists.
bool closedForm(const AnalogSection& proto, double w0T, Biquad& out)
{
    const Roots poles = rootsOf(proto.a, proto.order);
    if (poles.count == 0)
        return false;
    cd z[2];
    cd om[2], op[2]; // 1 - z and 1 + z
    for (int i = 0; i < poles.count; ++i)
    {
        const double x = poles.r[i].real() * w0T, y = poles.r[i].imag() * w0T;
        if (x > -1.0e-12) // not a decaying pole
            return false;
        z[i] = std::polar(std::exp(x), y);
        om[i] = oneMinusExp(x, y);
        op[i] = 1.0 + z[i];
    }
    Biquad d;
    double B0, B1, B2;
    if (poles.count == 2)
    {
        d.a1 = -(z[0] + z[1]).real();
        d.a2 = (z[0] * z[1]).real();
        B0 = std::norm(om[0] * om[1]);
        B1 = std::norm(op[0] * op[1]);
        B2 = -4.0 * d.a2;
    }
    else
    {
        d.a1 = -z[0].real();
        d.a2 = 0.0;
        B0 = std::norm(om[0]);
        B1 = std::norm(op[0]);
        B2 = 0.0;
    }
    const auto m2 = [&](double analogFreq)
    {
        const double m = std::abs(analogResponse(proto, analogFreq));
        return m * m;
    };
    const double A0 = B0 * m2(0.0);
    const double A1 = B1 * m2(kPi / w0T);
    const double s0 = std::sqrt(A0), s1 = std::sqrt(A1);
    double b0, b1, b2 = 0.0;
    if (proto.order >= 2)
    {
        const double sh = std::sin(0.5 * w0T);
        const double phi1 = sh * sh, phi0 = 1.0 - phi1, phi2 = 4.0 * phi0 * phi1;
        const double Bm = B0 * phi0 + B1 * phi1 + B2 * phi2;
        const double A2 = (m2(1.0) * Bm - A0 * phi0 - A1 * phi1) / phi2;
        const double W = 0.5 * (s0 + s1);
        const double disc = W * W + A2;
        if (disc < -1.0e-9 * (W * W + std::abs(A2) + 1.0e-300))
            return false;
        const double r = std::sqrt(std::max(disc, 0.0));
        b0 = 0.5 * (W + r);
        b1 = 0.5 * (s0 - s1);
        b2 = 0.5 * (W - r);
        minimumPhase(b0, b1, b2, 2);
    }
    else
    {
        b0 = 0.5 * (s0 + s1);
        b1 = 0.5 * (s0 - s1);
        minimumPhase(b0, b1, b2, 1);
    }
    d.b0 = b0;
    d.b1 = b1;
    d.b2 = b2;
    if (!std::isfinite(d.b0 + d.b1 + d.b2 + d.a1 + d.a2) || !d.stable())
        return false;
    out = d;
    return true;
}

// ---------------------------------------------------------------------------------------------------------------
// Refinement: Levenberg-Marquardt on the logarithm of the response.

// Natural logarithm, absolute error below 1e-9 (a few times faster than libm; the fit needs a few thousand per design).
inline double fastLog(double x)
{
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(x);
    int e = static_cast<int>((bits >> 52) & 0x7ff) - 1023;
    double m = std::bit_cast<double>((bits & 0x000fffffffffffffULL) | 0x3ff0000000000000ULL); // [1, 2)
    if (m > 1.41421356237)
    {
        m *= 0.5;
        ++e;
    }
    const double t = (m - 1.0) / (m + 1.0), t2 = t * t;
    return e * 0.6931471805599453 + 2.0 * t * (1.0 + t2 * (1.0 / 3.0 + t2 * (0.2 + t2 * (1.0 / 7.0 + t2 * (1.0 / 9.0)))));
}

struct Grid
{
    static constexpr int kMax = 40;
    double omega[kMax];
    double weight[kMax];
    double c1[kMax], s1[kMax], c2[kMax], s2[kMax]; // cos / sin of omega and 2 omega
    double targetLog[kMax];                        // ln |analog response|
    double targetArg[kMax];                        // phase of the analog response
    int count = 0;
    int core = 0; // the first `core` points are inside the audio band (the error criterion)
};

double sharpnessHint(const AnalogSection& s)
{
    // Q of the narrowest feature: geometric mean of the end coefficients over the damping term.
    double q = 1.0e9;
    const auto look = [&](const double c[3])
    {
        if (c[1] > 0.0 && c[0] > 0.0 && c[2] > 0.0)
            q = std::min(q, std::sqrt(c[0] * c[2]) / c[1]);
    };
    look(s.a);
    look(s.b);
    return std::clamp(q, 0.1, 60.0);
}

constexpr double kBellCentreBoost = 2.0, kBellStart = 0.5, kBellSpan = 0.3;

void buildGrid(Grid& g, const AnalogSection& proto, double w0T, double sampleRate)
{
    const double nyquist = 0.5 * sampleRate;
    const double top = std::min(0.95 * nyquist, 21000.0);
    const double low = 20.0;
    const double w = 2.0 * kPi / sampleRate;
    int n = 0;
    const auto add = [&](double hz, double weight)
    {
        if (n >= Grid::kMax || hz <= 0.0 || hz >= nyquist * 0.9995)
            return;
        g.omega[n] = hz * w;
        g.weight[n] = weight;
        ++n;
    };
    for (int i = 0; i < 12; ++i)
        add(low * std::pow(top / low, i / 11.0), 1.0);
    for (int j = 0; j < 6; ++j)
        add(top * (0.55 + 0.45 * j / 5.0), 1.0);
    const double f0 = w0T / w;
    const double bw = std::min(3.0, 1.44 / sharpnessHint(proto)); // bandwidth in octaves
    // A bell near Nyquist is cramped: the fit would trade centre gain for the error elsewhere, so the centre is held
    // harder (the weight rises smoothly from half Nyquist and does not touch the other prototypes).
    const bool bell = proto.order == 2 && proto.b[0] == 1.0 && proto.b[2] == 1.0 && proto.a[0] == 1.0 && proto.a[2] == 1.0;
    const double nearNyquist = std::clamp((f0 / nyquist - kBellStart) / kBellSpan, 0.0, 1.0);
    add(f0, bell ? 1.0 + kBellCentreBoost * nearNyquist * nearNyquist * (3.0 - 2.0 * nearNyquist) : 1.0);
    for (double o : {0.08, 0.2, 0.4, 0.7, 1.1, 1.8})
    {
        add(f0 * std::exp2(-bw * o), 1.0);
        add(f0 * std::exp2(bw * o), 1.0);
    }
    g.core = n;
    for (double t : {0.25, 0.5, 0.75, 1.0})
        add(top + (nyquist - top) * t, 0.3);
    g.count = n;
    for (int i = 0; i < n; ++i)
    {
        g.c1[i] = std::cos(g.omega[i]);
        g.s1[i] = std::sin(g.omega[i]);
        g.c2[i] = g.c1[i] * g.c1[i] - g.s1[i] * g.s1[i];
        g.s2[i] = 2.0 * g.c1[i] * g.s1[i];
        const cd h = analogResponse(proto, g.omega[i] / w0T);
        const double m = std::abs(h);
        if (m < 1.0e-3) // deeper than -60 dB: not worth fitting
        {
            g.weight[i] = 0.0;
            g.targetLog[i] = std::log(1.0e-3);
            g.targetArg[i] = 0.0;
        }
        else
        {
            g.targetLog[i] = std::log(m);
            g.targetArg[i] = std::arg(h);
        }
    }
}

struct Point
{
    double nr, ni, dr, di, nn, dd; // N, D (real, imaginary) and their squared magnitudes
};

inline Point evalPoint(const std::array<double, 5>& p, const Grid& g, int i)
{
    Point e;
    // z^-1 = c1 - j s1, z^-2 = c2 - j s2
    e.nr = p[0] + p[1] * g.c1[i] + p[2] * g.c2[i];
    e.ni = -(p[1] * g.s1[i] + p[2] * g.s2[i]);
    e.dr = 1.0 + p[3] * g.c1[i] + p[4] * g.c2[i];
    e.di = -(p[3] * g.s1[i] + p[4] * g.s2[i]);
    e.nn = e.nr * e.nr + e.ni * e.ni;
    e.dd = e.dr * e.dr + e.di * e.di;
    return e;
}

inline double logMagnitude(const Point& e)
{
    return 0.5 * fastLog((e.nn + 1.0e-300) / (e.dd + 1.0e-300));
}
inline double wrap(double x)
{
    return x - 2.0 * kPi * std::round(x / (2.0 * kPi));
}

double cost(const std::array<double, 5>& p, const Grid& g, bool phase)
{
    double c = 0.0;
    for (int i = 0; i < g.count; ++i)
    {
        if (g.weight[i] == 0.0)
            continue;
        const Point e = evalPoint(p, g, i);
        const double dm = logMagnitude(e) - g.targetLog[i];
        double term = dm * dm;
        if (phase)
        {
            const double dp = 0.25 * wrap(std::atan2(e.ni, e.nr) - std::atan2(e.di, e.dr) - g.targetArg[i]);
            term += dp * dp;
        }
        c += g.weight[i] * g.weight[i] * term;
    }
    return c;
}

double coreErrorDb(const std::array<double, 5>& p, const Grid& g)
{
    double worst = 0.0;
    for (int i = 0; i < g.core; ++i)
        if (g.weight[i] != 0.0)
            worst = std::max(worst, std::abs(logMagnitude(evalPoint(p, g, i)) - g.targetLog[i]) / kNeperPerDb);
    return worst;
}

bool stable(const std::array<double, 5>& p)
{
    return std::abs(p[4]) < 0.9999999 && std::abs(p[3]) < 0.9999999 + p[4];
}

bool solve(double (&m)[5][6], int n, double (&x)[5])
{
    for (int c = 0; c < n; ++c)
    {
        int pv = c;
        for (int r = c + 1; r < n; ++r)
            if (std::abs(m[r][c]) > std::abs(m[pv][c]))
                pv = r;
        if (std::abs(m[pv][c]) < 1.0e-300)
            return false;
        if (pv != c)
            for (int k = 0; k < 6; ++k)
                std::swap(m[c][k], m[pv][k]);
        for (int r = c + 1; r < n; ++r)
        {
            const double f = m[r][c] / m[c][c];
            for (int k = c; k < 6; ++k)
                m[r][k] -= f * m[c][k];
        }
    }
    for (int r = n - 1; r >= 0; --r)
    {
        double s = m[r][5];
        for (int k = r + 1; k < n; ++k)
            s -= m[r][k] * x[k];
        x[r] = s / m[r][r];
    }
    return true;
}

// The parameters the fit moves: p5 = T theta, where p5 = (b0, b1, b2, a1, a2). Tying columns together keeps the zeros a
// shape needs exactly where the prototype has them (a double zero at DC for a high-pass, a zero on the unit circle for a
// notch) however little the fit is told about the stop band.
struct Parametrisation
{
    int k = 5;
    double t[5][5] = {}; // t[column][row]
    static Parametrisation free(int order)
    {
        Parametrisation m;
        if (order <= 1)
        {
            m.k = 3;
            m.t[0][0] = 1.0;
            m.t[1][1] = 1.0;
            m.t[2][3] = 1.0;
            return m;
        }
        m.k = 5;
        for (int i = 0; i < 5; ++i)
            m.t[i][i] = 1.0;
        return m;
    }
    // Numerator g * (n0, n1, n2); first order: (n0, n1).
    static Parametrisation tied(int order, double n0, double n1, double n2)
    {
        Parametrisation m;
        m.t[0][0] = n0;
        m.t[0][1] = n1;
        m.t[0][2] = n2;
        m.t[1][3] = 1.0;
        if (order <= 1)
        {
            m.k = 2;
            return m;
        }
        m.t[2][4] = 1.0;
        m.k = 3;
        return m;
    }
    // Band-pass: zero at DC, the other free.
    static Parametrisation bandPass()
    {
        Parametrisation m;
        m.k = 4;
        m.t[0][0] = 1.0;
        m.t[0][2] = -1.0;
        m.t[1][1] = 1.0;
        m.t[1][2] = -1.0;
        m.t[2][3] = 1.0;
        m.t[3][4] = 1.0;
        return m;
    }
    // theta from p5 by least squares
    void project(const std::array<double, 5>& p, double (&theta)[5]) const
    {
        double ata[5][6] = {};
        for (int a = 0; a < k; ++a)
        {
            for (int b = 0; b < k; ++b)
                for (int r = 0; r < 5; ++r)
                    ata[a][b] += t[a][r] * t[b][r];
            for (int r = 0; r < 5; ++r)
                ata[a][5] += t[a][r] * p[static_cast<size_t>(r)];
        }
        solve(ata, k, theta);
    }
    std::array<double, 5> expand(const double (&theta)[5]) const
    {
        std::array<double, 5> p{};
        for (int a = 0; a < k; ++a)
            for (int r = 0; r < 5; ++r)
                p[static_cast<size_t>(r)] += t[a][r] * theta[a];
        return p;
    }
};

constexpr double kPhaseWeight = 0.25;

// Weight of the pull towards the starting point (cost units; a 0.1 dB error at every grid point costs about 4e-3). It is far
// too small to change the fit, but it removes the flat directions along which the optimum jumps from one basin to another
// as the frequency moves, so that the design is a continuous function of the parameters.
constexpr double kAnchor = 2.0e-4;

bool refine(Biquad& bq, const Grid& g, bool phase, const Parametrisation& map)
{
    double theta[5] = {};
    map.project({bq.b0, bq.b1, bq.b2, bq.a1, bq.a2}, theta);
    const double anchor[5] = {theta[0], theta[1], theta[2], theta[3], theta[4]};
    const int dimension = map.k;
    const auto anchored = [&](const double (&t)[5])
    {
        double c = 0.0;
        for (int a = 0; a < dimension; ++a)
            c += kAnchor * (t[a] - anchor[a]) * (t[a] - anchor[a]);
        return c;
    };
    std::array<double, 5> p = map.expand(theta);
    double current = cost(p, g, phase) + anchored(theta);
    double lambda = 1.0e-3;
    const int count = map.k;
    for (int iteration = 0; iteration < 5; ++iteration)
    {
        double jtj[5][5] = {};
        double jtr[5] = {};
        for (int i = 0; i < g.count; ++i)
        {
            if (g.weight[i] == 0.0)
                continue;
            const Point e = evalPoint(p, g, i);
            const double w2 = g.weight[i] * g.weight[i];
            const double dm = logMagnitude(e) - g.targetLog[i];
            double dp = 0.0;
            if (phase)
                dp = kPhaseWeight * wrap(std::atan2(e.ni, e.nr) - std::atan2(e.di, e.dr) - g.targetArg[i]);
            // d ln H / d parameter, as (real, imaginary): 1/N or -1/D times z^-k.
            const double inr = e.nr / e.nn, ini = -e.ni / e.nn; // 1 / N
            const double idr = e.dr / e.dd, idi = -e.di / e.dd; // 1 / D
            const double q1r = g.c1[i], q1i = -g.s1[i], q2r = g.c2[i], q2i = -g.s2[i];
            double dre[5], dim[5];
            dre[0] = inr;
            dim[0] = ini * kPhaseWeight;
            dre[1] = inr * q1r - ini * q1i;
            dim[1] = (inr * q1i + ini * q1r) * kPhaseWeight;
            dre[2] = inr * q2r - ini * q2i;
            dim[2] = (inr * q2i + ini * q2r) * kPhaseWeight;
            dre[3] = -(idr * q1r - idi * q1i);
            dim[3] = -(idr * q1i + idi * q1r) * kPhaseWeight;
            dre[4] = -(idr * q2r - idi * q2i);
            dim[4] = -(idr * q2i + idi * q2r) * kPhaseWeight;
            double jr[5] = {}, ji[5] = {};
            for (int a = 0; a < count; ++a)
                for (int r = 0; r < 5; ++r)
                {
                    jr[a] += map.t[a][r] * dre[r];
                    ji[a] += map.t[a][r] * dim[r];
                }
            for (int a = 0; a < count; ++a)
            {
                jtr[a] += w2 * (jr[a] * dm + (phase ? ji[a] * dp : 0.0));
                for (int b = 0; b <= a; ++b)
                    jtj[a][b] += w2 * (jr[a] * jr[b] + (phase ? ji[a] * ji[b] : 0.0));
            }
        }
        for (int a = 0; a < count; ++a)
        {
            jtj[a][a] += kAnchor;
            jtr[a] += kAnchor * (theta[a] - anchor[a]);
        }
        for (int a = 0; a < count; ++a)
            for (int b = a + 1; b < count; ++b)
                jtj[a][b] = jtj[b][a];
        bool improved = false;
        for (int attempt = 0; attempt < 5; ++attempt)
        {
            double m[5][6];
            for (int a = 0; a < count; ++a)
            {
                for (int b = 0; b < count; ++b)
                    m[a][b] = jtj[a][b];
                m[a][a] += lambda * (jtj[a][a] + 1.0e-30);
                m[a][5] = -jtr[a];
            }
            double dx[5] = {};
            if (!solve(m, count, dx))
            {
                lambda *= 10.0;
                continue;
            }
            double next[5] = {};
            for (int a = 0; a < count; ++a)
                next[a] = theta[a] + dx[a];
            const std::array<double, 5> q = map.expand(next);
            if (stable(q))
            {
                const double c = cost(q, g, phase) + anchored(next);
                if (c < current)
                {
                    const bool small = current - c < 1.0e-9 * (current + 1.0e-30);
                    p = q;
                    for (int a = 0; a < count; ++a)
                        theta[a] = next[a];
                    current = c;
                    lambda = std::max(lambda * 0.3, 1.0e-9);
                    improved = !small;
                    break;
                }
            }
            lambda *= 8.0;
        }
        if (!improved)
            break;
    }
    bq.b0 = p[0];
    bq.b1 = p[1];
    bq.b2 = p[2];
    bq.a1 = p[3];
    bq.a2 = p[4];
    return true;
}

Biquad invert(const Biquad& s)
{
    // 1 / H: numerator (1, a1, a2) / b0, denominator (1, b1/b0, b2/b0).
    Biquad out;
    out.b0 = 1.0 / s.b0;
    out.b1 = s.a1 / s.b0;
    out.b2 = s.a2 / s.b0;
    out.a1 = s.b1 / s.b0;
    out.a2 = s.b2 / s.b0;
    return out;
}

AnalogSection reciprocal(const AnalogSection& s)
{
    AnalogSection r;
    for (int i = 0; i < 3; ++i)
    {
        r.b[i] = s.a[i];
        r.a[i] = s.b[i];
    }
    r.order = s.order;
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// The "smooth" family: the gains at DC and at Nyquist are exactly the analog ones (Orfanidis' prescribed Nyquist gain) and
// only the poles and one numerator coefficient are fitted. With the Nyquist gain pinned the error landscape has a single
// basin, so the design is a continuous function of frequency, gain and Q (the unconstrained fit has local minima, and the
// optimum jumps between them somewhere along a sweep). It costs some accuracy near Nyquist, so it is used while a parameter
// moves and the fitted design takes over when it rests.

bool numeratorFromGains(double a1, double a2, double m0, double m1, double A2, Biquad& out)
{
    const double B0 = std::pow(1.0 + a1 + a2, 2.0), B1 = std::pow(1.0 - a1 + a2, 2.0);
    const double s0 = std::sqrt(std::max(B0 * m0, 0.0)), s1 = std::sqrt(std::max(B1 * m1, 0.0));
    const double W = 0.5 * (s0 + s1);
    const double disc = W * W + A2;
    if (disc < 0.0)
        return false;
    const double r = std::sqrt(disc);
    out.a1 = a1;
    out.a2 = a2;
    out.b0 = 0.5 * (W + r);
    out.b1 = 0.5 * (s0 - s1);
    out.b2 = 0.5 * (W - r);
    minimumPhase(out.b0, out.b1, out.b2, 2);
    return true;
}

bool refineSmooth(const Biquad& seed, const Grid& g, double m0, double m1, Biquad& out)
{
    double theta[3] = {seed.a1, seed.a2, -4.0 * seed.b0 * seed.b2};
    double r0[Grid::kMax], r1[Grid::kMax];
    const auto residuals = [&](const double (&t)[3], double* r, Biquad& b)
    {
        if (!numeratorFromGains(t[0], t[1], m0, m1, t[2], b))
            return false;
        const std::array<double, 5> p = {b.b0, b.b1, b.b2, b.a1, b.a2};
        if (!stable(p))
            return false;
        for (int i = 0; i < g.count; ++i)
            r[i] = g.weight[i] == 0.0 ? 0.0 : g.weight[i] * (logMagnitude(evalPoint(p, g, i)) - g.targetLog[i]);
        return true;
    };
    const auto sumsq = [&](const double* r)
    {
        double c = 0.0;
        for (int i = 0; i < g.count; ++i)
            c += r[i] * r[i];
        return c;
    };
    Biquad b;
    if (!residuals(theta, r0, b))
        return false;
    double current = sumsq(r0);
    double lambda = 1.0e-3;
    for (int iteration = 0; iteration < 5; ++iteration)
    {
        double jac[Grid::kMax][3];
        for (int k = 0; k < 3; ++k)
        {
            const double h = 1.0e-6 * (k == 2 ? std::max(1.0, std::abs(theta[2])) : 1.0);
            double t[3] = {theta[0], theta[1], theta[2]};
            t[k] += h;
            if (!residuals(t, r1, b))
            {
                t[k] -= 2.0 * h;
                if (!residuals(t, r1, b))
                    return false;
                for (int i = 0; i < g.count; ++i)
                    jac[i][k] = (r0[i] - r1[i]) / h;
                continue;
            }
            for (int i = 0; i < g.count; ++i)
                jac[i][k] = (r1[i] - r0[i]) / h;
        }
        double jtj[3][3] = {}, jtr[3] = {};
        for (int i = 0; i < g.count; ++i)
            for (int a = 0; a < 3; ++a)
            {
                jtr[a] += jac[i][a] * r0[i];
                for (int c = 0; c < 3; ++c)
                    jtj[a][c] += jac[i][a] * jac[i][c];
            }
        bool improved = false;
        for (int attempt = 0; attempt < 5 && !improved; ++attempt)
        {
            double m[5][6] = {};
            for (int a = 0; a < 3; ++a)
            {
                for (int c = 0; c < 3; ++c)
                    m[a][c] = jtj[a][c];
                m[a][a] += lambda * (jtj[a][a] + 1.0e-30);
                m[a][5] = -jtr[a];
            }
            double dx[5] = {};
            if (!solve(m, 3, dx))
            {
                lambda *= 10.0;
                continue;
            }
            const double t[3] = {theta[0] + dx[0], theta[1] + dx[1], theta[2] + dx[2]};
            if (residuals(t, r1, b))
            {
                const double c = sumsq(r1);
                if (c < current)
                {
                    const bool small = current - c < 1.0e-10 * (current + 1.0e-30);
                    theta[0] = t[0];
                    theta[1] = t[1];
                    theta[2] = t[2];
                    std::copy(r1, r1 + g.count, r0);
                    current = c;
                    lambda = std::max(lambda * 0.3, 1.0e-9);
                    improved = !small;
                    if (small)
                        attempt = 99;
                    break;
                }
            }
            lambda *= 8.0;
        }
        if (!improved)
            break;
    }
    return numeratorFromGains(theta[0], theta[1], m0, m1, theta[2], out);
}

// ---------------------------------------------------------------------------------------------------------------
// Bessel poles: roots of the reverse Bessel polynomials, scaled to a -3 dB frequency of 1.

struct SectionData
{
    double omega, q; // pole magnitude (relative to the -3 dB frequency) and quality factor; q = 0 marks a real pole
};

struct FilterTable
{
    int count[17] = {};
    SectionData section[17][8];
    explicit FilterTable(bool bessel)
    {
        for (int n = 1; n <= 16; ++n)
        {
            if (!bessel)
            {
                int k = 0;
                for (int i = 1; i <= n / 2; ++i)
                    section[n][k++] = {1.0, 1.0 / (2.0 * std::sin((2 * i - 1) * kPi / (2.0 * n)))};
                if (n % 2 == 1)
                    section[n][k++] = {1.0, 0.0};
                count[n] = k;
                continue;
            }
            // Reverse Bessel polynomial coefficients c[k] of s^k.
            double c[17];
            for (int k = 0; k <= n; ++k)
            {
                double num = 1.0, den = 1.0;
                for (int i = 1; i <= 2 * n - k; ++i)
                    num *= i;
                for (int i = 1; i <= n - k; ++i)
                    den *= 2.0 * i;
                for (int i = 1; i <= k; ++i)
                    den *= i;
                c[k] = num / den;
            }
            // Durand-Kerner on the monic polynomial.
            cd root[16];
            for (int i = 0; i < n; ++i)
                root[i] = std::polar(1.0 + 0.1 * i, 2.0 * kPi * (i + 0.3) / n);
            for (int it = 0; it < 500; ++it)
            {
                double move = 0.0;
                for (int i = 0; i < n; ++i)
                {
                    cd value = c[n];
                    for (int k = n - 1; k >= 0; --k)
                        value = value * root[i] + c[k];
                    cd denom = c[n];
                    for (int j = 0; j < n; ++j)
                        if (j != i)
                            denom *= root[i] - root[j];
                    const cd step = value / denom;
                    root[i] -= step;
                    move = std::max(move, std::abs(step));
                }
                if (move < 1.0e-15)
                    break;
            }
            // -3 dB frequency of H(s) = prod(-p) / prod(s - p).
            const auto mag = [&](double w)
            {
                double m = 1.0;
                for (int i = 0; i < n; ++i)
                    m *= std::abs(root[i]) / std::abs(cd(0.0, w) - root[i]);
                return m;
            };
            double lo = 0.0, hi = 20.0;
            for (int it = 0; it < 80; ++it)
            {
                const double mid = 0.5 * (lo + hi);
                (mag(mid) > 0.70710678118654752 ? lo : hi) = mid;
            }
            const double w3 = 0.5 * (lo + hi);
            int k = 0;
            for (int i = 0; i < n; ++i)
            {
                const cd p = root[i] / w3;
                if (std::abs(p.imag()) < 1.0e-9)
                    section[n][k++] = {std::abs(p), 0.0};
                else if (p.imag() > 0.0)
                    section[n][k++] = {std::abs(p), std::abs(p) / (-2.0 * p.real())};
            }
            // Highest quality factor first.
            std::sort(section[n], section[n] + k, [](const SectionData& a, const SectionData& b) { return a.q > b.q; });
            count[n] = k;
        }
    }
};

const FilterTable& butterworthTable()
{
    static const FilterTable t(false);
    return t;
}
const FilterTable& besselTable()
{
    static const FilterTable t(true);
    return t;
}

AnalogSection lowPassSection(const SectionData& d)
{
    AnalogSection s;
    s.b[0] = s.b[1] = s.b[2] = s.a[0] = s.a[1] = s.a[2] = 0.0;
    if (d.q == 0.0)
    {
        s.order = 1;
        s.b[0] = d.omega;
        s.a[0] = d.omega;
        s.a[1] = 1.0;
    }
    else
    {
        s.order = 2;
        s.b[0] = d.omega * d.omega;
        s.a[0] = d.omega * d.omega;
        s.a[1] = d.omega / d.q;
        s.a[2] = 1.0;
    }
    return s;
}

AnalogSection highPassSection(const SectionData& d)
{
    AnalogSection s;
    s.b[0] = s.b[1] = s.b[2] = s.a[0] = s.a[1] = s.a[2] = 0.0;
    const double w = 1.0 / d.omega;
    if (d.q == 0.0)
    {
        s.order = 1;
        s.b[1] = 1.0;
        s.a[0] = w;
        s.a[1] = 1.0;
    }
    else
    {
        s.order = 2;
        s.b[2] = 1.0;
        s.a[0] = w * w;
        s.a[1] = w / d.q;
        s.a[2] = 1.0;
    }
    return s;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
std::complex<double> biquadResponse(const Biquad& s, double omega)
{
    const cd q1 = std::polar(1.0, -omega), q2 = q1 * q1;
    return (s.b0 + s.b1 * q1 + s.b2 * q2) / (1.0 + s.a1 * q1 + s.a2 * q2);
}

std::complex<double> analogResponse(const AnalogSection& s, double f)
{
    const cd jw(0.0, f);
    return polyAt(s.b, jw) / polyAt(s.a, jw);
}

AnalogSection analogPrototype(SectionKind kind, double gainDb, double q, int order)
{
    AnalogSection s;
    s.b[0] = s.b[1] = s.b[2] = 0.0;
    s.a[0] = s.a[1] = s.a[2] = 0.0;
    s.order = order <= 1 ? 1 : 2;
    q = std::max(q, 0.05);
    const double A = std::pow(10.0, gainDb / 40.0); // sqrt of the linear gain
    const double G = A * A;
    switch (kind)
    {
    case SectionKind::Bell:
        s.order = 2;
        s.b[0] = 1.0;
        s.b[1] = A / q;
        s.b[2] = 1.0;
        s.a[0] = 1.0;
        s.a[1] = 1.0 / (A * q);
        s.a[2] = 1.0;
        break;
    case SectionKind::LowShelf:
        if (s.order == 1)
        {
            s.b[0] = A;
            s.b[1] = 1.0;
            s.a[0] = 1.0 / A;
            s.a[1] = 1.0;
        }
        else
        {
            const double r = std::sqrt(A) / q;
            s.b[0] = A * A;
            s.b[1] = A * r;
            s.b[2] = A;
            s.a[0] = 1.0;
            s.a[1] = r;
            s.a[2] = A;
        }
        break;
    case SectionKind::HighShelf:
        if (s.order == 1)
        {
            s.b[0] = A;
            s.b[1] = G;
            s.a[0] = A;
            s.a[1] = 1.0;
        }
        else
        {
            const double r = std::sqrt(A) / q;
            s.b[0] = A;
            s.b[1] = A * r;
            s.b[2] = A * A;
            s.a[0] = A;
            s.a[1] = r;
            s.a[2] = 1.0;
        }
        break;
    case SectionKind::Tilt:
        s.order = 1;
        s.b[0] = 1.0;
        s.b[1] = A;
        s.a[0] = A;
        s.a[1] = 1.0;
        break;
    case SectionKind::BandPass:
        s.order = 2;
        s.b[1] = 1.0 / q;
        s.a[0] = 1.0;
        s.a[1] = 1.0 / q;
        s.a[2] = 1.0;
        break;
    case SectionKind::Notch:
        s.order = 2;
        s.b[0] = 1.0;
        s.b[2] = 1.0;
        s.a[0] = 1.0;
        s.a[1] = 1.0 / q;
        s.a[2] = 1.0;
        break;
    case SectionKind::AllPass:
        if (s.order == 1)
        {
            s.b[0] = 1.0;
            s.b[1] = -1.0;
            s.a[0] = 1.0;
            s.a[1] = 1.0;
        }
        else
        {
            s.b[0] = 1.0;
            s.b[1] = -1.0 / q;
            s.b[2] = 1.0;
            s.a[0] = 1.0;
            s.a[1] = 1.0 / q;
            s.a[2] = 1.0;
        }
        break;
    case SectionKind::LowPass:
        if (s.order == 1)
        {
            s.b[0] = 1.0;
            s.a[0] = 1.0;
            s.a[1] = 1.0;
        }
        else
        {
            s.b[0] = 1.0;
            s.a[0] = 1.0;
            s.a[1] = 1.0 / q;
            s.a[2] = 1.0;
        }
        break;
    case SectionKind::HighPass:
        if (s.order == 1)
        {
            s.b[1] = 1.0;
            s.a[0] = 1.0;
            s.a[1] = 1.0;
        }
        else
        {
            s.b[2] = 1.0;
            s.a[0] = 1.0;
            s.a[1] = 1.0 / q;
            s.a[2] = 1.0;
        }
        break;
    }
    return s;
}

Biquad designSection(const AnalogSection& proto, double centreHz, double sampleRate, DesignQuality quality, bool cut)
{
    centreHz = std::clamp(centreHz, 0.01, 0.49 * sampleRate);
    const double w0T = 2.0 * kPi * centreHz / sampleRate;
    const bool firstOrder = proto.order <= 1;
    const bool allPass =
        proto.order == 2 && proto.b[0] == proto.a[0] && proto.b[1] == -proto.a[1] && proto.b[2] == proto.a[2];
    if (allPass)
        return bilinear(proto, w0T);
    const bool notch = proto.order == 2 && proto.b[1] == 0.0 && proto.b[0] > 0.0 && proto.b[0] == proto.b[2];
    const bool bandPass = proto.order == 2 && proto.b[0] == 0.0 && proto.b[2] == 0.0;
    const bool highPass = proto.order == 2 ? (proto.b[0] == 0.0 && proto.b[1] == 0.0 && proto.b[2] != 0.0)
                                           : (proto.b[0] == 0.0 && proto.b[1] != 0.0);

    Biquad seed;
    bool ok = false;
    if (cut)
    {
        Biquad boost;
        if (closedForm(reciprocal(proto), w0T, boost) && std::abs(boost.b0) > 1.0e-300)
        {
            seed = invert(boost);
            ok = seed.stable();
        }
    }
    else if (notch)
    {
        // Poles matched, zeros exactly on the unit circle at the centre, gain from the value at DC.
        Biquad d;
        AnalogSection flat = proto;
        flat.b[0] = 1.0;
        flat.b[1] = 0.0;
        flat.b[2] = 0.0; // only the poles are used
        if (closedForm(flat, w0T, d))
        {
            const double c = std::cos(w0T);
            const double B0 = std::pow(1.0 + d.a1 + d.a2, 2.0);
            const double g = std::sqrt(B0) / (2.0 - 2.0 * c);
            seed = d;
            seed.b0 = g;
            seed.b1 = -2.0 * c * g;
            seed.b2 = g;
            ok = seed.stable();
        }
    }
    else
        ok = closedForm(proto, w0T, seed);
    if (!ok)
        seed = bilinear(proto, w0T);
    if (quality == DesignQuality::Matched)
        return seed;

    Grid g;
    if (quality == DesignQuality::Smooth)
    {
        // Bells and second-order shelves only; everything else is already a smooth function of its parameters in closed
        // form.
        const bool lowPass = proto.order == 2 && proto.b[1] == 0.0 && proto.b[2] == 0.0;
        if (proto.order != 2 || notch || bandPass || highPass || lowPass)
            return seed;
        const AnalogSection boost = cut ? reciprocal(proto) : proto;
        Biquad boostSeed = seed;
        if (cut && !closedForm(boost, w0T, boostSeed))
            boostSeed = bilinear(boost, w0T);
        buildGrid(g, boost, w0T, sampleRate);
        const double m0 = std::norm(analogResponse(boost, 0.0)), m1 = std::norm(analogResponse(boost, kPi / w0T));
        Biquad smooth;
        // Resonant shelves cannot keep their Nyquist gain: those fall through to the ordinary fit below.
        if (refineSmooth(boostSeed, g, m0, m1, smooth) && smooth.stable() &&
            coreErrorDb({smooth.b0, smooth.b1, smooth.b2, smooth.a1, smooth.a2}, g) < 2.0)
            return cut ? invert(smooth) : smooth;
        quality = DesignQuality::Minimum;
    }

    // Refinement is worth its cost only where the closed form is visibly off.
    buildGrid(g, proto, w0T, sampleRate);
    const double seedError = coreErrorDb({seed.b0, seed.b1, seed.b2, seed.a1, seed.a2}, g);
    // The correction is faded in between 0.004 and 0.02 dB of seed error, so that a parameter sweep never steps between the
    // closed form and the fit.
    const double weight = quality == DesignQuality::Minimum ? std::clamp((seedError - 0.004) / 0.016, 0.0, 1.0) : 1.0;
    if (weight <= 0.0)
        return seed;

    Parametrisation map = Parametrisation::free(proto.order);
    if (!cut)
    {
        if (highPass)
            map = firstOrder ? Parametrisation::tied(1, 1.0, -1.0, 0.0) : Parametrisation::tied(2, 1.0, -2.0, 1.0);
        else if (bandPass)
            map = Parametrisation::bandPass();
        else if (notch)
            map = Parametrisation::tied(2, 1.0, -2.0 * std::cos(w0T), 1.0);
    }
    const bool natural = quality == DesignQuality::Natural;
    const auto fit = [&](const Biquad& start, double& error)
    {
        Biquad f = start;
        refine(f, g, natural, map);
        if (!natural && !(notch || bandPass || highPass))
            minimumPhase(f.b0, f.b1, f.b2, firstOrder ? 1 : 2);
        if (!f.stable() || !std::isfinite(f.b0 + f.b1 + f.b2 + f.a1 + f.a2))
        {
            error = 1.0e9;
            return start;
        }
        error = coreErrorDb({f.b0, f.b1, f.b2, f.a1, f.a2}, g);
        return f;
    };
    double error = 0.0;
    Biquad best = fit(seed, error);
    if (error > 0.25)
    {
        // The fit settled in a poor local minimum (extreme shelves near Nyquist): try the other starting points.
        const Biquad warp = bilinear(proto, w0T);
        Biquad direct;
        const bool haveDirect = cut && closedForm(proto, w0T, direct);
        const Biquad* const starts[2] = {haveDirect ? &direct : nullptr, &warp};
        for (const Biquad* start : starts)
        {
            if (start == nullptr || !start->stable())
                continue;
            double e = 0.0;
            const Biquad f = fit(*start, e);
            if (e < error)
            {
                error = e;
                best = f;
            }
        }
    }
    // A fit that ended up clearly worse than its starting point is dropped.
    if (!natural && error > seedError + 0.05)
        return seed;
    if (weight >= 1.0)
        return best;
    Biquad out;
    out.b0 = seed.b0 + weight * (best.b0 - seed.b0);
    out.b1 = seed.b1 + weight * (best.b1 - seed.b1);
    out.b2 = seed.b2 + weight * (best.b2 - seed.b2);
    out.a1 = seed.a1 + weight * (best.a1 - seed.a1);
    out.a2 = seed.a2 + weight * (best.a2 - seed.a2);
    return out;
}

int butterworthPoles(int order, PolePair* out)
{
    order = std::clamp(order, 1, 16);
    const FilterTable& t = butterworthTable();
    int k = 0;
    for (int i = 0; i < t.count[order]; ++i)
    {
        out[k].q = t.section[order][i].q;
        out[k].real = t.section[order][i].q == 0.0;
        ++k;
    }
    // The table lists the pairs first and the real pole last, which is the order promised.
    return k;
}

Biquad bilinearSection(const AnalogSection& prototype, double centreHz, double sampleRate)
{
    centreHz = std::clamp(centreHz, 0.01, 0.49 * sampleRate);
    return bilinear(prototype, 2.0 * kPi * centreHz / sampleRate);
}

double sectionError(const Biquad& digital, const AnalogSection& proto, double centreHz, double sampleRate, double lowHz,
                    double highHz)
{
    centreHz = std::clamp(centreHz, 0.01, 0.49 * sampleRate);
    double worst = 0.0;
    const double nyquist = 0.5 * sampleRate;
    highHz = std::min(highHz, 0.9999 * nyquist);
    const int n = 600;
    for (int i = 0; i <= n; ++i)
    {
        const double f = lowHz * std::pow(highHz / lowHz, static_cast<double>(i) / n);
        const double omega = 2.0 * kPi * f / sampleRate;
        const double a = 20.0 * std::log10(std::max(std::abs(analogResponse(proto, f / centreHz)), 1.0e-12));
        const double d = biquadMagnitudeDb(digital, omega);
        if (a > -80.0)
            worst = std::max(worst, std::abs(a - d));
    }
    // Dense around the centre, where narrow features live.
    for (int i = -200; i <= 200; ++i)
    {
        const double f = centreHz * std::pow(2.0, i / 100.0 * 0.5);
        if (f < lowHz || f > highHz)
            continue;
        const double omega = 2.0 * kPi * f / sampleRate;
        const double a = 20.0 * std::log10(std::max(std::abs(analogResponse(proto, f / centreHz)), 1.0e-12));
        if (a > -80.0)
            worst = std::max(worst, std::abs(a - biquadMagnitudeDb(digital, omega)));
    }
    return worst;
}

// ---------------------------------------------------------------------------------------------------------------
BandPrototype bandPrototype(const BandSpec& spec)
{
    BandPrototype out;
    const auto add = [&](const AnalogSection& s, double scale = 1.0)
    {
        if (out.count < kMaxBandSections)
            out.part[static_cast<size_t>(out.count++)] = {s, scale, 1.0};
    };
    const double q = std::clamp(spec.q, 0.05, 100.0);
    switch (spec.type)
    {
    case BandType::Bell:
        add(analogPrototype(SectionKind::Bell, spec.gainDb, q));
        break;
    case BandType::LowShelf:
        add(analogPrototype(SectionKind::LowShelf, spec.gainDb, q, spec.slope == 0 ? 1 : 2));
        break;
    case BandType::HighShelf:
        add(analogPrototype(SectionKind::HighShelf, spec.gainDb, q, spec.slope == 0 ? 1 : 2));
        break;
    case BandType::Tilt:
        add(analogPrototype(SectionKind::Tilt, spec.gainDb, q, 1));
        break;
    case BandType::BandPass:
        add(analogPrototype(SectionKind::BandPass, 0.0, q));
        break;
    case BandType::Notch:
        add(analogPrototype(SectionKind::Notch, 0.0, q));
        break;
    case BandType::AllPass:
        add(analogPrototype(SectionKind::AllPass, 0.0, q));
        break;
    case BandType::LowCut:
    case BandType::HighCut:
    {
        const bool high = spec.type == BandType::LowCut;
        int order = cutOrder(spec.slope);
        CutShape shape = spec.shape;
        if (shape == CutShape::LinkwitzRiley && (order % 2 != 0))
            shape = CutShape::Butterworth;
        const auto sectionFor = [&](const SectionData& d) { return high ? highPassSection(d) : lowPassSection(d); };
        if (shape == CutShape::LinkwitzRiley)
        {
            const FilterTable& t = butterworthTable();
            const int half = order / 2;
            for (int rep = 0; rep < 2; ++rep)
                for (int i = 0; i < t.count[half]; ++i)
                    add(sectionFor(t.section[half][i]));
        }
        else
        {
            const FilterTable& t = shape == CutShape::Bessel ? besselTable() : butterworthTable();
            for (int i = 0; i < t.count[order]; ++i)
            {
                SectionData d = t.section[order][i];
                if (shape == CutShape::Resonant && i == 0 && d.q != 0.0 && order >= 2)
                    d.q = std::max(q, 0.1);
                add(sectionFor(d));
            }
        }
        break;
    }
    }
    return out;
}

std::complex<double> bandAnalogResponse(const BandPrototype& p, const BandSpec& spec, double hz)
{
    cd h = 1.0;
    for (int i = 0; i < p.count; ++i)
    {
        const auto& part = p.part[static_cast<size_t>(i)];
        h *= analogResponse(part.section, hz / (spec.freqHz * part.freqScale)) * part.gain;
    }
    return h;
}

double bandAnalogMagnitudeDb(const BandPrototype& p, const BandSpec& spec, double hz)
{
    return 20.0 * std::log10(std::max(std::abs(bandAnalogResponse(p, spec, hz)), 1.0e-12));
}

std::complex<double> BandFilter::response(double omega) const
{
    cd h = gain;
    for (int i = 0; i < count; ++i)
        h *= biquadResponse(section[static_cast<size_t>(i)], omega);
    return h;
}

double BandFilter::magnitudeDb(double omega) const
{
    return 20.0 * std::log10(std::max(std::abs(response(omega)), 1.0e-12));
}

BandFilter designBand(const BandSpec& spec, double sampleRate, DesignQuality quality)
{
    return designBand(spec, bandPrototype(spec), sampleRate, quality);
}

BandFilter designBand(const BandSpec& spec, const BandPrototype& prototype, double sampleRate, DesignQuality quality)
{
    BandFilter out;
    const bool gainType = spec.type == BandType::Bell || spec.type == BandType::LowShelf ||
                          spec.type == BandType::HighShelf || spec.type == BandType::Tilt;
    if (gainType && spec.gainDb == 0.0)
        return out;
    const bool cut = gainType && spec.gainDb < 0.0;
    for (int i = 0; i < prototype.count; ++i)
    {
        const auto& part = prototype.part[static_cast<size_t>(i)];
        out.section[static_cast<size_t>(out.count++)] =
            designSection(part.section, spec.freqHz * part.freqScale, sampleRate, quality, cut);
        out.gain *= part.gain;
    }
    return out;
}

// ---------------------------------------------------------------------------------------------------------------
namespace
{
inline double flushTiny(double v)
{
    return std::abs(v) < 1.0e-30 ? 0.0 : v;
}
} // namespace

namespace
{
bool isIdentityTarget(const double (&t)[5])
{
    return t[0] == 1.0 && t[1] == 0.0 && t[2] == 0.0 && t[3] == 0.0 && t[4] == 0.0;
}
} // namespace

void BiquadChain::set(const BandFilter& filter, int rampSamples)
{
    set(filter.section.data(), filter.count, rampSamples);
}

void BiquadChain::set(const Biquad* sections, int count, int rampSamples)
{
    const Biquad identity;
    for (int i = 0; i < kMaxBandSections; ++i)
    {
        Stage& s = stage[static_cast<size_t>(i)];
        const Biquad& t = i < count ? sections[i] : identity;
        const double goal[5] = {t.b0, t.b1, t.b2, t.a1, t.a2};
        bool changed = false;
        for (int k = 0; k < 5; ++k)
            changed = changed || goal[k] != s.target[k];
        if (!changed)
            continue;
        for (int k = 0; k < 5; ++k)
        {
            s.target[k] = goal[k];
            s.step[k] = rampSamples > 0 ? (goal[k] - s.c[k]) / rampSamples : 0.0;
        }
        s.remaining = rampSamples > 0 ? rampSamples : 0;
        if (rampSamples <= 0)
            for (int k = 0; k < 5; ++k)
                s.c[k] = goal[k];
    }
    refreshActive();
}

void BiquadChain::refreshActive()
{
    int top = 0;
    for (int i = 0; i < kMaxBandSections; ++i)
    {
        const Stage& s = stage[static_cast<size_t>(i)];
        if (!(isIdentityTarget(s.target) && s.remaining == 0))
            top = i + 1;
    }
    active = top;
}

void BiquadChain::reset()
{
    for (auto& s : stage)
    {
        for (int k = 0; k < 5; ++k)
        {
            s.c[k] = s.target[k];
            s.step[k] = 0.0;
        }
        s.remaining = 0;
        for (int l = 0; l < 2; ++l)
            s.x1[l] = s.x2[l] = s.y1[l] = s.y2[l] = 0.0;
    }
    refreshActive();
}

void BiquadChain::processStage(Stage& s, double* a, double* b, int n, int lanes) noexcept
{
    double* lane[2] = {a, b};
    const int ramp = std::min(n, s.remaining);
    // Both lanes in one loop: the two recursions are independent, so their latencies overlap (the same arithmetic per
    // lane as the single-lane loop below).
    const bool both = (lanes & 3) == 3;
    if (both)
    {
        double xa1 = s.x1[0], xa2 = s.x2[0], ya1 = s.y1[0], ya2 = s.y2[0];
        double xb1 = s.x1[1], xb2 = s.x2[1], yb1 = s.y1[1], yb2 = s.y2[1];
        double c0 = s.c[0], c1 = s.c[1], c2 = s.c[2], c3 = s.c[3], c4 = s.c[4];
        int i = 0;
        for (; i < ramp; ++i)
        {
            c0 += s.step[0];
            c1 += s.step[1];
            c2 += s.step[2];
            c3 += s.step[3];
            c4 += s.step[4];
            const double inA = a[i], inB = b[i];
            const double yA = flushTiny(c0 * inA + c1 * xa1 + c2 * xa2 - c3 * ya1 - c4 * ya2);
            const double yB = flushTiny(c0 * inB + c1 * xb1 + c2 * xb2 - c3 * yb1 - c4 * yb2);
            xa2 = xa1;
            xa1 = inA;
            ya2 = ya1;
            ya1 = yA;
            xb2 = xb1;
            xb1 = inB;
            yb2 = yb1;
            yb1 = yB;
            a[i] = yA;
            b[i] = yB;
        }
        if (ramp > 0 && ramp == s.remaining)
        {
            c0 = s.target[0];
            c1 = s.target[1];
            c2 = s.target[2];
            c3 = s.target[3];
            c4 = s.target[4];
        }
        for (; i < n; ++i)
        {
            const double inA = a[i], inB = b[i];
            const double yA = flushTiny(c0 * inA + c1 * xa1 + c2 * xa2 - c3 * ya1 - c4 * ya2);
            const double yB = flushTiny(c0 * inB + c1 * xb1 + c2 * xb2 - c3 * yb1 - c4 * yb2);
            xa2 = xa1;
            xa1 = inA;
            ya2 = ya1;
            ya1 = yA;
            xb2 = xb1;
            xb1 = inB;
            yb2 = yb1;
            yb1 = yB;
            a[i] = yA;
            b[i] = yB;
        }
        s.x1[0] = xa1;
        s.x2[0] = xa2;
        s.y1[0] = ya1;
        s.y2[0] = ya2;
        s.x1[1] = xb1;
        s.x2[1] = xb2;
        s.y1[1] = yb1;
        s.y2[1] = yb2;
    }
    for (int l = 0; l < 2; ++l)
    {
        if (both || (lanes & (1 << l)) == 0)
            continue;
        double* x = lane[l];
        double x1 = s.x1[l], x2 = s.x2[l], y1 = s.y1[l], y2 = s.y2[l];
        double c0 = s.c[0], c1 = s.c[1], c2 = s.c[2], c3 = s.c[3], c4 = s.c[4];
        int i = 0;
        for (; i < ramp; ++i)
        {
            c0 += s.step[0];
            c1 += s.step[1];
            c2 += s.step[2];
            c3 += s.step[3];
            c4 += s.step[4];
            const double in = x[i];
            const double y = flushTiny(c0 * in + c1 * x1 + c2 * x2 - c3 * y1 - c4 * y2);
            x2 = x1;
            x1 = in;
            y2 = y1;
            y1 = y;
            x[i] = y;
        }
        if (ramp > 0 && ramp == s.remaining)
        {
            c0 = s.target[0];
            c1 = s.target[1];
            c2 = s.target[2];
            c3 = s.target[3];
            c4 = s.target[4];
        }
        for (; i < n; ++i)
        {
            const double in = x[i];
            const double y = flushTiny(c0 * in + c1 * x1 + c2 * x2 - c3 * y1 - c4 * y2);
            x2 = x1;
            x1 = in;
            y2 = y1;
            y1 = y;
            x[i] = y;
        }
        s.x1[l] = x1;
        s.x2[l] = x2;
        s.y1[l] = y1;
        s.y2[l] = y2;
    }
    // The coefficients after this block, the same for both lanes.
    if (ramp > 0)
    {
        if (ramp == s.remaining)
            for (int k = 0; k < 5; ++k)
                s.c[k] = s.target[k];
        else
            for (int k = 0; k < 5; ++k)
                s.c[k] += s.step[k] * ramp;
        s.remaining -= ramp;
    }
}

void BiquadChain::process(double* a, double* b, int n, int lanes) noexcept
{
    for (int i = 0; i < active; ++i)
    {
        Stage& s = stage[static_cast<size_t>(i)];
        if (isIdentityTarget(s.target) && s.remaining == 0)
            continue;
        processStage(s, a, b, n, lanes);
    }
    refreshActive();
}

// ---------------------------------------------------------------------------------------------------------------
void TptSvf::set(Mode mode, double sampleRate, double hz, double q, double gainDb)
{
    hz = std::clamp(hz, 1.0, 0.49 * sampleRate);
    q = std::max(q, 0.05);
    const double A = std::pow(10.0, gainDb / 40.0);
    g = std::tan(kPi * hz / sampleRate);
    k = 1.0 / q;
    switch (mode)
    {
    case Mode::LowPass:
        m0 = 0.0;
        m1 = 0.0;
        m2 = 1.0;
        break;
    case Mode::BandPass:
        m0 = 0.0;
        m1 = k;
        m2 = 0.0;
        break;
    case Mode::HighPass:
        m0 = 1.0;
        m1 = -k;
        m2 = -1.0;
        break;
    case Mode::Notch:
        m0 = 1.0;
        m1 = -k;
        m2 = 0.0;
        break;
    case Mode::AllPass:
        m0 = 1.0;
        m1 = -2.0 * k;
        m2 = 0.0;
        break;
    case Mode::Bell:
        k = 1.0 / (q * A);
        m0 = 1.0;
        m1 = k * (A * A - 1.0);
        m2 = 0.0;
        break;
    case Mode::LowShelf:
        g /= std::sqrt(A);
        m0 = 1.0;
        m1 = k * (A - 1.0);
        m2 = A * A - 1.0;
        break;
    case Mode::HighShelf:
        g *= std::sqrt(A);
        m0 = A * A;
        m1 = k * (1.0 - A) * A;
        m2 = 1.0 - A * A;
        break;
    }
    a1 = 1.0 / (1.0 + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
}

std::complex<double> TptSvf::response(double omega) const
{
    const cd s = cd(0.0, std::tan(0.5 * omega) / g); // normalised analog variable of the bilinear map
    const cd d = s * s + k * s + 1.0;
    return m0 + m1 * s / d + m2 / d;
}

} // namespace crucible::fx
