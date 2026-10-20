// SPDX-License-Identifier: MIT
#pragma once

// Second-order sections whose magnitude follows the analog prototype of the same curve up to 20 kHz, at any sample rate
// (docs/plans/ENGINE-FX-PLAN.md, quality item Q4, and section 3.3). The usual EQ designs (the bilinear transform with the
// centre frequency pre-warped) match only the centre: a bell near 16 kHz at 44.1 kHz loses several dB of its gain and
// bandwidth because the response above it is squeezed into the last few kHz ("cramping").
//
// Design in three steps, all in double precision:
//
//  1. Poles. The analog poles are mapped with the matched z-transform, z = exp(s T). The mapping keeps the resonance
//     (centre frequency and damping) at every sample rate, which pre-warping cannot do for the whole curve.
//  2. Numerator, closed form. The squared magnitude of a biquad is |H|^2 = (A0 phi0 + A1 phi1 + A2 phi2) / (B0 phi0 + B1
//     phi1 + B2 phi2) with A0 = (b0 + b1 + b2)^2, A1 = (b0 - b1 + b2)^2, A2 = -4 b0 b2, phi0 = cos^2(w/2), phi1 =
//     sin^2(w/2), phi2 = 4 phi0 phi1 (M. Vicanek, "Matched Second Order Digital Filters", 2016; the same identity
//     underlies Orfanidis, "Digital Parametric Equalizer Design with Prescribed Nyquist-Frequency Gain", JAES 45(6),
//     1997). Fixing the poles, the numerator that reproduces the analog magnitude at DC, at the centre and at Nyquist
//     follows from three squares and one root; the minimum-phase factor is kept. A cut is the exact inverse of the
//     boost of the same size, so it is designed as a boost and inverted. If the three values cannot be met by a real
//     numerator the pre-warped bilinear design stands in as the starting point of the next step.
//  3. Refinement. Three points cannot pin a curve that the digital response has to follow up to 20 kHz while it must be
//     flat at Nyquist, so a few Levenberg-Marquardt steps (analytic Jacobian of the complex logarithm of the response,
//     5 parameters, about 30 frequency points) fit the magnitude (Minimum) or magnitude and phase (Natural) to the analog
//     prototype between 20 Hz and 20 kHz; the correction fades in between 0.004 and 0.02 dB of closed-form error, so a sweep
//     never steps between the two, and the step is skipped away from Nyquist where the closed form is already exact.
//     A fourth variant, Smooth, pins the Nyquist gain and fits only the poles and one zero: the unconstrained fit has local
//     minima along which the optimum jumps as a parameter moves, the pinned one has a single basin and is continuous, at
//     the price of some accuracy near Nyquist. The pro EQ uses it while a band moves and the fit when it rests.
//
// The analog prototypes are the Audio EQ Cookbook (R. Bristow-Johnson) analog filters for bells and shelves, so cuts mirror
// boosts exactly; Butterworth, Bessel and Linkwitz-Riley cascades give the cut filters. Everything here is control-thread
// or audio-thread safe (no allocation); designs cost a few microseconds.

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <vector>

namespace crucible::fx
{

// H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2), double precision.
struct Biquad
{
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;

    bool isIdentity() const { return b0 == 1.0 && b1 == 0.0 && b2 == 0.0 && a1 == 0.0 && a2 == 0.0; }
    // Poles inside the unit circle with a margin.
    bool stable() const { return std::abs(a2) < 1.0 && std::abs(a1) < 1.0 + a2; }
};

// Complex response at the normalised angular frequency omega = 2 pi f / fs.
std::complex<double> biquadResponse(const Biquad& s, double omega);
// Squared magnitude from cos(omega): a dozen operations, for evaluating a response on a dense grid.
inline double biquadMagnitudeSquared(const Biquad& s, double cosOmega)
{
    const double c2 = 2.0 * cosOmega * cosOmega - 1.0;
    const double n =
        s.b0 * s.b0 + s.b1 * s.b1 + s.b2 * s.b2 + 2.0 * (s.b0 * s.b1 + s.b1 * s.b2) * cosOmega + 2.0 * s.b0 * s.b2 * c2;
    const double d = 1.0 + s.a1 * s.a1 + s.a2 * s.a2 + 2.0 * (s.a1 + s.a1 * s.a2) * cosOmega + 2.0 * s.a2 * c2;
    return d > 1.0e-300 ? n / d : 0.0;
}
inline double biquadMagnitudeDb(const Biquad& s, double omega)
{
    return 20.0 * std::log10(std::max(std::abs(biquadResponse(s, omega)), 1.0e-12));
}

// Analog prototype with the frequency normalised to the section's own frequency (s = j f / f0):
// H(s) = (b0 + b1 s + b2 s^2) / (a0 + a1 s + a2 s^2); a section of order 1 has b2 = a2 = 0.
struct AnalogSection
{
    double b[3] = {1.0, 0.0, 0.0};
    double a[3] = {1.0, 0.0, 0.0};
    int order = 2;
};
std::complex<double> analogResponse(const AnalogSection& s, double fOverF0);

enum class SectionKind
{
    Bell,
    LowShelf,
    HighShelf,
    Tilt, // first order, gain g at high frequencies and -g at low frequencies; 0 dB at f0
    BandPass,
    Notch,
    AllPass,
    LowPass,
    HighPass
};

// order 1: first-order shelf / low-pass / high-pass (6 dB per octave), q is ignored. Otherwise second order with
// resonance q (shelves: the cookbook Q; 0.7071 is the maximally flat shelf).
AnalogSection analogPrototype(SectionKind kind, double gainDb, double q, int order = 2);

// The sections of the Butterworth filter of the given order (1..16) with the -3 dB frequency normalised to 1: the quality
// factor of every pole pair, highest first, then the real pole of an odd order (real = true, q unused). Returns the count.
struct PolePair
{
    double q = 0.0;
    bool real = false;
};
int butterworthPoles(int order, PolePair* out);

// How a section is fitted to its prototype.
enum class DesignQuality
{
    Matched, // closed form only (DC, centre and Nyquist matched)
    Minimum, // closed form, then fitted in magnitude (default)
    Natural, // closed form, then fitted in magnitude and phase (analog-like phase)
    Smooth   // Nyquist gain pinned and only poles and one zero fitted: a continuous function of the parameters (for sweeps)
};

// Digital section for the prototype with centre frequency centreHz at sampleRate. centreHz is limited to 0.49 of the
// sampling rate.
// `cut` marks a prototype that is the inverse of a boost of the same shape (bell, shelf and tilt with negative gain): it is
// designed as that boost and inverted, which is the only way a deep cut stays realisable. The all-pass uses the pre-warped
// bilinear transform (the only mapping that keeps a unit magnitude).
Biquad designSection(const AnalogSection& prototype, double centreHz, double sampleRate,
                     DesignQuality quality = DesignQuality::Minimum, bool cut = false);

// The pre-warped bilinear transform of a prototype (exact at the centre frequency, cramped above it). It keeps every
// algebraic identity of the analog filter, which is what crossovers need for a flat sum.
Biquad bilinearSection(const AnalogSection& prototype, double centreHz, double sampleRate);

// Magnitude error (dB, largest absolute value) of a section against its prototype from lowHz to highHz (default 20 Hz to
// 20 kHz), on a dense grid; the figure the tests and docs quote.
double sectionError(const Biquad& digital, const AnalogSection& prototype, double centreHz, double sampleRate,
                    double lowHz = 20.0, double highHz = 20000.0);

// ---------------------------------------------------------------------------------------------------------------
// Bands: what the pro EQ offers, each a short cascade of sections.

enum class BandType
{
    Bell,
    LowShelf,
    HighShelf,
    Tilt,
    BandPass,
    Notch,
    AllPass,
    LowCut, // high-pass
    HighCut // low-pass
};

enum class CutShape
{
    Butterworth,
    Bessel,        // -3 dB frequency at the set frequency, maximally flat group delay
    LinkwitzRiley, // squared Butterworth: -6 dB at the set frequency (orders 2, 4, 8, 16; odd orders fall back to
                   // Butterworth)
    Resonant       // Butterworth with the highest-Q section set by the Q control
};

constexpr int kMaxBandSections = 8;
// Cut orders offered by the EQ: 6, 12, 18, 24, 36, 48 and 96 dB per octave.
constexpr int kNumCutSlopes = 7;
constexpr int cutOrder(int slopeIndex)
{
    constexpr int orders[kNumCutSlopes] = {1, 2, 3, 4, 6, 8, 16};
    return orders[slopeIndex < 0 ? 0 : (slopeIndex >= kNumCutSlopes ? kNumCutSlopes - 1 : slopeIndex)];
}

struct BandSpec
{
    BandType type = BandType::Bell;
    double freqHz = 1000.0;
    double gainDb = 0.0;
    double q = 0.7071;
    int slope = 1; // cuts: index into the orders above; shelves: 0 = 6 dB/oct, otherwise 12 dB/oct
    CutShape shape = CutShape::Butterworth;
};

// The analog prototype of the whole band as a list of sections that all share the band's frequency (the frequency scale
// of a section may differ: sections carry their own multiplier of the band frequency).
struct BandPrototype
{
    struct Part
    {
        AnalogSection section;
        double freqScale = 1.0; // this section's f0 = freqScale * band frequency
        double gain = 1.0;      // linear gain after the section (the prototypes of cuts are normalised to unity pass band)
    };
    int count = 0;
    std::array<Part, kMaxBandSections> part;
};

BandPrototype bandPrototype(const BandSpec& spec);
// Magnitude in dB and complex response of the analog prototype of the band at frequency hz.
std::complex<double> bandAnalogResponse(const BandPrototype& prototype, const BandSpec& spec, double hz);
double bandAnalogMagnitudeDb(const BandPrototype& prototype, const BandSpec& spec, double hz);

struct BandFilter
{
    int count = 0;
    std::array<Biquad, kMaxBandSections> section;
    double gain = 1.0; // overall linear gain (unity for every band type but the all-pass-free cuts' trim)

    std::complex<double> response(double omega) const;
    double magnitudeDb(double omega) const;
    bool identity() const { return count == 0 && gain == 1.0; }
};

BandFilter designBand(const BandSpec& spec, double sampleRate, DesignQuality quality = DesignQuality::Minimum);
BandFilter designBand(const BandSpec& spec, const BandPrototype& prototype, double sampleRate,
                      DesignQuality quality = DesignQuality::Minimum);

// Number of Bessel / Butterworth sections and their data are internal; this is the order -> section count rule.
constexpr int sectionsForOrder(int order)
{
    return (order + 1) / 2;
}

// ---------------------------------------------------------------------------------------------------------------
// A cascade of up to kMaxBandSections sections on two lanes (double precision, direct form 1: the states are signal
// history, so changing the coefficients under a running signal does not disturb them). New coefficients are reached by a
// straight line per coefficient over `rampSamples` samples; poles stay inside the unit circle all the way because the
// stability region is convex in (a1, a2). Denormals are flushed.
class BiquadChain
{
public:
    // Target coefficients for the next samples; sections the new filter does not have fade to the identity.
    void set(const BandFilter& filter, int rampSamples);
    void set(const Biquad* sections, int count, int rampSamples);
    // Jump to the coefficients (no ramp).
    void setNow(const BandFilter& filter) { set(filter, 0); }
    void reset();
    // True when every section is the identity and nothing ramps: the chain can be skipped.
    bool idle() const { return active == 0; }
    // Lanes: bit 0 = lane a, bit 1 = lane b. A lane that is not processed keeps its state (callers keep a lane's mask
    // constant while a band is in use).
    void process(double* a, double* b, int n, int lanes = 3) noexcept;

private:
    struct Stage
    {
        double c[5] = {1.0, 0.0, 0.0, 0.0, 0.0};
        double target[5] = {1.0, 0.0, 0.0, 0.0, 0.0};
        double step[5] = {};
        int remaining = 0;
        double x1[2] = {}, x2[2] = {}, y1[2] = {}, y2[2] = {};
    };
    void processStage(Stage& s, double* a, double* b, int n, int lanes) noexcept;
    void refreshActive();
    std::array<Stage, kMaxBandSections> stage;
    int active = 0; // number of leading stages that are not idle
};

// ---------------------------------------------------------------------------------------------------------------
// Topology-preserving state-variable filter (Zavalishin / Simper), for audio-rate modulation of cutoff and gain:
// stable and click-free under any modulation, with the cutoff pre-warped (so, like the bilinear designs, it cramps near
// Nyquist; it is the structure for modulated filters, not for static curve matching).
class TptSvf
{
public:
    enum class Mode
    {
        LowPass,
        HighPass,
        BandPass,
        Notch,
        AllPass,
        Bell,
        LowShelf,
        HighShelf
    };
    // gainDb applies to Bell and the shelves.
    void set(Mode mode, double sampleRate, double hz, double q, double gainDb = 0.0);
    void reset() { ic1 = ic2 = 0.0; }
    double process(double x)
    {
        const double v3 = x - ic2;
        const double v1 = a1 * ic1 + a2 * v3;
        const double v2 = ic2 + a2 * ic1 + a3 * v3;
        ic1 = 2.0 * v1 - ic1;
        ic2 = 2.0 * v2 - ic2;
        return m0 * x + m1 * v1 + m2 * v2;
    }
    // Complex response at the normalised angular frequency omega.
    std::complex<double> response(double omega) const;

private:
    double a1 = 0.0, a2 = 0.0, a3 = 0.0, m0 = 1.0, m1 = 0.0, m2 = 0.0, g = 0.0, k = 1.0;
    double ic1 = 0.0, ic2 = 0.0;
};

} // namespace crucible::fx
