// SPDX-License-Identifier: MIT
#pragma once

// Saturation library: the one home of the engine's nonlinear "colour" models. The saturator module (fx.saturator), the
// per-voice shaper and the later saturator, tape, console and character-compressor modules all call this file and
// none of them carries a private waveshaper.
//
// Models (all have a small-signal slope of 1 around the origin, so drive 0 dB is a transparent setting):
//   RectHalf, RectFull, SineFold,   appended shapers (see Model): half and full rectifier, sine fold, linear (triangle)
//   LinFold, ZeroSquare, Asym       fold, near-square "zero-square" and an asymmetric soft clip, clean-room curves with
//                                   closed-form antiderivatives; they leave the slope of 1 only where noted in Model
//   Soft, Hard, Rail, Diode, Fold   memoryless curves with a closed-form antiderivative: first-order antiderivative
//                                   anti-aliasing (ADAA, Parker et al., DAFx 2016) removes most of the folding
//                                   without oversampling; Fold is a triangle foldback
//   TriodeSE, TriodePP, Pentode     a vacuum-tube gain stage solved on its load line from Koren's published
//                                   closed-form tube equations (Norman Koren, "Improved vacuum tube models for SPICE
//                                   simulations", 2003), including grid current and a level-dependent bias shift
//                                   ("sag"). The transfer curve is tabulated once; ADAA runs on the table
//   Tape                            Jiles-Atherton magnetic hysteresis (Jiles and Atherton, 1986), integrated with
//                                   RK2 or RK4 (Holters and Zoelzer, DAFx 2016; Chowdhury, DAFx 2019), so the output
//                                   depends on the history of the input and a sine traces a loop with area > 0
//   Transformer                     flux saturation: the input is integrated (a leaky integrator with the corner
//                                   cornerHz), passed through tanh and differentiated, so distortion appears where
//                                   the flux is large, that is at low frequency and high level
//
// Second-order ADAA (Settings::adaaOrder = 2, Bilbao et al., DAFx 2017) runs on tabulated curves, whose
// antiderivatives are exact; it removes about twice as many decibels of aliasing as the first order for a sample more
// delay. The default stays the first order, which is what the earlier modules and patches use. The second order has
// block kernels of its own (process(in, out, n) and processPair): the same arithmetic as the sample-by-sample form with
// the recursive state in registers, the drive glide shortcut once it has settled, the tube sag following the level of
// the curve before the shift, and, on 64-bit ARM, two channels in the lanes of float64x2. Nothing else calls them, so
// the first-order and earlier paths are unchanged bit for bit.
//
// Rates: a Saturator runs at the rate it was prepared for. Callers that need the nonlinearity oversampled use
// OversampledSaturator (below) or run Saturator at their own oversampled rate. Corners are in Hz and follow the
// prepared rate. process() never allocates, locks or throws; prepare() allocates and may build shared tables.

#include <crucible/Oversampler.h>
#include <crucible/Simd.h>
#include <crucible/module/Module.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace crucible::saturation
{

enum class Model : int
{
    Soft = 0,
    Hard,
    Rail,
    Diode,
    Fold,
    TriodeSE,
    TriodePP,
    Pentode,
    Tape,
    Transformer,
    // Appended shapers (indices 10 to 15; earlier indices never move). All are memoryless with closed-form
    // antiderivatives and the same first- and second-order ADAA as Soft; the rectifiers and the asymmetric curve make
    // DC, which the DC blocker behind the model removes (dcCornerHz).
    RectHalf,   // max(tanh x, 0)
    RectFull,   // |tanh x|
    SineFold,   // sin x
    LinFold,    // triangle foldback that starts folding at |x| = 0.5 (peak 1, slope 2 around the origin)
    ZeroSquare, // tanh(8 x): a near-square wave from a small input, the zero crossing taken very steeply
    Asym,       // tanh x above zero, 0.4 tanh(x / 0.4) below: the negative half clips early (slope 1 at the origin)
    Count
};

constexpr int kNumModels = static_cast<int>(Model::Count);

const std::string& modelName(Model m);
const std::vector<std::string>& modelNames();

// Oversampling factor the plan assigns to a model per quality (1, 2, 4 or 8). shaperNode: the per-voice shaper,
// which runs in a wider voice and gets one step more.
int recommendedOversampling(Model m, Quality q, bool shaperNode = false);

struct Settings
{
    Model model = Model::Soft;
    float driveDb = 0.0f;    // gain in front of the nonlinearity, 0 to 48 dB; 0 dB is the transparent setting
    float bias = 0.0f;       // -1 to 1: input offset (asymmetry, adds even harmonics to symmetric curves)
    float dcCornerHz = 8.0f; // corner of the DC blocker behind the model; 0 disables it
    float sag = 0.0f;        // 0 to 1: level-dependent bias shift of the tube models
    float cornerHz = 12.0f;  // Transformer: flux corner (the saturation knee moves with it)
    float hysteresis = 0.5f; // Tape: width of the loop, 0 (thin) to 1 (wide)
    bool adaa = true;        // memoryless curves: antiderivative anti-aliasing
    int adaaOrder = 1;       // 1, or 2 for the memoryless curves and the tubes (second antiderivative, a sample of delay)
    int integrator = 2;      // Tape: 1 Euler, 2 RK2, 4 RK4
    bool tapeFast =
        false; // Tape: relaxation form of the loop with a block kernel (see Saturator::tapeFastBlock); integrator is ignored
};

// Jiles-Atherton parameters in normalised units (saturation magnetisation 1): anhysteretic knee a (tanh form, the
// J = 1/2 Brillouin function), pinning k, domain coupling alpha, reversibility c.
struct JaParams
{
    double a = 1.0, k = 0.05, alpha = 0.01, c = 0.8;
    double invA = 1.0; // 1 / a
};

// Tabulated transfer curve and its antiderivatives (piecewise linear f, so F is exactly piecewise quadratic and F2,
// the antiderivative of F, exactly piecewise cubic).
struct ShapeTable
{
    double range = 0.0; // table covers [-range, range]; beyond it f continues with its end value
    double step = 0.0;
    std::vector<double> f;  // f at the nodes
    std::vector<double> F;  // antiderivative, F(0) = 0
    std::vector<double> F2; // second antiderivative, F2(0) = 0
    double fAt(double x) const;
    double FAt(double x) const;
    double F2At(double x) const;

    // The same data packed per cell for the block kernels: one cache line gives f, F, F2 and the slope of the cell, and
    // the two extra nodes continue the curve with its end values beyond the range (slope 0).
    struct Node
    {
        double f, F, F2, slope;
    };
    std::vector<Node> node; // cells + 1 nodes, then the left and the right extension
    double invStep = 0.0;
    int cells = 0;
    // The cell holding x and the offset inside it. Forced inline: MSVC kept it an out-of-line call per sample, with the
    // offset passed through memory.
    CRUCIBLE_ALWAYS_INLINE const Node& locate(double x, double& t) const
    {
        const double u = (x + range) * invStep;
        const double top = static_cast<double>(cells);
        const auto i = static_cast<size_t>(u > 0.0 ? (u < top ? u : 0.0) : 0.0);
        const bool low = u <= 0.0, high = u >= top;
        const size_t k = low ? static_cast<size_t>(cells) + 1 : (high ? static_cast<size_t>(cells) + 2 : i);
        t = low ? x + range : (high ? x - range : (u - static_cast<double>(i)) * step);
        return node[k];
    }
};

// One channel of one model. Scalar; use Saturator4 for four voices at once.
class Saturator
{
public:
    // Control thread. rate: the rate process() runs at. Builds the shared tube tables on first use and calibrates
    // the tape model for this rate.
    void prepare(double rate);
    // Number of prepare() calls made by the calling thread so far (lets tests prove the audio thread never prepares).
    static long prepareCalls();
    void reset();
    // Cheap and allocation free: call per block. Drive and bias glide per sample; a changed model clears the state.
    void set(const Settings& s);
    const Settings& settings() const { return cfg; }

    float process(float x);
    void process(const float* in, float* out, int n);
    // Two independent channels of the same settings in one loop (in place). The recursion of the hysteresis is a long
    // chain of dependent operations, so running two channels side by side takes little more time than one.
    static void processPair(Saturator& a, Saturator& b, float* x, float* y, int n);

    // Takes over the slow state of another instance of the same model that runs at another rate: the drive glide, the
    // sag envelope, the DC blocker, the loop position of the hysteresis and the flux. The antiderivative history
    // restarts (its first two outputs are the plain curve).
    void adoptState(const Saturator& o);
    // Group delay in samples at low frequency (ADAA delays by half a sample per order).
    float latencySamples() const;
    // Loop state of the tape model, for tests: the magnetisation in output units.
    double magnetisation() const { return tapeM * tapeNorm; }

private:
    struct Dc
    {
        double s = 0.0;
        double step(double x, double g)
        {
            const double v = (x - s) * g / (1.0 + g);
            const double lp = v + s;
            s = lp + v;
            return x - lp;
        }
    };

    template <class Curve> double adaaStep(double x, const Curve& c);
    template <class Curve> double adaa2Step(double x, const Curve& c);
    template <class Curve> void adaa2Block(const Curve& c, const float* in, float* out, int n);
    void transformerBlock(const float* in, float* out, int n);
    bool hasSag() const
    {
        return cfg.sag > 0.0f &&
               (cfg.model == Model::TriodeSE || cfg.model == Model::TriodePP || cfg.model == Model::Pentode);
    }
    template <class Curve> void adaa2Drive(const Curve& c, const float* in, double* xs, int len, bool withSag = true);
    template <class Curve> void adaa2Sag(const Curve& c, double* xs, int len);
    template <class Curve>
    static void adaa2SagPair(Saturator& a, Saturator& b, const Curve& c, double* xs0, double* xs1, int len);
    template <class Curve> void adaa2Run(const Curve& c, const double* xs, float* out, int len);
    template <class Curve> static void adaa2PairBlock(Saturator& a, Saturator& b, const Curve& c, float* x, float* y, int n);
    template <class Curve>
    static void adaa2PairRun(Saturator& a, Saturator& b, const Curve& c, const double* xs0, const double* xs1, float* out0,
                             float* out1, int len);
    double tableStep(const ShapeTable& t, double x);
    double tapeStep(double h);
    double tapeFastStep(double h);
    template <int N>
    static void tapeFastBlock(Saturator* const (&s)[N], const float* const (&in)[N], float* const (&out)[N], int n);
    double transformerStep(double x);
    double model(double xDriven);

    Settings cfg;
    double sr = 48000.0;
    double driveGain = 1.0, driveTarget = 1.0, driveCoef = 0.002;
    double biasOffset = 0.0;
    double dcG = 0.0;
    double sagEnv = 0.0, sagAtk = 0.0, sagRel = 0.0;
    double xPrev = 0.0, FPrev = 0.0;
    double xPrev2 = 0.0, F2Prev = 0.0, divPrev = 0.0; // second order: x[n-2], F2(x[n-1]), F2[x[n-1], x[n-2]]
    int history = 0;
    bool havePrev = false;
    Dc dc;
    // tape
    double tapeM = 0.0, tapeHprev = 0.0, tapeNorm = 1.0;
    bool tapeCalibrated = false;
    JaParams ja;
    // transformer
    double fluxState = 0.0, fluxG = 0.0;
    const ShapeTable* table = nullptr;
};

// Four independent voices of the same model, one lane each (the lanes share the settings).
class Saturator4
{
public:
    void prepare(double rate)
    {
        for (auto& l : lane)
            l.prepare(rate);
    }
    void reset()
    {
        for (auto& l : lane)
            l.reset();
    }
    void set(const Settings& s)
    {
        for (auto& l : lane)
            l.set(s);
    }
    simd::Float4 process(simd::Float4 x);

private:
    std::array<Saturator, 4> lane;
};

// A Saturator behind the engine's oversampler: up, nonlinearity at factor x the rate, down. The reusable building
// block of the modules; latency() is the whole-sample delay of the path so that a dry signal can be aligned.
class OversampledSaturator
{
public:
    // factor 1 bypasses the oversampler. maxBlock: largest block passed to process().
    void prepare(double rate, int factor, OversamplingFilter filter, Quality quality, int maxBlock);
    void reset();
    void set(const Settings& s) { sat.set(s); }
    // In place. n <= maxBlock.
    void process(float* x, int n);
    int factor() const { return os; }
    float latency() const { return lat; }

private:
    Saturator sat;
    Oversampler over;
    int os = 1;
    float lat = 0.0f;
    std::vector<float> hi;
};

} // namespace crucible::saturation
