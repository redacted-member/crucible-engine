// SPDX-License-Identifier: MIT
#include <crucible/fx/Bbd.h>

#include <algorithm>
#include <cmath>
#include <complex>

namespace crucible::fx
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kClipLevel = 3.0;
constexpr double kOutputLimit = 4.0;
constexpr double kEnvFloor = 1.0e-4;
constexpr int kCoefInterval = 16;
constexpr double kCompAttackMs = 3.0, kCompReleaseMs = 30.0;
constexpr double kDriveMs = 5.0; // time constant of each of the two poles that move the drive gain

double timeConstant(double ms, double rate)
{
    return std::exp(-1.0 / (0.001 * ms * rate));
}

// Rational tanh, exact at 0 and +-3 (where it reaches 1): x (27 + x^2) / (27 + 9 x^2).
double softLimit(double u)
{
    if (u >= 3.0)
        return 1.0;
    if (u <= -3.0)
        return -1.0;
    const double u2 = u * u;
    return u * (27.0 + u2) / (27.0 + 9.0 * u2);
}
} // namespace

void Bbd::Lowpass::init()
{
    // 4th-order Butterworth as two sections, Q = 1 / (2 sin(theta)), theta = pi/8 and 3 pi/8.
    s[0].k = 2.0 * std::sin(kPi / 8.0);
    s[1].k = 2.0 * std::sin(3.0 * kPi / 8.0);
}

void Bbd::prepare(double sampleRate)
{
    rate = sampleRate;
    invRate = 1.0 / sampleRate;
    ring.assign(static_cast<std::size_t>(kMaxStages / 2), 0.0f);
    tapDelay.prepare(64.0, 16);
    inFilter.init();

    // The reconstruction filter (8th-order Butterworth, cutoff kFilterCutoffRatio clock frequencies) from the partial
    // fractions of its step response, S(x) = 1 + sum of res exp(p x), x in clock periods. The poles come in conjugate pairs:
    // the four with a positive imaginary part are the modes, and the output is the held value plus twice the real part of
    // their sum.
    {
        using Cx = std::complex<double>;
        const double wc = 2.0 * kPi * kFilterCutoffRatio;
        constexpr int kOrder = 8;
        std::array<Cx, kOrder> p{};
        for (int k = 0; k < kOrder; ++k)
            p[static_cast<std::size_t>(k)] = std::polar(wc, kPi / 2.0 + (2.0 * k + 1.0) * kPi / (2.0 * kOrder));
        std::array<Cx, kOrder> res{};
        for (int k = 0; k < kOrder; ++k)
        {
            Cx r = std::pow(Cx(wc, 0.0), kOrder) / p[static_cast<std::size_t>(k)];
            for (int m = 0; m < kOrder; ++m)
                if (m != k)
                    r /= (p[static_cast<std::size_t>(k)] - p[static_cast<std::size_t>(m)]);
            res[static_cast<std::size_t>(k)] = r;
        }
        advance.assign(static_cast<std::size_t>(kModeRes + 2), ModeEntry{});
        inject.assign(static_cast<std::size_t>(kModeRes + 2), ModeEntry{});
        for (int i = 0; i <= kModeRes + 1; ++i)
        {
            const double x = static_cast<double>(i) / kModeRes;
            for (int m = 0; m < kModes; ++m)
            {
                const auto k = static_cast<std::size_t>(m);
                const Cx e = std::exp(p[k] * x);
                const Cx r = res[k] * e;
                advance[static_cast<std::size_t>(i)].re[m] = static_cast<float>(e.real());
                advance[static_cast<std::size_t>(i)].im[m] = static_cast<float>(e.imag());
                inject[static_cast<std::size_t>(i)].re[m] = static_cast<float>(r.real());
                inject[static_cast<std::size_t>(i)].im[m] = static_cast<float>(r.imag());
            }
        }
    }
    cfg = BbdSettings{};
    glideAlpha = 1.0 - std::exp(-1.0 / (0.001 * cfg.glideMs * rate));
    setSettings(cfg);
    targetDelay = 0.02;
    setDelaySeconds(targetDelay);
    clock = clockTarget;
    reset();
}

void Bbd::reset()
{
    std::fill(ring.begin(), ring.end(), 0.0f);
    ringPos = 0;
    tapDelay.reset();
    inFilter.reset();
    mode = ModeEntry{};
    held = 0.0f;
    phase = 0.0;
    envC = envE = 0.0;
    feedSign = 1.0;
    coefCountdown = 0;
    clock = clockTarget;
    driveStage1 = driveStage2 = driveGain;
}

void Bbd::setSettings(const BbdSettings& s)
{
    const int stages = std::clamp((s.stages / 2) * 2, kMinStages, kMaxStages);
    const bool restage = stages != cfg.stages || ringLength != stages / 2;
    cfg = s;
    cfg.stages = stages;
    cfg.driveDb = std::clamp(cfg.driveDb, -24.0f, 24.0f);
    cfg.breathing = std::clamp(cfg.breathing, 0.0f, 1.0f);
    cfg.glideMs = std::max(cfg.glideMs, 0.1f);
    ringLength = stages / 2;
    if (restage)
    {
        std::fill(ring.begin(), ring.end(), 0.0f);
        ringPos = 0;
    }
    driveGain = std::pow(10.0, static_cast<double>(cfg.driveDb) / 20.0);
    driveAlpha = 1.0 - std::exp(-1.0 / (kDriveMs * 0.001 * rate));
    noiseAmp = cfg.noiseDb <= -120.0f ? 0.0 : std::pow(10.0, static_cast<double>(cfg.noiseDb) / 20.0);
    feedAmp = cfg.clockFeedthroughDb <= -120.0f ? 0.0 : std::pow(10.0, static_cast<double>(cfg.clockFeedthroughDb) / 20.0);
    glideAlpha = 1.0 - std::exp(-1.0 / (0.001 * static_cast<double>(cfg.glideMs) * rate));
    attC = timeConstant(kCompAttackMs, rate);
    relC = timeConstant(kCompReleaseMs, rate);
    const double slow = 1.0 + 2.0 * static_cast<double>(cfg.breathing);
    attE = timeConstant(kCompAttackMs * slow, rate);
    relE = timeConstant(kCompReleaseMs * slow, rate);
    setDelaySeconds(targetDelay);
}

void Bbd::setDelaySeconds(double seconds)
{
    targetDelay = std::max(seconds, 1.0e-4);
    clockTarget = std::clamp(clockForDelay(cfg.stages, targetDelay), kMinClockHz, kMaxClockRatio * rate);
}

void Bbd::updateCoefficients()
{
    const double fc = std::min(kFilterCutoffRatio * clock, 0.45 * rate);
    const double g = std::tan(kPi * fc / rate);
    inFilter.setCutoff(g);
}

Bbd::ModeEntry Bbd::lookup(const std::vector<ModeEntry>& table, double x) const
{
    const float pos = static_cast<float>(x * kModeRes);
    const int idx = static_cast<int>(pos);
    const float w = pos - static_cast<float>(idx);
    const ModeEntry& a = table[static_cast<std::size_t>(idx)];
    const ModeEntry& b = table[static_cast<std::size_t>(idx) + 1];
    ModeEntry r;
    for (int m = 0; m < kModes; ++m)
    {
        r.re[m] = a.re[m] + w * (b.re[m] - a.re[m]);
        r.im[m] = a.im[m] + w * (b.im[m] - a.im[m]);
    }
    return r;
}

double Bbd::nextNoise()
{
    rng ^= rng << 13;
    rng ^= rng >> 7;
    rng ^= rng << 17;
    // Uniform in (-sqrt(3), sqrt(3)): unit variance.
    return (static_cast<double>(rng >> 11) * (1.0 / 9007199254740992.0) - 0.5) * 3.4641016151377544;
}

void Bbd::processBlock(const float* in, float* out, int n)
{
    run([in](int i, float) { return in[i]; }, out, n);
}

void Bbd::processWithInput(InputFn input, void* context, float* out, int n)
{
    run([input, context](int i, float previous) { return input(context, i, previous); }, out, n);
}

template <class Input> void Bbd::run(Input&& input, float* out, int n)
{
    const bool comp = cfg.compander;
    float previous = 0.0f;
    for (int i = 0; i < n; ++i)
    {
        clock += (clockTarget - clock) * glideAlpha;
        if (coefCountdown <= 0)
        {
            updateCoefficients();
            coefCountdown = kCoefInterval;
            float mag = 0.0f;
            for (int m = 0; m < kModes; ++m)
                mag += std::abs(mode.re[m]) + std::abs(mode.im[m]);
            if (mag < 1.0e-20f) // a decayed mode would end in denormal numbers
                mode = ModeEntry{};
        }
        --coefCountdown;

        if (driveStage2 != driveGain || driveStage1 != driveGain)
        {
            driveStage1 += (driveGain - driveStage1) * driveAlpha;
            driveStage2 += (driveStage1 - driveStage2) * driveAlpha;
            if (std::abs(driveGain - driveStage2) < 1.0e-9 * driveGain &&
                std::abs(driveGain - driveStage1) < 1.0e-9 * driveGain)
                driveStage1 = driveStage2 = driveGain;
        }
        double x = static_cast<double>(input(i, previous)) * driveStage2;
        if (comp)
        {
            const double a = std::abs(x);
            envC = a + (envC - a) * (a > envC ? attC : relC);
            if (envC < 1.0e-15)
                envC = 0.0;
            x /= std::sqrt(std::max(envC, kEnvFloor));
        }
        tapDelay.write(static_cast<float>(inFilter.process(x)));

        const double inc = clock * invRate;
        {
            const ModeEntry e = lookup(advance, inc);
            for (int m = 0; m < kModes; ++m)
            {
                const float re = mode.re[m] * e.re[m] - mode.im[m] * e.im[m];
                mode.im[m] = mode.re[m] * e.im[m] + mode.im[m] * e.re[m];
                mode.re[m] = re;
            }
        }
        phase += inc;
        if (phase >= 1.0)
        {
            const double u = (1.0 - (phase - inc)) / inc; // fraction of this sample interval at which the tick falls
            phase -= 1.0;
            const double sampled = static_cast<double>(tapDelay.read(tapDelay.minDelay() + (1.0 - u)));
            const double clipped = kClipLevel * softLimit(sampled * (1.0 / kClipLevel));
            const double oldest = static_cast<double>(ring[static_cast<std::size_t>(ringPos)]);
            ring[static_cast<std::size_t>(ringPos)] = static_cast<float>(clipped);
            if (++ringPos >= ringLength)
                ringPos = 0;
            double value = oldest;
            if (noiseAmp > 0.0)
                value += noiseAmp * nextNoise();
            if (feedAmp > 0.0)
            {
                value += feedAmp * feedSign;
                feedSign = -feedSign;
            }
            const float step = static_cast<float>(value) - held;
            held = static_cast<float>(value);
            const ModeEntry e = lookup(inject, (1.0 - u) * inc);
            for (int m = 0; m < kModes; ++m)
            {
                mode.re[m] += step * e.re[m];
                mode.im[m] += step * e.im[m];
            }
        }

        double y = static_cast<double>(held);
        {
            const float sum = (mode.re[0] + mode.re[1]) + (mode.re[2] + mode.re[3]);
            y += 2.0 * static_cast<double>(sum);
        }
        if (comp)
        {
            const double a = std::abs(y);
            envE = a + (envE - a) * (a > envE ? attE : relE);
            if (envE < 1.0e-15)
                envE = 0.0;
            y *= envE;
        }
        previous = static_cast<float>(kOutputLimit * softLimit(y / kOutputLimit));
        out[i] = previous;
    }
}

} // namespace crucible::fx
