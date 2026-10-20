// SPDX-License-Identifier: MIT
// util.meter: loudness, true-peak and level meter that passes the audio through unchanged (include/crucible/fx/Loudness.h).
#include "FxModuleSupport.h"

#include <crucible/fx/Loudness.h>
#include <crucible/module/MeterReadout.h>

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule, public MeterReadout
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : resetStats(localIndex(d, "fx_mtr_reset")), ppmType(localIndex(d, "fx_mtr_ppm"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        rate = c.sampleRate;
        meter.prepare(rate, 2, true);
        type = crucible::fx::PpmType::TypeI;
        // The programme meters are slow: they run on groups of kGroup samples (peak, mean magnitude, mean square).
        const double groupRate = rate / kGroup;
        for (int ch = 0; ch < 2; ++ch)
        {
            ppm[ch].prepare(groupRate, type);
            vu[ch].prepare(groupRate);
        }
        rmsCoefficient = static_cast<float>(1.0 - std::exp(-1.0 / (0.3 * groupRate)));
        correlation.prepare(groupRate);
        clearGroup();
    }
    void reset() override
    {
        meter.reset();
        for (int ch = 0; ch < 2; ++ch)
        {
            ppm[ch].reset();
            vu[ch].reset();
            meanSquare[ch] = 0.0f;
        }
        correlation.reset();
        clearGroup();
    }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        using namespace crucible::fx;
        if (in.on(resetStats) && !resetHeld)
            meter.resetStatistics();
        resetHeld = in.on(resetStats);
        const PpmType wanted = in.choice(ppmType) == 1 ? PpmType::TypeII : PpmType::TypeI;
        if (wanted != type)
        {
            type = wanted;
            ppm[0].prepare(rate / kGroup, type);
            ppm[1].prepare(rate / kGroup, type);
        }
        meter.process(l, r, n);
        for (int i = 0; i < n; ++i)
        {
            // The audio passes through untouched; what the meters see is cleaned of NaN and infinities.
            const float x = (l[i] == l[i] && std::abs(l[i]) < 1.0e18f) ? l[i] : 0.0f;
            const float y = (r[i] == r[i] && std::abs(r[i]) < 1.0e18f) ? r[i] : 0.0f;
            const float a = std::abs(x), b = std::abs(y);
            peak[0] = std::max(peak[0], a);
            peak[1] = std::max(peak[1], b);
            sumAbs[0] += a;
            sumAbs[1] += b;
            sumSquare[0] += x * x;
            sumSquare[1] += y * y;
            sumCross += x * y;
            if (++filled == kGroup)
                closeGroup();
        }
    }

    void loudness(float* out) const override
    {
        out[0] = meter.momentaryLufs();
        out[1] = meter.shortTermLufs();
        out[2] = meter.integratedLufs();
        out[3] = meter.loudnessRange();
        out[4] = meter.truePeakDb();
        out[5] = meter.maxMomentaryLufs();
        out[6] = meter.rangeLowLufs();
        out[7] = meter.rangeHighLufs();
    }
    bool levels(float* out) const override
    {
        const auto db = [](float v) { return v > 1.0e-6f ? 20.0f * std::log10(v) : -120.0f; };
        out[0] = db(meter.samplePeak(0));
        out[1] = db(meter.samplePeak(1));
        out[2] = db(meter.decayingTruePeak(0));
        out[3] = db(meter.decayingTruePeak(1));
        // RMS in dB relative to a full-scale sine: a sine of amplitude A reads A.
        out[4] = db(std::sqrt(meanSquare[0]) * 1.41421356f);
        out[5] = db(std::sqrt(meanSquare[1]) * 1.41421356f);
        out[6] = ppm[0].levelDb();
        out[7] = ppm[1].levelDb();
        out[8] = vu[0].levelDb();
        out[9] = vu[1].levelDb();
        out[10] = correlation.value();
        return true;
    }

private:
    static constexpr int kGroup = 8;
    void clearGroup()
    {
        peak[0] = peak[1] = 0.0f;
        sumAbs[0] = sumAbs[1] = sumSquare[0] = sumSquare[1] = 0.0f;
        sumCross = 0.0;
        filled = 0;
    }
    void closeGroup()
    {
        const float inv = 1.0f / kGroup;
        for (int c = 0; c < 2; ++c)
        {
            ppm[c].process(peak[c]);
            vu[c].process(sumAbs[c] * inv);
            meanSquare[c] += rmsCoefficient * (sumSquare[c] * inv - meanSquare[c]);
            if (meanSquare[c] < 1.0e-30f)
                meanSquare[c] = 0.0f;
        }
        correlation.processMeans(sumSquare[0] * inv, sumSquare[1] * inv, sumCross * inv);
        clearGroup();
    }

    const int resetStats, ppmType;
    float peak[2] = {0.0f, 0.0f}, sumAbs[2] = {0.0f, 0.0f}, sumSquare[2] = {0.0f, 0.0f};
    double sumCross = 0.0;
    int filled = 0;
    double rate = 48000.0;
    crucible::fx::LoudnessMeter meter;
    crucible::fx::PpmMeter ppm[2];
    crucible::fx::VuMeter vu[2];
    float meanSquare[2] = {0.0f, 0.0f}, rmsCoefficient = 1.0e-3f;
    crucible::fx::CorrelationMeter correlation;
    crucible::fx::PpmType type = crucible::fx::PpmType::TypeI;
    bool resetHeld = false;
};
} // namespace

ModuleDescriptor meterEffect()
{
    auto d = effectDescriptor(
        "util.meter", "Meter", "Utility", CpuClass::Light,
        "Loudness and level meter that passes the audio through unchanged: ITU-R BS.1770-4 / EBU R128 momentary, "
        "short-term and integrated loudness with gating, loudness range (EBU Tech 3342), sample and 4x true peak, "
        "RMS, PPM (type I or II ballistics), VU and stereo correlation, published as views.");
    auto& p = d.params;
    addOn(p, "mtr", "Meter", "fx_mtr", 1.0f, false);
    p.push_back(params::toggle("fx_mtr_reset", "Meter Reset", "fx_mtr", false));
    p.push_back(params::choice("fx_mtr_ppm", "Meter PPM Type", "fx_mtr", {"Type I", "Type II"}, 0));
    p.push_back(params::integer("fx_pos_mtr", "Meter Slot", "fx_mtr", 1, kNumModuleFx, kNumFx + 7));
    d.onParam = "fx_mtr_on";
    d.positionParam = "fx_pos_mtr";
    d.capabilities = {"loudness", "levels"};
    d.pages.push_back(
        {"fx_mtr", "Meter", "fx", "Meter", "Reset of the integrated loudness, range and maxima, and the PPM ballistics."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("util.meter")); };
    return d;
}

} // namespace crucible::modules::fx
