// SPDX-License-Identifier: MIT
// Facts about the current oscillators and filters, established with the measurement primitives (AnalogMeasure.h) and
// recorded in docs/MEASUREMENTS.md. They guard the baseline the analog-filter and VA-oscillator packages start from:
// a number that moves for the worse fails here, a number that moves for the better is a reason to update the
// document and tighten the bound.

#include "AnalogMeasure.h"
#include "GraphSupport.h"

#include <crucible/Filters.h>

using namespace crucible;
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
auto makeFilter(int model, float cutoff, float res, double rate)
{
    return [=]
    {
        FilterChannel c;
        c.configure({model, cutoff, res}, rate);
        return [c](double x) mutable { return static_cast<double>(c.process(static_cast<float>(x))); };
    };
}

m::Signal renderTone(const std::vector<float>& values, double rate, int note, double seconds)
{
    SynthEngine engine;
    engine.prepare(rate);
    engine.setParameterValues(values.data(), static_cast<int>(values.size()));
    engine.noteOn(note, 1.0f);
    return m::toDouble(renderBlocks(engine, static_cast<int>(seconds * rate), 512, true).left);
}

std::vector<float> sawPatch()
{
    auto v = graphPatch();
    setValue(v, "filter_on", 0.0f);
    setValue(v, "osc1_level", 1.0f);
    setValue(v, "osc1_uni_rand", 0.0f);
    setValue(v, "osc1_table", 0.0f);
    setValue(v, "osc1_pos", 1.0f);
    return v;
}
} // namespace

class MeasureEngineTests : public UnitTest
{
public:
    MeasureEngineTests() : UnitTest("MeasuredBaseline", "Verification") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("wavetable saw keeps its aliasing below -100 dB at C2, C4 and C6 at 44.1, 48, 96 and 192 kHz");
        {
            // Regression: at 96 kHz and above a low note read table partials up to 76 kHz and the interpolation
            // images of the highest ones folded into the audible band (C2 at 96 kHz measured -50 dB).
            double worst = -300.0;
            for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
                for (const int note : {36, 60, 84})
                {
                    const auto sig = renderTone(sawPatch(), rate, note, 1.9);
                    const size_t start = static_cast<size_t>(0.5 * rate), n = m::floorPow2(sig.size() - start);
                    const double f0 = m::estimateFrequency(sig.data() + start, n, rate, midiToHz(note));
                    const double alias = m::measureAliasing(sig.data() + start, n, rate, f0).aliasDb;
                    worst = std::max(worst, alias);
                    expect(alias < -100.0,
                           "saw note " + String(note) + " at " + String(rate) + " Hz: alias " + String(alias, 1) + " dB");
                    expectWithinAbsoluteError(m::centsBetween(f0, midiToHz(note)), 0.0, 0.1, "tuning, note " + String(note));
                }
            logMessage("worst saw aliasing of the 12 cases: " + String(worst, 1) + " dB");
        }

        beginTest("the saw follows 1/n within 0.05 dB to 10 kHz");
        {
            const auto sig = renderTone(sawPatch(), 48000.0, 60, 1.9);
            const size_t start = 24000, n = m::floorPow2(sig.size() - start);
            const auto prof = m::harmonicProfile(sig.data() + start, n, 48000.0, 0.0, 200);
            const auto e = m::shapeError(prof, [](int k) { return -20.0 * std::log10(static_cast<double>(k)); }, 10000.0);
            expect(e.maxAbsDb < 0.05,
                   "saw shape error " + String(e.maxAbsDb, 4) + " dB over " + String(e.harmonics) + " harmonics");
        }

        beginTest("an idle engine and a silent voice output exact zeros");
        {
            SynthEngine idle;
            prepare(idle, ParameterMap::defaultValues());
            expectEquals(m::rmsOf(m::toDouble(renderBlocks(idle, 24000, 512, true).left).data(), 24000), 0.0);
            auto v = sawPatch();
            setValue(v, "osc1_level", 0.0f);
            const auto held = renderTone(v, 48000.0, 60, 1.0);
            expectEquals(m::rmsOf(held.data(), held.size()), 0.0);
        }

        beginTest("the state-variable filters never self-oscillate; the ladder does at 1 kHz and up");
        {
            for (const FilterModel model :
                 {FilterModel::Lowpass12, FilterModel::Lowpass24, FilterModel::DirtyLowpass, FilterModel::DiodeLowpass})
                expect(!m::measureSelfOscillation(makeFilter(static_cast<int>(model), 1000.0f, 1.0f, 48000.0), 48000.0, 2.0,
                                                  0.05, 1000.0)
                            .oscillating,
                       "model " + String(static_cast<int>(model)) + " decays at resonance 1");
            for (const float fc : {1000.0f, 3000.0f, 8000.0f})
            {
                const auto r = m::measureSelfOscillation(
                    makeFilter(static_cast<int>(FilterModel::LadderLowpass), fc, 1.0f, 48000.0), 48000.0, 3.0, 0.05, fc);
                expect(r.oscillating, "ladder rings at " + String(fc) + " Hz");
                expectWithinAbsoluteError(m::centsBetween(r.freqHz, fc), 0.0, 1.0, "ladder pitch at " + String(fc) + " Hz");
            }
        }

        beginTest("filter slopes: 12 dB/oct for the 2-pole low-pass and 24 dB/oct for the 4-pole low-pass, well below the "
                  "warping range");
        {
            const auto grid = m::logSpacedFrequencies(1000.0, 4000.0, 12);
            const auto slope = [&](FilterModel model)
            {
                return m::slopeDbPerOctave(m::steppedResponse(makeFilter(static_cast<int>(model), 250.0f, 0.0f, 48000.0),
                                                              grid, 48000.0, 0.01, 0.4, 0.2),
                                           1000.0, 4000.0);
            };
            expectWithinAbsoluteError(slope(FilterModel::Lowpass12), -12.0, 1.5);
            expectWithinAbsoluteError(slope(FilterModel::Lowpass24), -24.0, 1.5);
            expectWithinAbsoluteError(slope(FilterModel::LadderLowpass), -24.0, 1.5);
        }

        beginTest("the ladder loses bass as resonance rises, the 2-pole low-pass does not");
        {
            const auto bass = [&](FilterModel model, float res)
            {
                return m::steppedGainDb(makeFilter(static_cast<int>(model), 1000.0f, res, 48000.0), 50.0, 48000.0, 0.01, 0.6,
                                        0.3);
            };
            expectWithinAbsoluteError(bass(FilterModel::Lowpass12, 0.9f) - bass(FilterModel::Lowpass12, 0.0f), 0.0, 0.2);
            const double loss = bass(FilterModel::LadderLowpass, 0.9f) - bass(FilterModel::LadderLowpass, 0.0f);
            expect(loss < -2.0 && loss > -8.0, "ladder bass loss at resonance 0.9: " + String(loss, 2) + " dB");
        }
    }
};

static MeasureEngineTests measureEngineTests;
