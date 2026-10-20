// SPDX-License-Identifier: MIT
#include <crucible/Filters.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
double gainAt(FilterModel model, float cutoff, float res, double freq)
{
    FilterChannel c;
    FilterSettings s;
    s.model = static_cast<int>(model);
    s.cutoffHz = cutoff;
    s.resonance = res;
    c.configure(s, kRate);
    const int total = 24000;
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < total; ++i)
    {
        const float x = std::sin(static_cast<float>(2.0 * 3.14159265358979 * freq * i / kRate));
        const float y = c.process(x);
        if (i > total / 2)
        {
            sum += static_cast<double>(y) * y;
            ++count;
        }
    }
    return 20.0 * std::log10(std::sqrt(sum / count) / 0.70710678 + 1.0e-12);
}
} // namespace

class FilterTests : public UnitTest
{
public:
    FilterTests() : UnitTest("Filters", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        const int models = static_cast<int>(FilterModel::Count);
        expectEquals(static_cast<int>(filterModelNames().size()), models);

        beginTest("all models are stable across a cutoff x resonance grid with noise and impulses");
        {
            Rng rng;
            for (int m = 0; m < models; ++m)
                for (const float fc : {20.0f, 90.0f, 800.0f, 5000.0f, 15000.0f, 20000.0f})
                    for (const float res : {0.0f, 0.5f, 0.95f, 1.0f})
                    {
                        FilterChannel c;
                        c.configure({m, fc, res}, kRate);
                        float peak = 0.0f;
                        bool finite = true;
                        for (int i = 0; i < 6000; ++i)
                        {
                            const float x = i == 100 ? 1.0f : (i < 3000 ? rng.bipolar() * 0.5f : 0.0f);
                            const float y = c.process(x);
                            finite = finite && std::isfinite(y);
                            peak = std::max(peak, std::abs(y));
                        }
                        expect(finite, "non-finite model " + String(m));
                        expect(peak < 60.0f,
                               "model " + String(m) + " fc " + String(fc) + " res " + String(res) + " peak " + String(peak));
                    }
        }

        beginTest("stable under fast cutoff and resonance modulation");
        {
            Rng rng;
            for (int m = 0; m < models; ++m)
            {
                FilterChannel c;
                bool ok = true;
                float peak = 0.0f;
                for (int block = 0; block < 400; ++block)
                {
                    FilterSettings s;
                    s.model = m;
                    s.cutoffHz = 20.0f * std::pow(1000.0f, rng.unipolar());
                    s.resonance = rng.unipolar();
                    c.configure(s, kRate);
                    for (int i = 0; i < 32; ++i)
                    {
                        const float y = c.process(rng.bipolar());
                        ok = ok && std::isfinite(y);
                        peak = std::max(peak, std::abs(y));
                    }
                }
                expect(ok && peak < 80.0f, "model " + String(m) + " peak " + String(peak));
            }
        }

        beginTest("lowpass, highpass, bandpass, notch responses have the right shape");
        {
            const float fc = 1000.0f;
            expect(std::abs(gainAt(FilterModel::Lowpass12, fc, 0.0f, 100.0)) < 1.0);
            expect(gainAt(FilterModel::Lowpass12, fc, 0.0f, 10000.0) < -30.0);
            expect(gainAt(FilterModel::Highpass12, fc, 0.0f, 100.0) < -30.0);
            expect(std::abs(gainAt(FilterModel::Highpass12, fc, 0.0f, 10000.0)) < 1.0);
            expect(gainAt(FilterModel::Bandpass12, fc, 0.0f, 1000.0) > -1.5);
            expect(gainAt(FilterModel::Bandpass12, fc, 0.0f, 100.0) < -12.0);
            expect(gainAt(FilterModel::Notch, fc, 0.5f, 1000.0) < -25.0);
            expect(std::abs(gainAt(FilterModel::Notch, fc, 0.5f, 100.0)) < 1.5);
            expect(gainAt(FilterModel::Peak, fc, 0.8f, 1000.0) > 6.0);
            // 24 dB slopes are steeper than 12 dB.
            expect(gainAt(FilterModel::Lowpass24, fc, 0.0f, 4000.0) <
                   gainAt(FilterModel::Lowpass12, fc, 0.0f, 4000.0) - 10.0);
            expect(gainAt(FilterModel::Highpass24, fc, 0.0f, 250.0) <
                   gainAt(FilterModel::Highpass12, fc, 0.0f, 250.0) - 10.0);
            expect(gainAt(FilterModel::LadderLowpass, fc, 0.0f, 4000.0) < -30.0);
            expect(std::abs(gainAt(FilterModel::LadderLowpass, fc, 0.0f, 100.0)) < 1.5);
            expect(gainAt(FilterModel::LadderHighpass, fc, 0.0f, 100.0) < -25.0);
            expect(gainAt(FilterModel::DiodeLowpass, fc, 0.0f, 8000.0) < -25.0);
            expect(gainAt(FilterModel::DirtyLowpass, fc, 0.0f, 8000.0) < -25.0);
            // Resonance raises the level at the cutoff.
            expect(gainAt(FilterModel::Lowpass12, fc, 0.9f, 1000.0) >
                   gainAt(FilterModel::Lowpass12, fc, 0.0f, 1000.0) + 10.0);
            expect(gainAt(FilterModel::LadderLowpass, fc, 0.9f, 1000.0) >
                   gainAt(FilterModel::LadderLowpass, fc, 0.0f, 1000.0) + 3.0);
        }

        beginTest("comb filters peak at multiples of the cutoff; formant and phaser shape the spectrum");
        {
            const float fc = 600.0f;
            const double peakGain = gainAt(FilterModel::CombPlus, fc, 0.8f, 600.0);
            const double troughGain = gainAt(FilterModel::CombPlus, fc, 0.8f, 900.0);
            expect(peakGain > troughGain + 10.0, "comb+ peak " + String(peakGain) + " trough " + String(troughGain));
            const double negPeak = gainAt(FilterModel::CombMinus, fc, 0.8f, 900.0);
            const double negTrough = gainAt(FilterModel::CombMinus, fc, 0.8f, 600.0);
            expect(negPeak > negTrough + 10.0);

            // Formant: vowel A has energy near 800 Hz and far less at 6 kHz.
            expect(gainAt(FilterModel::Formant, 20.0f, 0.5f, 800.0) >
                   gainAt(FilterModel::Formant, 20.0f, 0.5f, 6000.0) + 15.0);
            // Phaser: notches exist, so the response is not flat.
            double lo = 100.0, hi = -100.0;
            for (double f = 100.0; f < 8000.0; f *= 1.2)
            {
                const double g = gainAt(FilterModel::Phaser, 1000.0f, 0.7f, f);
                lo = std::min(lo, g);
                hi = std::max(hi, g);
            }
            expect(hi - lo > 12.0, "phaser ripple " + String(hi - lo));
        }

        beginTest("drive stage saturates and stays bounded");
        {
            FilterUnit u;
            FilterSettings s;
            s.model = 0;
            s.cutoffHz = 20000.0f;
            u.configure(s, 1.0f, kRate);
            float peak = 0.0f;
            for (int i = 0; i < 4000; ++i)
            {
                float l = 20.0f * std::sin(0.05f * static_cast<float>(i));
                float r = l;
                u.process(l, r);
                peak = std::max(peak, std::abs(l));
            }
            expect(peak < 0.6f && peak > 0.1f, "drive peak " + String(peak));
        }

        beginTest("engine filter routing: serial, parallel and split behave differently");
        {
            const auto render = [&](int route, int osc1Route, int f1Type, float f1Cut)
            {
                auto v = ParameterMap::defaultValues();
                setValue(v, "osc1_table", 0.0f);
                setValue(v, "osc1_pos", 1.0f);
                setValue(v, "osc1_level", 1.0f);
                setValue(v, "osc1_route", static_cast<float>(osc1Route));
                setValue(v, "filter_type", static_cast<float>(f1Type));
                setValue(v, "filter_cutoff", f1Cut);
                setValue(v, "filter_res", 0.0f);
                setValue(v, "filter_route", static_cast<float>(route));
                setValue(v, "filter2_on", 1.0f);
                setValue(v, "filter2_type", 1.0f);
                setValue(v, "filter2_cutoff", 20000.0f);
                CRUCIBLE_HEAP(SynthEngine, e);
                prepare(e, v);
                e.noteOn(57, 1.0f);
                renderBlocks(e, 4800);
                return renderBlocks(e, 9600).peak;
            };
            const float serial = render(0, 0, 0, 200.0f);
            const float parallel = render(1, 0, 0, 200.0f);
            const float splitF1 = render(2, 0, 0, 200.0f);
            const float splitDry = render(2, 3, 0, 200.0f);
            logMessage("serial " + String(serial) + " parallel " + String(parallel) + " split->F1 " + String(splitF1) +
                       " split->dry " + String(splitDry));
            // Serial LP(200) then HP(20k) is nearly silent; parallel also sends the signal through the 20 kHz HP
            // which is quiet, but Split with the dry route leaves the saw untouched.
            expect(splitDry > serial * 3.0f);
            expect(splitDry > splitF1 * 3.0f);
            expect(parallel < splitDry);
        }

        beginTest("filter off passes the dry signal and mix blends");
        {
            auto v = ParameterMap::defaultValues();
            setValue(v, "osc1_pos", 1.0f);
            setValue(v, "filter_cutoff", 100.0f);
            setValue(v, "filter_on", 0.0f);
            CRUCIBLE_HEAP(SynthEngine, off);
            prepare(off, v);
            off.noteOn(69, 1.0f);
            renderBlocks(off, 4800);
            const float bypass = renderBlocks(off, 9600).peak;
            setValue(v, "filter_on", 1.0f);
            CRUCIBLE_HEAP(SynthEngine, on);
            prepare(on, v);
            on.noteOn(69, 1.0f);
            renderBlocks(on, 4800);
            const float filtered = renderBlocks(on, 9600).peak;
            setValue(v, "filter_mix", 0.5f);
            CRUCIBLE_HEAP(SynthEngine, half);
            prepare(half, v);
            half.noteOn(69, 1.0f);
            renderBlocks(half, 4800);
            const float mixed = renderBlocks(half, 9600).peak;
            expect(bypass > filtered * 2.0f);
            expect(mixed > filtered && mixed < bypass);
        }

        beginTest("every filter model through the full voice stays finite with unison and modulation");
        for (int m = 0; m < models; ++m)
        {
            auto v = ParameterMap::defaultValues();
            setValue(v, "filter_type", static_cast<float>(m));
            setValue(v, "filter2_on", 1.0f);
            setValue(v, "filter2_type", static_cast<float>((m + 5) % models));
            setValue(v, "filter_res", 1.0f);
            setValue(v, "filter2_res", 1.0f);
            setValue(v, "filter_drive", 1.0f);
            setValue(v, "filter2_drive", 0.7f);
            setValue(v, "osc1_uni_voices", 8.0f);
            setValue(v, "mod1_src", 1.0f);
            setValue(v, "mod1_dst", 7.0f);
            setValue(v, "mod1_amt", 1.0f);
            setValue(v, "lfo1_rate", 25.0f);
            CRUCIBLE_HEAP(SynthEngine, e);
            prepare(e, v);
            for (int n = 0; n < 6; ++n)
                e.noteOn(30 + 7 * n, 1.0f);
            const auto r = renderBlocks(e, 24000, 77);
            expect(r.finite && r.peak < 50.0f, "model " + String(m) + " peak " + String(r.peak));
        }
    }
};

static FilterTests filterTests;
