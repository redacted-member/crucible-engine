// SPDX-License-Identifier: MIT
#include <crucible/ParameterMap.h>
#include <crucible/SynthEngine.h>
#include <crucible/Wavetables.h>
#include "TestSupport.h"

#include <cmath>
#include <set>

using namespace crucible;
using namespace crucible::testing;

class ParameterTests : public UnitTest
{
public:
    ParameterTests() : UnitTest("ParameterMap", "Engine") {}

    void runTest() override
    {
        const auto& params = ParameterMap::params();

        beginTest("ids are unique and non-empty");
        std::set<std::string> ids;
        for (const auto& p : params)
        {
            expect(!p.id.empty());
            expect(ids.insert(p.id).second, "duplicate id " + String(p.id));
        }

        beginTest("defaults lie within range");
        for (const auto& p : params)
            expect(p.defaultValue >= p.min && p.defaultValue <= p.max, "default out of range " + String(p.id));

        beginTest("pages hold at most six parameters and every parameter has a page");
        for (size_t page = 0; page < ParameterMap::pages().size(); ++page)
        {
            const auto n = ParameterMap::paramsOnPage(static_cast<int>(page)).size();
            expect(n >= 1 && n <= static_cast<size_t>(ParameterMap::knobsPerPage));
        }
        for (const auto& p : params)
            expect(p.page >= 0 && p.page < static_cast<int>(ParameterMap::pages().size()));

        beginTest("page sections and layout hints are valid");
        for (const auto& page : ParameterMap::pages())
        {
            expect(page.section >= 0 && page.section < static_cast<int>(ParameterMap::sections().size()));
            expect(!page.layoutTitle.empty() && !page.layoutDescription.empty());
        }
        expect(ParameterMap::sections().size() <= 8);

        beginTest("choice parameters have consistent choice lists");
        for (const auto& p : params)
            if (p.type == ParamType::Choice)
                expect(static_cast<size_t>(p.max) == p.choices.size() - 1);

        beginTest("normalised mapping round-trips and snaps discrete parameters");
        for (const auto& p : params)
        {
            const float back = ParameterMap::fromNormalised(p, ParameterMap::toNormalised(p, p.defaultValue));
            expectWithinAbsoluteError(back, p.defaultValue, 1.0e-3f * (1.0f + std::abs(p.max - p.min)), String(p.id));
            if (p.type != ParamType::Float)
            {
                const float mid = ParameterMap::fromNormalised(p, 0.37f);
                expectEquals(mid, std::round(mid), String(p.id));
            }
        }

        beginTest("JSON export lists every page and accepts host extras");
        {
            const auto plain = ParameterMap::toJson();
            expect(plain.find("\"pages\"") != std::string::npos && plain.find("\"hardware\"") == std::string::npos);
            ParameterMap::JsonExtras extras;
            extras.pageBlock = [](int page, int) { return std::string("{\"p\": ") + std::to_string(page) + "}"; };
            extras.globalBlock = [](int) { return std::string("{}"); };
            const auto withExtras = ParameterMap::toJson(extras);
            expect(withExtras.find("\"hardware\": {\"p\": 0}") != std::string::npos);
            expect(withExtras.find("\"hardware\": {}") != std::string::npos);
        }
    }
};

class WavetableTests : public UnitTest
{
public:
    WavetableTests() : UnitTest("Wavetables", "Engine") {}

    void runTest() override
    {
        const auto& bank = WavetableBank::instance();

        beginTest("all frames are finite and normalised");
        bool ok = true;
        for (int t = 0; t < WavetableBank::numTables; ++t)
            for (int m = 0; m < WavetableBank::numMips; ++m)
                for (int f = 0; f < WavetableBank::numFrames; ++f)
                {
                    const float* d = bank.frame(t, m, f);
                    for (int i = 0; i <= WavetableBank::mipSize(m); ++i)
                        if (!std::isfinite(d[i]) || std::abs(d[i]) > 3.0f)
                            ok = false;
                }
        expect(ok);

        beginTest("first frame of table 0 is a sine");
        {
            const float* d = bank.frame(0, 0, 0);
            float peak = 0.0f;
            const int size = WavetableBank::mipSize(0);
            for (int i = 0; i < size; ++i)
                peak = std::max(peak, std::abs(d[i]));
            expectWithinAbsoluteError(peak, 1.0f, 1.0e-3f);
            float maxDiff = 0.0f;
            for (int i = 0; i < size; ++i)
                maxDiff = std::max(maxDiff, std::abs(std::abs(d[i]) - std::abs(std::sin(6.2831853f * static_cast<float>(i) /
                                                                                        static_cast<float>(size)))));
            expect(maxDiff < 1.0e-3f, "sine shape error " + String(maxDiff));
        }

        beginTest("high mips contain fewer harmonics than mip 0");
        {
            const auto roughness = [&](int mip)
            {
                const float* d = bank.frame(0, mip, WavetableBank::numFrames - 1);
                float sum = 0.0f;
                const int size = WavetableBank::mipSize(mip);
                for (int i = 1; i < size; ++i)
                    sum += std::abs(d[i] - d[i - 1]);
                return sum;
            };
            expect(roughness(WavetableBank::numMips - 6) < roughness(0));
        }
    }
};

class EngineTests : public UnitTest
{
public:
    EngineTests() : UnitTest("SynthEngine", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("note-on renders audio and release reaches silence");
        {
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, ParameterMap::defaultValues());
            engine.noteOn(60, 0.8f);
            const auto on = renderBlocks(engine, static_cast<int>(kRate * 0.5));
            expect(on.finite);
            expect(on.peak > 0.01f, "peak " + String(on.peak));
            expectEquals(engine.activeVoiceCount(), 1);

            engine.noteOff(60);
            renderBlocks(engine, static_cast<int>(kRate * 1.0));
            const auto tail = renderBlocks(engine, static_cast<int>(kRate * 0.25));
            expect(tail.peak < 1.0e-6f, "tail peak " + String(tail.peak));
            expectEquals(engine.activeVoiceCount(), 0);
        }

        beginTest("every wavetable renders audio");
        for (int table = 0; table < WavetableBank::numTables; ++table)
        {
            auto values = ParameterMap::defaultValues();
            setValue(values, "osc1_table", static_cast<float>(table));
            setValue(values, "osc1_pos", 0.7f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, values);
            engine.noteOn(48, 1.0f);
            const auto r = renderBlocks(engine, 24000);
            expect(r.finite && r.peak > 0.01f);
        }

        beginTest("1 second with all unison voices, full polyphony and modulation is finite");
        {
            auto values = ParameterMap::defaultValues();
            for (const char* osc : {"osc1", "osc2"})
            {
                setValue(values, std::string(osc) + "_uni_voices", 8.0f);
                setValue(values, std::string(osc) + "_uni_detune", 100.0f);
                setValue(values, std::string(osc) + "_uni_spread", 1.0f);
                setValue(values, std::string(osc) + "_level", 1.0f);
                setValue(values, std::string(osc) + "_pos", 0.5f);
            }
            setValue(values, "osc2_table", 3.0f);
            setValue(values, "sub_level", 1.0f);
            setValue(values, "noise_level", 1.0f);
            setValue(values, "filter_res", 1.0f);
            setValue(values, "filter_drive", 1.0f);
            setValue(values, "filter_env", 1.0f);
            setValue(values, "filter_cutoff", 400.0f);
            setValue(values, "lfo1_shape", 4.0f);
            setValue(values, "lfo2_rate", 30.0f);
            const std::pair<int, int> routes[] = {{1, 1}, {2, 3}, {3, 7}, {1, 12}};
            for (int s = 0; s < 4; ++s)
            {
                const auto prefix = "mod" + std::to_string(s + 1);
                setValue(values, prefix + "_src", static_cast<float>(routes[s].first));
                setValue(values, prefix + "_dst", static_cast<float>(routes[s].second));
                setValue(values, prefix + "_amt", 1.0f);
            }

            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, values);
            for (int n = 0; n < kNumVoices + 4; ++n)
                engine.noteOn(24 + n * 5, 1.0f);
            const auto r = renderBlocks(engine, static_cast<int>(kRate));
            expect(r.finite, "non-finite sample");
            expect(r.peak > 0.001f);
            expect(r.peak < 100.0f, "peak " + String(r.peak));
            expect(engine.activeVoiceCount() <= kNumVoices);
        }

        beginTest("filter modes and extreme notes stay finite");
        for (int type = 0; type < 3; ++type)
            for (const int note : {0, 127})
            {
                auto values = ParameterMap::defaultValues();
                setValue(values, "filter_type", static_cast<float>(type));
                setValue(values, "filter_res", 1.0f);
                setValue(values, "osc1_uni_voices", 8.0f);
                CRUCIBLE_HEAP(SynthEngine, engine);
                prepare(engine, values);
                engine.noteOn(note, 1.0f);
                const auto r = renderBlocks(engine, 24000, 97);
                expect(r.finite);
            }
    }
};

static ParameterTests parameterTests;
static WavetableTests wavetableTests;
static EngineTests engineTests;
