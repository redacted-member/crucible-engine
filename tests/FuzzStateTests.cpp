// SPDX-License-Identifier: MIT
// Fuzz findings F3 (sequencer parameters set from the audio thread), F5 (default outside the range with one oscillator) and
// F6 (user wavetable frames not stable through save / load).
#include <crucible/crucible.h>
#include "TestSupport.h"

#include <cmath>
#include <string>
#include <vector>

using namespace crucible::testing;

namespace
{
std::string savePatch(CrucibleEngine* e)
{
    const int n = crucible_save_patch(e, "t", nullptr, 0);
    std::vector<char> buf(static_cast<size_t>(std::max(n, 0)) + 1, '\0');
    crucible_save_patch(e, "t", buf.data(), buf.size());
    return std::string(buf.data());
}

// "bars" of the first saved pattern (slot 0).
int firstBars(const std::string& patch)
{
    const size_t at = patch.find("\"bars\":");
    return at == std::string::npos ? -1 : std::atoi(patch.c_str() + at + 7);
}

bool loadPatch(CrucibleEngine* e, const std::string& text)
{
    char err[256] = {0};
    return crucible_load_patch(e, text.c_str(), err, sizeof(err)) != 0;
}

void processEvents(CrucibleEngine* e, const std::vector<CrucibleParamEvent>& events)
{
    std::vector<float> l(256), r(256);
    crucible_process_events(e, l.data(), r.data(), 256, nullptr, 0, events.data(), static_cast<int>(events.size()), nullptr);
}
} // namespace

class FuzzStateTests : public UnitTest
{
public:
    FuzzStateTests() : UnitTest("Fuzz state fixes", "Engine") {}

    void runTest() override
    {
        beginTest("F3: seq_length / seq_pattern set by parameter events save, load and save identically");
        {
            CrucibleEngine* e = crucible_create(nullptr);
            crucible_prepare(e, 48000.0, 256);
            const int length = crucible_param_index(e, "seq_length");
            const int pattern = crucible_param_index(e, "seq_pattern");
            expect(length >= 0 && pattern >= 0);
            // A host that forgets crucible_service: the save (and the load that follows it) must still be a fixed point.
            processEvents(e, {{0, length, 3.0f}});
            crucible_service(e);
            processEvents(e, {{0, length, 1.0f}});
            std::string a = savePatch(e);
            expect(firstBars(a) == 1, "the saved pattern has the length the parameter says (1), not " + std::to_string(firstBars(a)));
            expect(loadPatch(e, a));
            expect(savePatch(e) == a, "length set by event");
            processEvents(e, {{0, length, 7.0f}});
            crucible_service(e);
            processEvents(e, {{0, pattern, 2.0f}});
            a = savePatch(e);
            expect(firstBars(a) == 7, "pattern 0 keeps its length until the selected slot is serviced or saved");
            expect(loadPatch(e, a));
            expect(savePatch(e) == a, "pattern selected by event");
            processEvents(e, {{0, length, 5.0f}, {0, pattern, 0.0f}});
            a = savePatch(e);
            expect(loadPatch(e, a));
            expect(savePatch(e) == a, "length and pattern in one block");
            crucible_destroy(e);
        }

        beginTest("F5: every default lies inside its range, also with one oscillator or one filter");
        for (const char* layout : {"{\"oscillators\":1,\"filters\":1}", "{\"oscillators\":1,\"filters\":2}",
                                   "{\"oscillators\":2,\"filters\":1}", "{\"oscillators\":3,\"filters\":2}"})
        {
            const std::string json = std::string("{\"format\":\"crucible-product\",\"version\":1,\"id\":\"f5\",\"layout\":") +
                                     layout + "}";
            char err[512] = {0};
            CrucibleEngine* e = crucible_create_from_product(nullptr, json.c_str(), nullptr, err, sizeof(err));
            expect(e != nullptr, std::string("product builds: ") + layout + " " + err);
            if (e == nullptr)
                continue;
            for (int i = 0; i < crucible_param_count(e); ++i)
            {
                CrucibleParamMeta m{};
                m.struct_size = sizeof(m);
                crucible_param_meta(e, i, &m);
                expect(m.default_value >= m.min && m.default_value <= m.max,
                       std::string(layout) + ": " + m.id + " default " + std::to_string(m.default_value) + " outside [" +
                           std::to_string(m.min) + ", " + std::to_string(m.max) + "]");
                expect(crucible_get_param(e, i) >= m.min && crucible_get_param(e, i) <= m.max,
                       std::string(layout) + ": " + m.id + " stored value outside its range");
            }
            crucible_destroy(e);
        }

        beginTest("F6: a user wavetable is bit-stable through save / load, and an old-format patch loads unchanged");
        {
            CrucibleEngine* e = crucible_create(nullptr);
            crucible_prepare(e, 48000.0, 256);
            const int frameSize = 2048;
            const int frameCount = 28;
            std::vector<float> frames(static_cast<size_t>(frameSize * frameCount));
            for (int f = 0; f < frameCount; ++f)
                for (int i = 0; i < frameSize; ++i)
                {
                    const double ph = 6.283185307179586 * i / frameSize;
                    frames[static_cast<size_t>(f * frameSize + i)] = static_cast<float>(
                        std::sin(ph) * (1.0 - f / 40.0) + std::sin(3.0 * ph + f * 0.3) * 0.3 * f / frameCount);
                }
            CrucibleWavetableInfo info{};
            info.struct_size = sizeof(info);
            char err[256] = {0};
            expect(crucible_load_wavetable(e, 0, frames.data(), frameCount, frameSize, "stable", &info, err, sizeof(err)) != 0,
                   err);
            // The format is unchanged (the 64 base frames, text encoded): this is what every earlier release wrote.
            const std::string old = savePatch(e);
            expect(old.find("stable") != std::string::npos, "the patch holds the user wavetable");
            std::string previous = old;
            for (int round = 0; round < 3; ++round)
            {
                expect(loadPatch(e, previous), "loads");
                const std::string next = savePatch(e);
                expect(next == previous, "save / load / save is identical (round " + std::to_string(round) + ")");
                previous = next;
            }
            CrucibleEngine* f = crucible_create(nullptr);
            crucible_prepare(f, 48000.0, 256);
            expect(loadPatch(f, old), "an old-format patch with a user wavetable loads into a fresh engine");
            expect(savePatch(f) == old, "and saves back unchanged");
            crucible_destroy(f);
            crucible_destroy(e);
        }
    }
};

static FuzzStateTests fuzzStateTests;
