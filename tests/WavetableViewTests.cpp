// SPDX-License-Identifier: MIT
// The "oscN/wavetable" view: the 64 frames of the table an oscillator slot selects (factory or user), decimated to 256
// points, and the live frame position that "params/oscN_pos.mod" / ".voice" / ".band" already give.

#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/UserWavetables.h>
#include <crucible/Views.h>

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr int kPoints = 256;
constexpr int kFrames = WavetableBank::numFrames;

void run(Engine& e, int blocks, int size = 256)
{
    std::vector<float> l(static_cast<size_t>(size)), r(static_cast<size_t>(size));
    for (int b = 0; b < blocks; ++b)
        e.process(l.data(), r.data(), size);
}

std::vector<float> readFull(const Engine& e, int view)
{
    std::vector<float> buffer(static_cast<size_t>(e.views().info(view).maxElements));
    const int n = e.views().read(view, buffer.data(), static_cast<int>(buffer.size()), nullptr);
    buffer.resize(static_cast<size_t>(std::max(n, 0)));
    return buffer;
}

std::vector<float> sineCycle(int length)
{
    std::vector<float> v(static_cast<size_t>(length));
    for (int i = 0; i < length; ++i)
        v[static_cast<size_t>(i)] = 0.5f * static_cast<float>(std::sin(6.283185307179586 * i / length));
    return v;
}
} // namespace

class WavetableViewTests : public UnitTest
{
public:
    WavetableViewTests() : UnitTest("Views wavetable", "Engine") {}

    void runTest() override
    {
        beginTest("osc1/wavetable is a pulled vector: frame count, then frames x 256 points");
        {
            Engine e;
            e.prepare(kRate, 256);
            const int v = e.views().find("osc1", "wavetable");
            expect(v >= 0, "view exists");
            const auto& info = e.views().info(v);
            expect(info.kind == ViewKind::Vector && !info.live && info.maxElements == 1 + kFrames * kPoints,
                   "pulled vector of 1 + 64 x 256 elements");
            expect(e.views().find("osc2", "wavetable") >= 0, "every oscillator slot has one");
        }

        beginTest("a factory table: the points are the table's base frames, every 8th sample");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("osc1_table", 3.0f);
            run(e, 2);
            const auto got = readFull(e, e.views().find("osc1", "wavetable"));
            expect(got.size() == static_cast<size_t>(1 + kFrames * kPoints), "full payload");
            expect(!got.empty() && got[0] == static_cast<float>(kFrames), "header holds the frame count");
            bool same = got.size() == static_cast<size_t>(1 + kFrames * kPoints);
            const auto& bank = WavetableBank::instance();
            for (int f : {0, 17, 63})
                for (int i = 0; same && i < kPoints; ++i)
                    same = got[static_cast<size_t>(1 + f * kPoints + i)] ==
                           bank.frame(3, 0, f)[i * (WavetableBank::tableSize / kPoints)];
            expect(same, "points match WavetableBank frames");
            e.setParam("osc1_table", 5.0f);
            run(e, 2);
            expect(readFull(e, e.views().find("osc1", "wavetable")) != got, "following the table parameter");
        }

        beginTest("a user table is read, an empty user slot is an empty payload, osc2 is independent");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("osc1_table", static_cast<float>(WavetableBank::numTables + 1));
            run(e, 2);
            const int v = e.views().find("osc1", "wavetable");
            expect(readFull(e, v).empty(), "empty slot: nothing");
            expect(e.userWavetables().loadCycles(1, {sineCycle(2048)}, WavetableImport::Normalise::None, "sine"));
            const auto got = readFull(e, v);
            expect(got.size() == static_cast<size_t>(1 + kFrames * kPoints), "loaded");
            const auto* t = e.userWavetables().table(1);
            bool same = t != nullptr;
            for (int i = 0; same && i < kPoints; ++i)
                same =
                    got[static_cast<size_t>(1 + 5 * kPoints + i)] == t->baseFrames()[static_cast<size_t>(5 * 2048 + i * 8)];
            expect(same, "points match the user table's base frames");
            e.userWavetables().clear(1);
            expect(readFull(e, v).empty(), "cleared again");
        }

        beginTest("the live frame position is params/osc1_pos .mod / .voice / .band");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("osc1_pos", 0.375f);
            run(e, 2);
            for (const char* id : {"osc1_pos.mod", "osc1_pos.voice", "osc1_pos.band"})
                expect(e.views().find("params", id) >= 0, id);
            e.views().subscribe(e.views().find("params", "osc1_pos.mod"), 0.0f);
            run(e, 2);
            const auto got = readFull(e, e.views().find("params", "osc1_pos.mod"));
            expect(got.size() == 1 && std::abs(got[0] - 0.375f) < 1.0e-4f, "unmodulated position");
        }
    }
};

static WavetableViewTests wavetableViewTests;
