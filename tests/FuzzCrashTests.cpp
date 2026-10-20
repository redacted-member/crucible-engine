// SPDX-License-Identifier: MIT
// Fuzz findings F1 (57..64 voices overran per-voice arrays) and F2 (infinite / huge tempo or position hung the sequencer).
#include <crucible/Engine.h>
#include <crucible/crucible.h>
#include "TestSupport.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

using namespace crucible::testing;

namespace
{
// Runs `job` on a helper thread and waits up to `seconds`. A job that is still running is abandoned (the thread is detached
// and the shared state kept alive), so a hang fails the test instead of hanging the suite.
bool finishesWithin(double seconds, std::function<void()> job)
{
    auto done = std::make_shared<std::atomic<bool>>(false);
    std::thread worker(
        [done, job = std::move(job)]
        {
            job();
            done->store(true);
        });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!done->load() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (done->load())
    {
        worker.join();
        return true;
    }
    worker.detach();
    return false;
}

struct Engine
{
    CrucibleEngine* e = nullptr;
    explicit Engine(int voices)
    {
        CrucibleConfig c;
        crucible_config_init(&c);
        c.max_voices = voices;
        e = crucible_create(&c);
        crucible_prepare(e, 48000.0, 256);
    }
    ~Engine() { crucible_destroy(e); }
};
} // namespace

class FuzzCrashTests : public UnitTest
{
public:
    FuzzCrashTests() : UnitTest("Fuzz crash fixes", "Engine") {}

    void runTest() override
    {
        beginTest("F1: 57..64 voices with MPE and stealing stay in bounds and finite");
        for (const int voices : {56, 57, 64, 200})
        {
            Engine eng(voices);
            crucible_set_param_id(eng.e, "mpe_on", 1.0f);
            std::vector<float> l(256), r(256);
            bool finite = true;
            for (int n = 0; n < 128; ++n)
            {
                crucible_midi(eng.e, static_cast<uint8_t>(0x91 + (n % 15)), static_cast<uint8_t>(n), 100);
                crucible_midi(eng.e, static_cast<uint8_t>(0xd1 + (n % 15)), 40, 0);
                crucible_midi(eng.e, static_cast<uint8_t>(0xa1 + (n % 15)), static_cast<uint8_t>(n), 30);
            }
            for (int b = 0; b < 8; ++b)
            {
                crucible_process(eng.e, l.data(), r.data(), 256, nullptr, 0);
                for (size_t i = 0; i < 256; ++i)
                    finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
            }
            expect(finite, "output finite with max_voices " + std::to_string(voices));
        }

        beginTest("F2: infinite or huge tempo / position does not hang process");
        const double inf = std::numeric_limits<double>::infinity();
        struct Case
        {
            const char* name;
            double bpm, ppq;
            int sync;
        };
        for (const Case& c : {Case{"internal bpm inf", inf, -1.0, 0}, Case{"internal bpm 1e300", 1.0e300, -1.0, 0},
                              Case{"host ppq inf", 120.0, inf, 1}, Case{"host ppq 1e300", 120.0, 1.0e300, 1}})
        {
            auto eng = std::make_shared<Engine>(8);
            crucible_set_param_id(eng->e, "seq_run", 1.0f);
            crucible_set_param_id(eng->e, "seq_sync", static_cast<float>(c.sync));
            const Case cc = c;
            const bool ok = finishesWithin(5.0,
                                           [eng, cc]
                                           {
                                               std::vector<float> l(256), r(256);
                                               crucible_process_host(eng->e, l.data(), r.data(), 256, nullptr, 0, cc.bpm,
                                                                     cc.ppq, 1);
                                               CrucibleTransport t{};
                                               t.struct_size = sizeof(t);
                                               t.bpm = cc.bpm;
                                               t.ppq = cc.ppq;
                                               t.playing = 1;
                                               crucible_process_events(eng->e, l.data(), r.data(), 256, nullptr, 0, nullptr,
                                                                       0, &t);
                                               crucible_process_aux(eng->e, l.data(), r.data(), nullptr, nullptr, 256,
                                                                    nullptr, 0, cc.bpm, cc.ppq, 1);
                                           });
            expect(ok, std::string("returns in time: ") + c.name);
            if (!ok)
                break; // the abandoned thread still owns the engine
        }

        beginTest("F2: the sequencer itself survives an infinite tempo or position (C++ API)");
        for (const bool sync : {false, true})
        {
            auto e = std::make_shared<crucible::Engine>();
            e->prepare(48000.0, 256);
            e->setParam("seq_run", 1.0f);
            e->setParam("seq_sync", sync ? 1.0f : 0.0f);
            const bool ok = finishesWithin(5.0,
                                           [e]
                                           {
                                               std::vector<float> l(256), r(256);
                                               crucible::SeqHostTime host;
                                               host.hasBpm = true;
                                               host.bpm = std::numeric_limits<double>::infinity();
                                               host.hasPpq = true;
                                               host.ppq = std::numeric_limits<double>::infinity();
                                               host.playing = true;
                                               e->process(l.data(), r.data(), 256, nullptr, 0, &host);
                                           });
            expect(ok, sync ? "host sync returns in time" : "internal sync returns in time");
            if (!ok)
                break;
        }
    }
};

static FuzzCrashTests fuzzCrashTests;
