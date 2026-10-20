// SPDX-License-Identifier: MIT
// crucible-measure: renders a measurement job file with the engine, so an external analysis suite can score the
// engine without a plugin host.
//
//   crucible-measure --job job.json --out-dir renders/ [--voices N] [--quality 0|1|2] [--seed N]
//
// The job is the file an external analysis suite writes: parameters are addressed by
// display name (or id); numbers are normalised values, strings are display texts ("Lowpass 24", "1000", "On").
// Output: <name>.wav (stereo, 32-bit float) per render and manifest.json with timing in the host's format
// (wall time of process(), CPU time of the process, worst block), then
// which the analysis suite then reads.

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <crucible/AudioFile.h>
#include <crucible/Version.h>
#include <crucible/Engine.h>
#include <crucible/Json.h>
#include "../WinArgv.h"
#include "../../src/Utf8Path.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <map>
#include <sstream>

#if defined(__APPLE__)
#include <pthread.h>
#include <pthread/qos.h>
#endif

namespace
{
using namespace crucible;

std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string trim(const std::string& s)
{
    const auto a = s.find_first_not_of(" \t");
    const auto b = s.find_last_not_of(" \t");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

std::map<std::string, int> nameIndex(const Engine& engine)
{
    std::map<std::string, int> m;
    for (int i = 0; i < engine.model().size(); ++i)
    {
        const auto& p = engine.model().params()[static_cast<size_t>(i)];
        m.emplace(lower(p.name), i);
        m.emplace(lower(p.id), i);
    }
    return m;
}

// Display text to natural value.
bool parseText(const ParamDef& p, const std::string& raw, float& out)
{
    const std::string s = trim(raw);
    if (p.type == ParamType::Choice)
    {
        for (size_t i = 0; i < p.choices.size(); ++i)
            if (lower(p.choices[i]) == lower(s))
            {
                out = static_cast<float>(i);
                return true;
            }
    }
    if (p.type == ParamType::Bool)
    {
        const auto l = lower(s);
        if (l == "on" || l == "1" || l == "true")
            out = 1.0f;
        else if (l == "off" || l == "0" || l == "false")
            out = 0.0f;
        else
            return false;
        return true;
    }
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str())
        return false;
    out = static_cast<float>(v);
    return true;
}

struct Event
{
    long long sample;
    std::uint8_t status, d1, d2;
};

#if defined(_WIN32)
double fileTimeSeconds(const FILETIME& kernel, const FILETIME& user)
{
    const auto ticks = [](const FILETIME& f)
    { return static_cast<double>((static_cast<unsigned long long>(f.dwHighDateTime) << 32) | f.dwLowDateTime); };
    return (ticks(kernel) + ticks(user)) * 1.0e-7; // 100 ns units
}

// CPU seconds of the calling thread / of every thread of the process; neither grows while a thread is preempted.
double threadCpuSeconds()
{
    FILETIME creation{}, exit{}, kernel{}, user{};
    GetThreadTimes(GetCurrentThread(), &creation, &exit, &kernel, &user);
    return fileTimeSeconds(kernel, user);
}

double processCpuSeconds()
{
    FILETIME creation{}, exit{}, kernel{}, user{};
    GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user);
    return fileTimeSeconds(kernel, user);
}
#else
double cpuSecondsOf(clockid_t clock)
{
    timespec ts{};
    clock_gettime(clock, &ts);
    return static_cast<double>(ts.tv_sec) + 1.0e-9 * static_cast<double>(ts.tv_nsec);
}

// CPU seconds of the calling thread / of every thread of the process; neither grows while a thread is preempted.
double threadCpuSeconds()
{
    return cpuSecondsOf(CLOCK_THREAD_CPUTIME_ID);
}

double processCpuSeconds()
{
    return cpuSecondsOf(CLOCK_PROCESS_CPUTIME_ID);
}
#endif
} // namespace

int main(int argc, char** argv)
{
    const crucible::tools::Utf8Args utf8Args(argc, argv);
    argc = utf8Args.argc;
    argv = utf8Args.argv;
#if defined(__APPLE__)
    // Keeps the render thread on a performance core, as the host's audio thread would be.
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    std::string jobPath, outDir = ".";
    EngineConfig cfg;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--version")
        {
            std::printf("%s (Crucible Engine) %s\n", "crucible-measure", CRUCIBLE_VERSION_STRING);
            return 0;
        }
        const auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : std::string(); };
        if (a == "--job")
            jobPath = next();
        else if (a == "--out-dir")
            outDir = next();
        else if (a == "--voices")
            cfg.maxVoices = std::atoi(next().c_str());
        else if (a == "--quality")
            cfg.quality = static_cast<Quality>(std::clamp(std::atoi(next().c_str()), 0, 2));
        else if (a == "--seed")
            cfg.seed = static_cast<std::uint32_t>(std::strtoul(next().c_str(), nullptr, 10));
    }
    if (jobPath.empty())
    {
        std::fprintf(stderr,
                     "usage: crucible-measure --job job.json --out-dir DIR [--voices N] [--quality 0|1|2] [--seed N]\n");
        return 2;
    }
    std::ifstream f(crucible::utf8Path(jobPath));
    std::stringstream buffer;
    buffer << f.rdbuf();
    json::Value job;
    std::string error;
    if (!json::parse(buffer.str(), job, error))
    {
        std::fprintf(stderr, "job: %s\n", error.c_str());
        return 2;
    }
    const double rate = job["sampleRate"].asNumber(48000.0);
    const int block = static_cast<int>(job["blockSize"].asNumber(512.0));
    const double warmup = job["warmup"].asNumber(0.5);
    Engine layout(cfg);
    const auto names = nameIndex(layout);

    json::Value manifest = json::Value::object();
    json::Value renders = json::Value::array();
    int failures = 0;

    const auto apply = [&](Engine& e, const json::Value& set, std::string& problem)
    {
        for (const auto& [name, value] : set.entries())
        {
            const auto it = names.find(lower(name));
            if (it == names.end())
            {
                problem = "unknown parameter " + name;
                return false;
            }
            const auto& p = e.model().params()[static_cast<size_t>(it->second)];
            if (value.isNumber())
                e.setParamNormalised(it->second, static_cast<float>(value.asNumber()));
            else
            {
                float v = 0.0f;
                if (!parseText(p, value.asString(), v))
                {
                    problem = "cannot read '" + value.asString() + "' for " + name;
                    return false;
                }
                e.setParam(it->second, v);
            }
        }
        return true;
    };

    for (const auto& r : job["renders"].elements())
    {
        const std::string name = r["name"].asString("");
        json::Value entry = json::Value::object();
        entry.set("name", name);
        Engine e(cfg);
        e.prepare(rate, block);
        std::string problem;
        if (!apply(e, job["set"], problem) || !apply(e, r["set"], problem))
        {
            entry.set("error", problem);
            renders.push(entry);
            ++failures;
            continue;
        }

        // Events in samples.
        std::vector<Event> events;
        const auto at = [&](double t) { return static_cast<long long>(std::llround(t * rate)); };
        for (const auto& n : r["notes"].elements())
        {
            const int pitch = static_cast<int>(n["pitch"].asNumber());
            const int vel = static_cast<int>(n["velocity"].asNumber(100.0));
            events.push_back(
                {at(n["start"].asNumber()), 0x90, static_cast<std::uint8_t>(pitch), static_cast<std::uint8_t>(vel)});
            events.push_back(
                {at(n["start"].asNumber() + n["length"].asNumber()), 0x80, static_cast<std::uint8_t>(pitch), 0});
        }
        for (const auto& b : r["bend"].elements())
        {
            const int raw = std::clamp(static_cast<int>(std::lround((b["value"].asNumber() + 1.0) * 8192.0)), 0, 16383);
            events.push_back({at(b["time"].asNumber()), 0xE0, static_cast<std::uint8_t>(raw & 0x7f),
                              static_cast<std::uint8_t>(raw >> 7)});
        }
        for (const auto& c : r["cc"].elements())
            events.push_back({at(c["time"].asNumber()), 0xB0, static_cast<std::uint8_t>(c["cc"].asNumber()),
                              static_cast<std::uint8_t>(c["value"].asNumber())});
        std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.sample < b.sample; });

        struct Ramp
        {
            int index;
            float from, to;
            long long start, end;
        };
        std::vector<Ramp> ramps;
        for (const auto& rp : r["ramps"].elements())
        {
            const auto it = names.find(lower(rp["param"].asString("")));
            if (it != names.end())
                ramps.push_back({it->second, static_cast<float>(rp["from"].asNumber()),
                                 static_cast<float>(rp["to"].asNumber()), at(rp["start"].asNumber()),
                                 at(rp["end"].asNumber())});
        }

        std::vector<float> l(static_cast<size_t>(block)), rr(static_cast<size_t>(block));
        // Warm-up: silence after the parameter changes so the smoothing settles.
        for (long long done = 0; done < at(warmup); done += block)
            e.process(l.data(), rr.data(), block);

        const long long total = at(r["seconds"].asNumber(1.0));
        AudioData audio;
        audio.sampleRate = rate;
        audio.channels.assign(2, {});
        audio.channels[0].reserve(static_cast<size_t>(total));
        audio.channels[1].reserve(static_cast<size_t>(total));
        size_t nextEvent = 0;
        double wall = 0.0, worst = 0.0, threadCpu = 0.0;
        double processCpu = 0.0;
        long long nonFinite = 0;
        for (long long pos = 0; pos < total; pos += block)
        {
            const int n = static_cast<int>(std::min<long long>(block, total - pos));
            for (const auto& rp : ramps)
            {
                const double t =
                    rp.end > rp.start
                        ? std::clamp(static_cast<double>(pos - rp.start) / static_cast<double>(rp.end - rp.start), 0.0, 1.0)
                        : (pos >= rp.start ? 1.0 : 0.0);
                if (pos >= rp.start - block)
                    e.setParamNormalised(rp.index, rp.from + (rp.to - rp.from) * static_cast<float>(t));
            }
            MidiInput midi[64];
            int count = 0;
            while (nextEvent < events.size() && events[nextEvent].sample < pos + n && count < 64)
            {
                const auto& ev = events[nextEvent++];
                midi[count++] = {static_cast<int>(std::max<long long>(0, ev.sample - pos)), ev.status, ev.d1, ev.d2};
            }
            const auto t0 = std::chrono::steady_clock::now();
            const double p0 = processCpuSeconds();
            const double c0 = threadCpuSeconds();
            e.process(l.data(), rr.data(), n, count > 0 ? midi : nullptr, count);
            threadCpu += threadCpuSeconds() - c0;
            processCpu += processCpuSeconds() - p0;
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            wall += ms;
            worst = std::max(worst, ms);
            for (int i = 0; i < n; ++i)
            {
                if (!std::isfinite(l[static_cast<size_t>(i)]) || !std::isfinite(rr[static_cast<size_t>(i)]))
                    ++nonFinite;
                audio.channels[0].push_back(l[static_cast<size_t>(i)]);
                audio.channels[1].push_back(rr[static_cast<size_t>(i)]);
            }
        }
        const double cpuSeconds = processCpu;
        const double seconds = static_cast<double>(total) / rate;
        if (!writeWavFile(outDir + "/" + name + ".wav", audio, 32))
        {
            entry.set("error", "cannot write the wav file");
            ++failures;
        }
        entry.set("seconds", seconds);
        entry.set("processMs", wall);
        entry.set("realtimeFactor", seconds * 1000.0 / std::max(wall, 1.0e-9));
        entry.set("cpuPercent", 100.0 * threadCpu / seconds);
        entry.set("cpuPercentAllThreads", 100.0 * cpuSeconds / seconds);
        entry.set("worstBlockMs", worst);
        entry.set("blockBudgetMs", block / rate * 1000.0);
        entry.set("nonFiniteSamples", static_cast<double>(nonFinite));
        entry.set("readback", json::Value::array());
        renders.push(entry);
    }
    manifest.set("renders", renders);
    std::ofstream out(crucible::utf8Path(outDir + "/manifest.json"));
    out << manifest.dump(2);
    std::printf("rendered %zu tests, %d failed\n", job["renders"].size(), failures);
    return failures == 0 ? 0 : 3;
}
