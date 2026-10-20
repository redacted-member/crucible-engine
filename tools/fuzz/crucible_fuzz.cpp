// SPDX-License-Identifier: MIT
// crucible-fuzz: a seeded, reproducible fuzz / soak harness for the plain-C API (include/crucible/crucible.h).
//
// One run = one seed. The setup (product, config, sample rate, block size) comes from the seed; then a sequence of
// steps, each drawn from its own generator derived from (seed, step), drives the engine through the public API:
// parameter values (extremes, NaN / Inf included), parameter-event streams, MIDI, block sizes, patch round trips,
// routes, reset, re-prepare, views, content loads. Because every step has its own generator, a subset of steps can be
// replayed (--keep) and every kept step does exactly what it did in the full run; tools/fuzz/minimise.py uses that to
// shrink a failure to a few steps.
//
// Checks after every process call: every output sample finite and |x| <= --bound (default 256, +48 dBFS); no heap
// allocation on the calling thread inside the process call. After every patch round trip: save -> load -> save gives
// the same text and the same parameter values. Sanitizer reports abort the process (-fno-sanitize-recover=all).
// A check failure prints "FUZZ-FAIL <kind> seed=<s> step=<k> ..." and exits with status 3.
//
// Usage: crucible-fuzz --root <repo> [--seed S | --seeds A:B] [--steps N] [--keep i,j,..] [--trace] [--bound X]
//                      [--seconds T] [--stats] [--progress] [--no-alloc-stacks] [--strict-frames] [--strict-meta]
//                      [--step-timeout S] [--avoid-known] [--product FILE] [--hardware-preset]

#include <crucible/crucible.h>

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <new>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define CRUCIBLE_FUZZ_ASAN 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__)
#define CRUCIBLE_FUZZ_ASAN 1
#endif

#if CRUCIBLE_FUZZ_ASAN
// Declared here rather than through <sanitizer/*.h>, which not every toolchain ships.
extern "C" int __sanitizer_install_malloc_and_free_hooks(void (*malloc_hook)(const volatile void*, size_t),
                                                         void (*free_hook)(const volatile void*));
extern "C" void __sanitizer_print_stack_trace();
extern "C" void __sanitizer_set_death_callback(void (*callback)(void));
#endif

// ---------------------------------------------------------------------------------------------------------------------
// Allocation counting. Under AddressSanitizer the allocator hooks see every malloc / new (C and C++); elsewhere the
// replaced global operator new counts C++ allocations only.
namespace
{
thread_local bool gTrack = false;
thread_local long gAllocs = 0;
thread_local bool gStackPrinted = false;
bool gPrintAllocStacks = true;

void noteAllocation()
{
    if (!gTrack)
        return;
    ++gAllocs;
#if CRUCIBLE_FUZZ_ASAN
    if (gPrintAllocStacks && !gStackPrinted)
    {
        gStackPrinted = true;
        gTrack = false; // printing allocates
        std::fprintf(stderr, "FUZZ-ALLOC-STACK (first allocation inside process):\n");
        __sanitizer_print_stack_trace();
        gTrack = true;
    }
#endif
}
} // namespace

#if CRUCIBLE_FUZZ_ASAN
extern "C" void fuzzMallocHook(const volatile void*, size_t)
{
    noteAllocation();
}
extern "C" void fuzzFreeHook(const volatile void*) {}
#else
void* operator new(std::size_t n)
{
    noteAllocation();
    if (void* p = std::malloc(n == 0 ? 1 : n))
        return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n)
{
    noteAllocation();
    if (void* p = std::malloc(n == 0 ? 1 : n))
        return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept
{
    std::free(p);
}
void operator delete[](void* p) noexcept
{
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept
{
    std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept
{
    std::free(p);
}
#endif

namespace
{
// ---------------------------------------------------------------------------------------------------------------------
// Deterministic generator (splitmix64 seeding, xoshiro256**): identical streams on every platform and compiler.
uint64_t splitmix(uint64_t& x)
{
    uint64_t z = (x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

struct Rng
{
    uint64_t s[4];
    explicit Rng(uint64_t seed)
    {
        for (auto& v : s)
            v = splitmix(seed);
    }
    static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
    uint64_t next()
    {
        const uint64_t r = rotl(s[1] * 5, 7) * 9;
        const uint64_t t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = rotl(s[3], 45);
        return r;
    }
    // [0, n)
    uint32_t below(uint32_t n) { return n == 0 ? 0 : static_cast<uint32_t>((next() >> 32) % n); }
    int range(int lo, int hi) { return lo + static_cast<int>(below(static_cast<uint32_t>(hi - lo + 1))); } // [lo, hi]
    double unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }              // [0, 1)
    float uniform(float lo, float hi) { return lo + static_cast<float>(unit()) * (hi - lo); }
    bool chance(double p) { return unit() < p; }
};

uint64_t stepSeed(uint64_t seed, uint64_t step)
{
    uint64_t x = seed * 0xD1B54A32D192ED03ull + step * 0x9E3779B97F4A7C15ull + 0x632BE59BD9B4E019ull;
    splitmix(x);
    return splitmix(x);
}

constexpr uint64_t kSetupStream = 0xFFFFFFFFull;

// ---------------------------------------------------------------------------------------------------------------------
struct Options
{
    std::string root = ".";
    uint64_t seedFirst = 1, seedLast = 1;
    int steps = 400;
    bool trace = false;
    bool stats = false;
    bool strictFrames = false;
    bool strictMeta = false; // a default outside its parameter's range fails instead of being noted (known F-META1)
    std::string product;     // --product <file>: always this product (relative to --root)
    bool hwPreset = false;        // --hardware-preset: a fixed-host hardware preset (48 kHz, blocks 64..1024, 16 voices, its MIDI subset)
    bool avoidKnown = false; // leave out the inputs of findings already reported, so a long run finds new ones
    unsigned stepTimeout = 30; // seconds one step may take before it counts as a hang (0: no watchdog)
    bool progress = false; // "FUZZ-STEP k" before every step: names the step of a UBSan abort, which runs no hook
    float bound = 256.0f;
    double seconds = 0.0; // > 0: stop starting new seeds after this much wall time
    bool haveKeep = false;
    std::set<int> keep;
};

Options gOpt;
uint64_t gSeed = 0;
int gStep = -1;
std::string gStepName;

#if CRUCIBLE_FUZZ_ASAN
// Runs when a sanitizer report ends the process: names the seed and step for tools/fuzz/minimise.py.
void onSanitizerDeath()
{
    std::fprintf(stderr, "FUZZ-DEATH seed=%" PRIu64 " step=%d action=%s\n", gSeed, gStep, gStepName.c_str());
}
#endif

} // namespace

#if CRUCIBLE_FUZZ_ASAN
// Sanitizer hooks: stack traces on for UBSan, and every report (ASan or UBSan) tagged with the seed and step.
extern "C" const char* __ubsan_default_options()
{
    return "print_stacktrace=1:halt_on_error=1";
}
extern "C" void __sanitizer_report_error_summary(const char* summary)
{
    std::fprintf(stderr, "%s\n", summary);
    onSanitizerDeath();
}
#endif

namespace
{
// SIGALRM: a step ran past --step-timeout. Only async-signal-safe calls here.
void onAlarm(int)
{
    char buf[160];
    const int len = std::snprintf(buf, sizeof buf, "FUZZ-FAIL hang seed=%" PRIu64 " step=%d (over %u s in one step)\n",
                                  gSeed, gStep, gOpt.stepTimeout);
    if (len > 0)
        (void)!write(2, buf, static_cast<size_t>(len));
    _exit(3);
}

[[noreturn]] void fail(const char* kind, const std::string& detail)
{
    std::fprintf(stderr, "FUZZ-FAIL %s seed=%" PRIu64 " step=%d action=%s %s\n", kind, gSeed, gStep, gStepName.c_str(),
                 detail.c_str());
    std::fflush(stderr);
    std::fflush(stdout);
    std::_Exit(3);
}

void trace(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void trace(const char* fmt, ...)
{
    if (!gOpt.trace)
        return;
    std::fprintf(stderr, "[%d] ", gStep);
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
}

std::string readFile(const std::filesystem::path& p)
{
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string fmtFloat(float v)
{
    char b[64];
    std::snprintf(b, sizeof b, "%.9g", static_cast<double>(v));
    return b;
}

// ---------------------------------------------------------------------------------------------------------------------
// Random values

float extremeFloat(Rng& r)
{
    static const float table[] = {std::numeric_limits<float>::quiet_NaN(),
                                  -std::numeric_limits<float>::quiet_NaN(),
                                  std::numeric_limits<float>::infinity(),
                                  -std::numeric_limits<float>::infinity(),
                                  std::numeric_limits<float>::max(),
                                  -std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::denorm_min(),
                                  1e-40f,
                                  -0.0f,
                                  0.0f,
                                  1e30f,
                                  -1e30f,
                                  1e9f,
                                  -1e9f};
    return table[r.below(sizeof table / sizeof table[0])];
}

struct ParamInfo
{
    std::string id;
    int type = 0;
    float min = 0, max = 1, def = 0;
    int choices = 0;
    int routable = 0;
    bool defaultOutOfRange = false;
};

// A value for parameter p: mostly inside its range, sometimes on the edges, out of range or non-finite.
float randomValue(Rng& r, const ParamInfo& p, bool allowNonFinite)
{
    const uint32_t m = r.below(100);
    const float span = p.max - p.min;
    if (m < 45)
    {
        if (p.type != 0)
            return static_cast<float>(r.range(static_cast<int>(std::floor(p.min)), static_cast<int>(std::ceil(p.max))));
        return r.uniform(p.min, p.max);
    }
    if (m < 60)
        return p.min;
    if (m < 75)
        return p.max;
    if (m < 82)
        return p.def;
    if (m < 90)
        return r.chance(0.5) ? p.min - span * r.uniform(0.0f, 10.0f) : p.max + span * r.uniform(0.0f, 10.0f);
    if (m < 93 && p.type != 0)
        return static_cast<float>(r.range(-3, p.choices + 3)) + (r.chance(0.3) ? 0.5f : 0.0f);
    if (allowNonFinite)
        return extremeFloat(r);
    return r.chance(0.5) ? p.min : p.max;
}

// ---------------------------------------------------------------------------------------------------------------------
// Products

struct ProductSource
{
    std::string name;
    std::string json;
};

std::vector<ProductSource> gProductFiles;
std::vector<std::string> gEffectTypes;

void loadProductFiles()
{
    namespace fs = std::filesystem;
    const fs::path root(gOpt.root);
    std::vector<fs::path> files;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(root / "products", ec))
        if (e.path().extension() == ".json")
            files.push_back(e.path());
    std::sort(files.begin(), files.end());
    files.push_back(root / "docs" / "products" / "template.json");
    for (const auto& f : files)
    {
        std::string text = readFile(f);
        if (text.empty())
        {
            std::fprintf(stderr, "crucible-fuzz: cannot read %s\n", f.string().c_str());
            std::exit(2);
        }
        gProductFiles.push_back({f.filename().string(), std::move(text)});
    }

    // The effect types of this build, from the default product's module list (public API only).
    CrucibleEngine* e = crucible_create(nullptr);
    if (e == nullptr)
    {
        std::fprintf(stderr, "crucible-fuzz: crucible_create(NULL) failed\n");
        std::exit(2);
    }
    std::set<std::string> types;
    for (int m = 0; m < crucible_module_count(e); ++m)
    {
        CrucibleModuleMeta meta{};
        meta.struct_size = sizeof meta;
        if (crucible_module_meta(e, m, &meta) && meta.kind != nullptr && std::strcmp(meta.kind, "effect") == 0 &&
            meta.id != nullptr)
        {
            std::string id = meta.id;
            if (auto dot = id.find('.', 3); dot != std::string::npos) // "fx.convolution.2" -> "fx.convolution"
                id = id.substr(0, dot);
            types.insert(id);
        }
    }
    crucible_destroy(e);
    // Types that only appear in non-default products.
    for (const char* t : {"fx.delay_pro", "fx.ensemble", "fx.phaser2", "fx.flanger2", "fx.saturator", "fx.exciter",
                          "fx.transient", "fx.deesser", "fx.reverb_algo", "fx.comp_clean", "fx.eq_pro", "fx.convolution"})
        types.insert(t);
    gEffectTypes.assign(types.begin(), types.end());
}

// A product built from random effect instances and lanes; sometimes deliberately invalid (the compiler must refuse it
// cleanly).
std::string generateProduct(Rng& r, std::string& label)
{
    std::ostringstream o;
    const int osc = r.chance(0.1) ? r.range(-1, 10) : r.range(1, 3);
    const int filt = r.chance(0.1) ? r.range(-1, 4) : r.range(1, 2);
    const int macros = r.chance(0.1) ? r.range(-1, 10) : (r.chance(0.5) ? 6 : 8);
    const bool voice = r.chance(0.8);
    const int numFx = r.range(0, 8);
    std::vector<std::string> ids;
    o << "{\"format\":\"crucible-product\",\"version\":1,\"id\":\"fuzz.gen\",\"name\":\"Fuzz\",";
    o << "\"layout\":{\"oscillators\":" << osc << ",\"filters\":" << filt << ",\"macros\":" << macros << "},";
    o << "\"voice\":\"" << (voice ? "classic" : "none") << "\",\"fx\":{\"instances\":[";
    for (int i = 0; i < numFx; ++i)
    {
        const std::string& t = gEffectTypes[r.below(static_cast<uint32_t>(gEffectTypes.size()))];
        ids.push_back("i" + std::to_string(i));
        o << (i ? "," : "") << "{\"id\":\"" << ids.back() << "\",\"type\":\"" << t << "\"}";
    }
    o << "],\"lanes\":[";
    const int lanes = numFx == 0 ? 1 : r.range(1, 3);
    std::vector<std::string> remaining = ids;
    for (int l = 0; l < lanes; ++l)
    {
        o << (l ? "," : "") << "{\"id\":\"l" << l << "\",\"order\":\"" << (r.chance(0.5) ? "listed" : "position")
          << "\",\"input\":[";
        const uint32_t in = r.below(4);
        if (in == 0 || (l == 0 && in != 3))
            o << "\"voices\"";
        else if (in == 1)
            o << "\"aux\"";
        else if (in == 2)
            o << "\"voices\",\"aux\"";
        o << "],\"chain\":[";
        int taken = 0;
        while (!remaining.empty() && (l == lanes - 1 || r.chance(0.6)))
        {
            o << (taken++ ? "," : "") << "\"" << remaining.front() << "\"";
            remaining.erase(remaining.begin());
        }
        o << "]";
        if (r.chance(0.3))
            o << ",\"level\":" << fmtFloat(r.uniform(0.0f, 2.0f));
        o << "}";
    }
    o << "]";
    if (lanes > 1 && r.chance(0.5))
        o << ",\"sends\":[{\"from\":\"l0\",\"to\":\"l" << r.range(1, lanes - 1)
          << "\",\"gain\":" << fmtFloat(r.uniform(0.0f, 1.0f)) << "}]";
    o << "}}";
    label = "generated(osc=" + std::to_string(osc) + ",filt=" + std::to_string(filt) + ",macros=" +
            std::to_string(macros) + ",fx=" + std::to_string(numFx) + ",lanes=" + std::to_string(lanes) + ")";
    return o.str();
}

// Byte-level damage to a JSON text: truncation, flips, deletions, duplicated spans, injected tokens.
std::string mutateText(Rng& r, std::string s)
{
    const int edits = r.range(1, 6);
    static const char* tokens[] = {"NaN", "1e999", "-1e999", "null", "[]", "{}", "\"\"", "\"\\u0000\"", "-0", "1e-400",
                                   "999999999999999999999", "\"\\ud800\"", ",", ":", "{", "}", "[", "]", "\""};
    for (int k = 0; k < edits && !s.empty(); ++k)
    {
        const size_t pos = r.below(static_cast<uint32_t>(s.size()));
        switch (r.below(6))
        {
            case 0: s.resize(pos); break;
            case 1: s[pos] = static_cast<char>(r.below(256)); break;
            case 2: s.erase(pos, std::min<size_t>(r.range(1, 32), s.size() - pos)); break;
            case 3:
            {
                const size_t len = std::min<size_t>(r.range(1, 64), s.size() - pos);
                s.insert(r.below(static_cast<uint32_t>(s.size())), s.substr(pos, len));
                break;
            }
            default: s.insert(pos, tokens[r.below(sizeof tokens / sizeof tokens[0])]); break;
        }
    }
    return s;
}

// ---------------------------------------------------------------------------------------------------------------------
// The harness state for one run.

struct Run
{
    CrucibleEngine* e = nullptr;
    std::string productLabel;
    std::string productJson;
    CrucibleConfig cfg{};
    double sr = 48000.0;
    int maxBlock = 512;
    std::vector<ParamInfo> params;
    std::vector<int> routable;
    std::vector<std::pair<int, std::string>> routes; // added routes (source, target)
    std::vector<int> subs;                           // view subscriptions
    CrucibleKeyBus* bus = nullptr;
    double ppq = 0.0;
    float peak = 0.0f;
    long blocks = 0, samples = 0;
    long roundTrips = 0;
    long framesDrift = 0;
    std::vector<float> L, R, AL, AR;
};

void describeParams(Run& run)
{
    run.params.clear();
    run.routable.clear();
    const int n = crucible_param_count(run.e);
    for (int i = 0; i < n; ++i)
    {
        CrucibleParamMeta m{};
        m.struct_size = sizeof m;
        if (!crucible_param_meta(run.e, i, &m))
            fail("meta", "crucible_param_meta failed for index " + std::to_string(i));
        ParamInfo p;
        p.id = m.id != nullptr ? m.id : "";
        p.type = m.type;
        p.min = m.min;
        p.max = m.max;
        p.def = m.default_value;
        p.choices = m.num_choices;
        p.routable = m.routable;
        if (!std::isfinite(p.min) || !std::isfinite(p.max) || p.min > p.max)
            fail("meta", "non-finite or inverted range for " + p.id);
        if (!(p.def >= p.min && p.def <= p.max))
        {
            if (gOpt.strictMeta)
                fail("meta", "default " + fmtFloat(p.def) + " outside [" + fmtFloat(p.min) + ", " + fmtFloat(p.max) +
                                 "] for " + p.id);
            p.defaultOutOfRange = true;
            std::fprintf(stderr, "FUZZ-NOTE meta-default seed=%" PRIu64 " %s default %s outside [%s, %s]\n", gSeed,
                         p.id.c_str(), fmtFloat(p.def).c_str(), fmtFloat(p.min).c_str(), fmtFloat(p.max).c_str());
        }
        if (p.routable)
            run.routable.push_back(i);
        run.params.push_back(std::move(p));
    }
}

bool createEngine(Run& run, Rng& r)
{
    crucible_config_init(&run.cfg);
    run.cfg.max_voices = r.chance(0.1) ? r.range(-2, 70) : r.range(1, 16);
    if (gOpt.avoidKnown) // F-VOICES1: 57..64 voices (and anything clamped to them) overrun the per-voice arrays
        run.cfg.max_voices = std::min(run.cfg.max_voices, 56);
    run.cfg.max_block_size = r.chance(0.1) ? r.range(-5, 8192) : r.range(1, 1024);
    run.cfg.quality = r.chance(0.1) ? r.range(-1, 5) : r.range(0, 2);
    run.cfg.seed = static_cast<uint32_t>(r.next());
    run.cfg.oversampling_filter = r.range(0, 1);
    if (r.chance(0.05))
        run.cfg.struct_size = r.below(sizeof run.cfg); // older, shorter struct: the rest takes defaults

    if (gOpt.hwPreset)
    {
        run.cfg.max_voices = 16; // the hardware preset: 16 voices, 8 of them polyphonic
        run.cfg.max_block_size = 1024;
        run.cfg.struct_size = sizeof run.cfg;
    }
    const uint32_t pick = gOpt.product.empty() ? r.below(100) : 0;
    bool generated = false;
    if (!gOpt.product.empty())
    {
        run.productLabel = gOpt.product;
        run.productJson = readFile(std::filesystem::path(gOpt.root) / gOpt.product);
        if (run.productJson.empty())
            fail("create", "cannot read " + gOpt.product);
    }
    else if (pick < 65)
    {
        const auto& f = gProductFiles[r.below(static_cast<uint32_t>(gProductFiles.size()))];
        run.productLabel = f.name;
        run.productJson = f.json;
    }
    else if (pick < 92)
    {
        run.productJson = generateProduct(r, run.productLabel);
        generated = true;
    }
    else
    {
        const auto& f = gProductFiles[r.below(static_cast<uint32_t>(gProductFiles.size()))];
        run.productLabel = "mutated(" + f.name + ")";
        run.productJson = mutateText(r, f.json);
        generated = true;
    }
    const size_t errSize = r.chance(0.2) ? r.below(16) : 4096;
    std::vector<char> err(errSize == 0 ? 1 : errSize);
    run.e = crucible_create_from_product(&run.cfg, run.productJson.c_str(), nullptr, errSize == 0 ? nullptr : err.data(),
                                         errSize);
    if (errSize > 0 && std::memchr(err.data(), 0, err.size()) == nullptr)
        fail("api", "error_buffer not NUL terminated");
    trace("create product=%s voices=%d block=%d quality=%d os=%d -> %s", run.productLabel.c_str(), run.cfg.max_voices,
          run.cfg.max_block_size, run.cfg.quality, run.cfg.oversampling_filter, run.e ? "ok" : err.data());
    if (run.e == nullptr)
    {
        if (!generated)
            fail("create", "product file refused: " + run.productLabel + ": " + std::string(err.data()));
        return false;
    }
    describeParams(run);
    return true;
}

void prepare(Run& run, Rng& r)
{
    static const double rates[] = {44100.0, 48000.0, 96000.0, 192000.0};
    run.sr = rates[r.below(4)];
    run.maxBlock = r.chance(0.15) ? r.range(1, 64) : r.range(64, 2048);
    if (gOpt.hwPreset)
    {
        run.sr = 48000.0;
        run.maxBlock = 1024;
    }
    trace("prepare sr=%.0f maxBlock=%d", run.sr, run.maxBlock);
    crucible_prepare(run.e, run.sr, run.maxBlock);
}

void checkOutput(Run& run, int n, const char* what)
{
    for (int i = 0; i < n; ++i)
    {
        const float a = run.L[static_cast<size_t>(i)], b = run.R[static_cast<size_t>(i)];
        if (!std::isfinite(a) || !std::isfinite(b))
            fail("nonfinite", std::string(what) + " sample " + std::to_string(i) + "/" + std::to_string(n) + " L=" +
                                  fmtFloat(a) + " R=" + fmtFloat(b) + " block=" + std::to_string(run.blocks));
        const float m = std::max(std::fabs(a), std::fabs(b));
        run.peak = std::max(run.peak, m);
        if (m > gOpt.bound)
            fail("bound", std::string(what) + " |x|=" + fmtFloat(m) + " > " + fmtFloat(gOpt.bound) + " at sample " +
                              std::to_string(i) + " block=" + std::to_string(run.blocks));
    }
}

void randomMidi(Rng& r, int n, std::vector<CrucibleMidiEvent>& ev, bool allowRealtime)
{
    int count = r.chance(0.3) ? 0 : r.range(0, 24);
    if (r.chance(0.01))
        count = r.range(1000, 1100); // past CRUCIBLE_MAX_EVENTS_PER_BLOCK: the excess must be dropped
    ev.resize(static_cast<size_t>(count));
    for (auto& e : ev)
    {
        e.position = n > 0 ? r.range(0, n - 1) : 0;
        static const uint8_t kinds[] = {0x80, 0x90, 0x90, 0x90, 0xA0, 0xB0, 0xB0, 0xC0, 0xD0, 0xE0};
        uint8_t status = static_cast<uint8_t>(kinds[r.below(sizeof kinds)] | r.below(16));
        if (allowRealtime && r.chance(0.02))
            status = static_cast<uint8_t>(r.chance(0.5) ? 0xF8 + r.below(8) : r.below(0x80)); // realtime / no status
        e.status = status;
        e.data1 = static_cast<uint8_t>(r.chance(0.02) ? r.below(256) : r.below(128));
        e.data2 = static_cast<uint8_t>(r.chance(0.02) ? r.below(256) : (r.chance(0.2) ? (r.chance(0.5) ? 0 : 127) : r.below(128)));
        if (gOpt.hwPreset) // note on / off, poly and channel pressure, bend, sustain, all notes off; valid bytes only
        {
            static const uint8_t hwMidi[] = {0x80, 0x90, 0x90, 0x90, 0xA0, 0xD0, 0xE0, 0xB0, 0xB0};
            e.status = static_cast<uint8_t>(hwMidi[r.below(sizeof hwMidi)] | r.below(16));
            e.data1 = static_cast<uint8_t>(r.below(128));
            e.data2 = static_cast<uint8_t>(r.below(128));
            if ((e.status & 0xF0) == 0xB0)
                e.data1 = r.chance(0.8) ? 64 : 123;
        }
        if ((status & 0xF0) == 0xB0 && r.chance(0.3)) // favour the CCs that mean something
        {
            static const uint8_t ccs[] = {1, 7, 10, 11, 64, 66, 74, 120, 121, 123};
            e.data1 = ccs[r.below(sizeof ccs)];
        }
    }
    std::stable_sort(ev.begin(), ev.end(),
                     [](const CrucibleMidiEvent& a, const CrucibleMidiEvent& b) { return a.position < b.position; });
}

void randomParamEvents(Rng& r, Run& run, int n, std::vector<CrucibleParamEvent>& pe)
{
    const int count = r.chance(0.4) ? 0 : r.range(1, 32);
    pe.resize(static_cast<size_t>(count));
    const int np = static_cast<int>(run.params.size());
    for (auto& p : pe)
    {
        p.position = r.chance(0.05) ? r.range(-100, n + 100) : r.range(0, std::max(0, n - 1));
        if (np == 0 || r.chance(0.03))
        {
            static const int bad[] = {-1, -1000, std::numeric_limits<int>::min(), std::numeric_limits<int>::max()};
            p.index = r.chance(0.5) ? bad[r.below(4)] : np + r.range(0, 10);
            p.value = r.uniform(-1.0f, 1.0f);
        }
        else
        {
            p.index = r.range(0, np - 1);
            p.value = randomValue(r, run.params[static_cast<size_t>(p.index)], true);
        }
    }
    std::stable_sort(pe.begin(), pe.end(),
                     [](const CrucibleParamEvent& a, const CrucibleParamEvent& b) { return a.position < b.position; });
}

void processStep(Run& run, Rng& r)
{
    int n = r.chance(0.1) ? r.range(1, 8) : (r.chance(0.2) ? run.maxBlock : r.range(1, run.maxBlock));
    if (r.chance(0.01))
        n = 0;
    if (gOpt.hwPreset)
        n = r.range(64, 1024);
    std::vector<CrucibleMidiEvent> midi;
    std::vector<CrucibleParamEvent> pe;
    randomMidi(r, n, midi, true);
    // Exact-size heap buffers, so that AddressSanitizer catches any write past num_samples.
    run.L.assign(static_cast<size_t>(std::max(n, 1)), 0.0f);
    run.R.assign(static_cast<size_t>(std::max(n, 1)), 0.0f);
    if (r.chance(0.2)) // garbage in the output buffers must not leak into the output
        for (int i = 0; i < n; ++i)
            run.L[static_cast<size_t>(i)] = run.R[static_cast<size_t>(i)] = extremeFloat(r);
    const uint32_t variant = r.below(10);
    CrucibleTransport tr{};
    tr.struct_size = sizeof tr;
    double bpm = r.chance(0.3) ? -1.0 : r.uniform(20.0f, 300.0f);
    if (r.chance(0.03) && !gOpt.avoidKnown) // F-HANG1: infinite / huge tempo
    {
        static const double odd[] = {0.0, 1e-9, 1e9, std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::quiet_NaN(), 0.001, 99999.0};
        bpm = odd[r.below(7)];
    }
    if (r.chance(0.05))
        run.ppq = r.chance(0.5) ? -1.0 : r.uniform(0.0f, 1e6f);
    if (r.chance(0.01) && !gOpt.avoidKnown) // F-HANG1 also covers an infinite position
        run.ppq = r.chance(0.5) ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::quiet_NaN();
    else if (run.ppq >= 0.0 && bpm > 0.0 && std::isfinite(bpm))
        run.ppq += n / run.sr * bpm / 60.0;
    tr.bpm = bpm;
    tr.ppq = run.ppq;
    tr.playing = r.chance(0.7);
    if (r.chance(0.05))
        tr.struct_size = r.below(sizeof tr + 1);
    const CrucibleMidiEvent* mp = midi.empty() ? (r.chance(0.5) ? nullptr : midi.data()) : midi.data();
    const int nm = static_cast<int>(midi.size());

    const char* what = "";
    if (variant < 5)
    {
        randomParamEvents(r, run, n, pe);
        trace("process_events n=%d midi=%d params=%d bpm=%g ppq=%g play=%d tsize=%zu", n, nm, static_cast<int>(pe.size()),
              bpm, run.ppq, tr.playing, tr.struct_size);
        for (const auto& p : pe)
            trace("    param %d (%s) = %s @%d", p.index,
                  p.index >= 0 && p.index < static_cast<int>(run.params.size())
                      ? run.params[static_cast<size_t>(p.index)].id.c_str()
                      : "?",
                  fmtFloat(p.value).c_str(), p.position);
        for (const auto& m : midi)
            trace("    midi %02X %02X %02X @%d", m.status, m.data1, m.data2, m.position);
        what = "process_events";
        const bool noTransport = r.chance(0.1);
        gAllocs = 0;
        gStackPrinted = false;
        gTrack = true;
        crucible_process_events(run.e, run.L.data(), run.R.data(), n, mp, nm, pe.empty() ? nullptr : pe.data(),
                                static_cast<int>(pe.size()), noTransport ? nullptr : &tr);
        gTrack = false;
    }
    else if (variant < 7)
    {
        trace("process n=%d midi=%d", n, nm);
        for (const auto& m : midi)
            trace("    midi %02X %02X %02X @%d", m.status, m.data1, m.data2, m.position);
        what = "process";
        gAllocs = 0;
        gStackPrinted = false;
        gTrack = true;
        crucible_process(run.e, run.L.data(), run.R.data(), n, mp, nm);
        gTrack = false;
    }
    else if (variant < 8)
    {
        trace("process_host n=%d midi=%d bpm=%g ppq=%g play=%d", n, nm, bpm, run.ppq, tr.playing);
        for (const auto& m : midi)
            trace("    midi %02X %02X %02X @%d", m.status, m.data1, m.data2, m.position);
        what = "process_host";
        gAllocs = 0;
        gStackPrinted = false;
        gTrack = true;
        crucible_process_host(run.e, run.L.data(), run.R.data(), n, mp, nm, bpm, run.ppq, tr.playing);
        gTrack = false;
    }
    else
    {
        run.AL.assign(static_cast<size_t>(std::max(n, 1)), 0.0f);
        run.AR.assign(static_cast<size_t>(std::max(n, 1)), 0.0f);
        const uint32_t sig = r.below(5);
        const float amp = sig == 4 ? r.uniform(1.0f, 8.0f) : r.uniform(0.0f, 1.0f);
        const float f = r.uniform(20.0f, 18000.0f);
        for (int i = 0; i < n; ++i)
        {
            float x = 0.0f;
            switch (sig)
            {
                case 0: x = 0.0f; break;
                case 1: x = amp * static_cast<float>(std::sin(6.283185307 * f * (run.samples + i) / run.sr)); break;
                case 2: x = amp * r.uniform(-1.0f, 1.0f); break;
                case 3: x = (i == 0) ? amp : 0.0f; break;
                default: x = (i & 1) ? amp : -amp; break; // hot square at Nyquist
            }
            run.AL[static_cast<size_t>(i)] = x;
            run.AR[static_cast<size_t>(i)] = r.chance(0.5) ? x : -x;
        }
        const uint32_t layout = r.below(4); // stereo, mono (right NULL), none (left NULL), stereo
        trace("process_aux n=%d midi=%d sig=%u amp=%g layout=%u", n, nm, sig, amp, layout);
        for (const auto& m : midi)
            trace("    midi %02X %02X %02X @%d", m.status, m.data1, m.data2, m.position);
        what = "process_aux";
        gAllocs = 0;
        gStackPrinted = false;
        gTrack = true;
        crucible_process_aux(run.e, run.L.data(), run.R.data(), layout == 2 ? nullptr : run.AL.data(),
                             layout == 1 ? nullptr : run.AR.data(), n, mp, nm, bpm, run.ppq, tr.playing);
        gTrack = false;
    }
    if (gAllocs != 0)
        fail("alloc", std::string(what) + " allocated " + std::to_string(gAllocs) + " time(s), n=" + std::to_string(n));
    checkOutput(run, n, what);
    ++run.blocks;
    run.samples += n;
    // A host services after blocks when work is pending (an offline render has no timer); sometimes it forgets.
    if (crucible_service_pending(run.e) && r.chance(0.8))
        crucible_service(run.e);
}

void setParamStep(Run& run, Rng& r)
{
    const int np = static_cast<int>(run.params.size());
    if (np == 0)
        return;
    const int burst = r.chance(0.2) ? r.range(2, 40) : 1;
    for (int b = 0; b < burst; ++b)
    {
        int i = r.range(0, np - 1);
        if (r.chance(0.02))
            i = r.chance(0.5) ? -1 : np;
        const ParamInfo* p = (i >= 0 && i < np) ? &run.params[static_cast<size_t>(i)] : nullptr;
        const uint32_t how = r.below(10);
        if (how < 6 || p == nullptr)
        {
            const float v = p ? randomValue(r, *p, true) : 0.5f;
            trace("set_param %d (%s) = %s", i, p ? p->id.c_str() : "?", fmtFloat(v).c_str());
            crucible_set_param(run.e, i, v);
        }
        else if (how < 8)
        {
            const float v = r.chance(0.1) ? extremeFloat(r) : r.uniform(-0.2f, 1.2f);
            trace("set_param_normalised %d (%s) = %s", i, p->id.c_str(), fmtFloat(v).c_str());
            crucible_set_param_normalised(run.e, i, v);
        }
        else if (how < 9)
        {
            const float v = randomValue(r, *p, false);
            trace("set_param_id %s = %s", p->id.c_str(), fmtFloat(v).c_str());
            crucible_set_param_id(run.e, p->id.c_str(), v);
        }
        else
        {
            // Display text of a random value (the round trip text -> value), or junk.
            std::string text;
            if (r.chance(0.7))
            {
                char buf[128];
                const float v = randomValue(r, *p, true);
                crucible_value_to_text(run.e, i, v, buf, sizeof buf);
                text = buf;
            }
            else
            {
                static const char* junk[] = {"", " ", "nan", "inf", "-inf", "1e999", "C#-9", "G99", "on", "off", "-6dB",
                                             "1.5k", "999999k", "0x10", "\xff\xfe", "Analog", "a", "-", "+", "1e",
                                             "120 bpm", "1/4", "1/0", "-0", "100%", "1000000%"};
                text = junk[r.below(sizeof junk / sizeof junk[0])];
            }
            trace("set_param_text %s = \"%s\"", p->id.c_str(), text.c_str());
            crucible_set_param_text(run.e, p->id.c_str(), text.c_str());
        }
    }
    if (r.chance(0.7))
        crucible_service(run.e);
}

void noteStep(Run& run, Rng& r)
{
    const int count = r.range(1, 20);
    for (int k = 0; k < count; ++k)
    {
        switch (r.below(7))
        {
            case 0:
            case 1:
            {
                const int note = r.chance(0.05) ? r.range(-10, 300) : r.range(0, 127);
                const float vel = r.chance(0.05) ? extremeFloat(r) : r.uniform(0.0f, 1.0f);
                trace("note_on %d %s", note, fmtFloat(vel).c_str());
                crucible_note_on(run.e, note, vel);
                break;
            }
            case 2:
            {
                const int note = r.chance(0.05) ? r.range(-10, 300) : r.range(0, 127);
                trace("note_off %d", note);
                crucible_note_off(run.e, note);
                break;
            }
            case 3:
            {
                const float v = r.chance(0.1) ? extremeFloat(r) : r.uniform(-1.2f, 1.2f);
                trace("pitch_bend %s", fmtFloat(v).c_str());
                crucible_pitch_bend(run.e, v);
                break;
            }
            case 4:
            {
                const float v = r.chance(0.1) ? extremeFloat(r) : r.uniform(-0.2f, 1.2f);
                trace("mod_wheel %s", fmtFloat(v).c_str());
                crucible_mod_wheel(run.e, v);
                break;
            }
            case 5:
            {
                const int d = r.range(-1, 2);
                trace("sustain %d", d);
                crucible_sustain(run.e, d);
                break;
            }
            default:
                if (r.chance(0.3))
                {
                    trace("all_notes_off");
                    crucible_all_notes_off(run.e);
                }
                else
                {
                    static const uint8_t kinds[] = {0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0};
                    const uint8_t st = static_cast<uint8_t>(r.chance(0.03) ? r.below(256) : kinds[r.below(7)] | r.below(16));
                    const uint8_t d1 = static_cast<uint8_t>(r.chance(0.03) ? r.below(256) : r.below(128));
                    const uint8_t d2 = static_cast<uint8_t>(r.chance(0.03) ? r.below(256) : r.below(128));
                    trace("midi %02X %02X %02X", st, d1, d2);
                    crucible_midi(run.e, st, d1, d2);
                }
                break;
        }
    }
}

void mapCcStep(Run& run, Rng& r)
{
    const int np = static_cast<int>(run.params.size());
    const int cc = r.chance(0.05) ? r.range(-5, 200) : r.range(0, 127);
    const int idx = r.chance(0.2) ? -1 : (r.chance(0.03) ? np + 5 : r.range(0, std::max(0, np - 1)));
    trace("map_cc %d -> %d", cc, idx);
    crucible_map_cc(run.e, cc, idx);
}

void routeStep(Run& run, Rng& r)
{
    const uint32_t what = r.below(10);
    if (what < 3 && !run.routes.empty())
    {
        const size_t k = r.below(static_cast<uint32_t>(run.routes.size()));
        trace("remove_route %d %s", run.routes[k].first, run.routes[k].second.c_str());
        crucible_remove_route(run.e, run.routes[k].first, run.routes[k].second.c_str());
        run.routes.erase(run.routes.begin() + static_cast<long>(k));
        return;
    }
    if (what == 3)
    {
        const int src = r.range(-3, 40);
        trace("remove_route %d (junk target)", src);
        crucible_remove_route(run.e, src, r.chance(0.5) ? "" : "no_such_param");
        return;
    }
    int source = r.chance(0.8) ? r.range(0, 25) : (r.chance(0.5) ? 1000 + r.range(0, 127) : r.range(-5, 2000));
    std::string target;
    if (!run.routable.empty() && r.chance(0.9))
        target = run.params[static_cast<size_t>(run.routable[r.below(static_cast<uint32_t>(run.routable.size()))])].id;
    else if (!run.params.empty() && r.chance(0.5))
        target = run.params[r.below(static_cast<uint32_t>(run.params.size()))].id;
    else
        target = r.chance(0.5) ? "" : "bogus_target";
    const float amount = r.chance(0.05) ? extremeFloat(r) : r.uniform(-1.5f, 1.5f);
    int ok = 0;
    if (r.chance(0.5))
    {
        ok = crucible_add_route(run.e, source, target.c_str(), amount);
        trace("add_route %d %s %s -> %d", source, target.c_str(), fmtFloat(amount).c_str(), ok);
    }
    else
    {
        CrucibleRoute rt{};
        rt.struct_size = r.chance(0.1) ? r.below(sizeof rt + 1) : sizeof rt;
        rt.source = source;
        rt.target_id = target.c_str();
        rt.amount = amount;
        rt.polarity = r.range(-1, 4);
        rt.curve = r.chance(0.05) ? extremeFloat(r) : r.uniform(-1.5f, 1.5f);
        rt.smoothing_ms = r.chance(0.05) ? extremeFloat(r) : r.uniform(-10.0f, 3000.0f);
        ok = crucible_add_route_ex(run.e, &rt);
        trace("add_route_ex size=%zu %d %s amt=%s pol=%d curve=%s smooth=%s -> %d", rt.struct_size, source, target.c_str(),
              fmtFloat(amount).c_str(), rt.polarity, fmtFloat(rt.curve).c_str(), fmtFloat(rt.smoothing_ms).c_str(), ok);
    }
    if (ok)
        run.routes.emplace_back(source, target);
}

std::vector<float> snapshotParams(Run& run)
{
    std::vector<float> v(run.params.size());
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = crucible_get_param(run.e, static_cast<int>(i));
    return v;
}

std::string savePatch(Run& run, const char* name)
{
    const int len = crucible_save_patch(run.e, name, nullptr, 0);
    if (len < 0)
        fail("patch", "save_patch(NULL, 0) returned " + std::to_string(len));
    std::vector<char> buf(static_cast<size_t>(len) + 1);
    const int len2 = crucible_save_patch(run.e, name, buf.data(), buf.size());
    if (len2 != len)
        fail("patch", "save_patch length changed between calls: " + std::to_string(len) + " vs " + std::to_string(len2));
    if (buf[static_cast<size_t>(len)] != '\0')
        fail("patch", "save_patch did not NUL terminate");
    return std::string(buf.data(), static_cast<size_t>(len));
}

// A patch line of a parameter whose default lies outside its range (known F-META1: a load clamps it).
bool knownDrift(const Run& run, const std::string& line)
{
    if (gOpt.strictMeta)
        return false;
    for (const auto& p : run.params)
        if (p.defaultOutOfRange && line.find("\"" + p.id + "\":") != std::string::npos)
            return true;
    return false;
}

void patchStep(Run& run, Rng& r)
{
    const uint32_t what = r.below(10);
    if (what < 6)
    {
        // Round trip: save -> load -> save gives the same text and the same stored values.
        const std::string a = savePatch(run, "fuzz");
        const auto before = snapshotParams(run);
        char err[512] = {0};
        const int ok = crucible_load_patch(run.e, a.c_str(), err, sizeof err);
        trace("patch round trip (%zu bytes) -> %d %s", a.size(), ok, err);
        if (!ok)
            fail("patch", std::string("load of a just-saved patch failed: ") + err);
        crucible_service(run.e);
        const std::string b = savePatch(run, "fuzz");
        // Known finding F-RT1: the base64 "frames" of a user wavetable are not bit-stable through a load. Unless
        // --strict-frames is given it is counted and the rest of the text is still compared line by line.
        bool onlyFrames = a != b && !gOpt.strictFrames;
        if (onlyFrames)
        {
            std::istringstream sa(a), sb(b);
            std::string la, lb;
            while (true)
            {
                const bool ga = static_cast<bool>(std::getline(sa, la)), gb = static_cast<bool>(std::getline(sb, lb));
                if (ga != gb)
                {
                    onlyFrames = false;
                    break;
                }
                if (!ga)
                    break;
                if (la != lb && la.find("\"frames\": \"") == std::string::npos && !knownDrift(run, la))
                {
                    onlyFrames = false;
                    break;
                }
            }
        }
        if (onlyFrames)
        {
            if (run.framesDrift++ == 0)
                std::fprintf(stderr, "FUZZ-NOTE roundtrip-frames seed=%" PRIu64 " step=%d\n", gSeed, gStep);
        }
        else if (a != b)
        {
            if (const char* dir = std::getenv("CRUCIBLE_FUZZ_DUMP")) // the two texts, for a diff
            {
                std::ofstream(std::filesystem::path(dir) / "roundtrip-a.json", std::ios::binary) << a;
                std::ofstream(std::filesystem::path(dir) / "roundtrip-b.json", std::ios::binary) << b;
            }
            size_t d = 0;
            while (d < a.size() && d < b.size() && a[d] == b[d])
                ++d;
            auto lineAt = [d](const std::string& t)
            {
                const size_t from = t.rfind('\n', d == 0 ? 0 : d - 1);
                const size_t start = from == std::string::npos ? 0 : from + 1;
                const size_t end = t.find('\n', d);
                std::string line = t.substr(start, (end == std::string::npos ? t.size() : end) - start);
                line.erase(0, line.find_first_not_of(' '));
                return line.size() > 160 ? line.substr(0, 160) + "..." : line;
            };
            fail("roundtrip", "save/load/save text differs at byte " + std::to_string(d) + "\n  A: " + lineAt(a) +
                                  "\n  B: " + lineAt(b));
        }
        const auto after = snapshotParams(run);
        for (size_t i = 0; i < before.size(); ++i)
        {
            const bool same = (std::isnan(before[i]) && std::isnan(after[i])) || before[i] == after[i] ||
                              (run.params[i].defaultOutOfRange && !gOpt.strictMeta);
            if (!same)
                fail("roundtrip", "param " + run.params[i].id + " " + fmtFloat(before[i]) + " -> " + fmtFloat(after[i]));
        }
        ++run.roundTrips;
    }
    else if (what < 8)
    {
        // A damaged patch must be refused or loaded, never crash.
        const std::string a = savePatch(run, "fuzz");
        const std::string m = mutateText(r, a);
        const size_t es = r.chance(0.3) ? r.below(8) : 256;
        std::vector<char> err(std::max<size_t>(es, 1));
        const int ok = crucible_load_patch(run.e, m.c_str(), es ? err.data() : nullptr, es);
        trace("load damaged patch (%zu bytes) -> %d", m.size(), ok);
        crucible_service(run.e);
    }
    else if (what < 9)
    {
        // Short buffers: snprintf contract (NUL terminated, full length returned, nothing written past capacity).
        const int full = crucible_save_patch(run.e, "x", nullptr, 0);
        const size_t cap = r.below(64) + 1;
        std::vector<char> buf(cap, '#');
        const int got = crucible_save_patch(run.e, "x", buf.data(), cap);
        trace("save_patch cap=%zu -> %d (full %d)", cap, got, full);
        if (got != full)
            fail("patch", "save_patch with a short buffer returned " + std::to_string(got) + ", full " + std::to_string(full));
        if (std::memchr(buf.data(), 0, cap) == nullptr)
            fail("patch", "save_patch short buffer not NUL terminated");
    }
    else
    {
        // A random patch name with odd characters.
        std::string name;
        const int len = r.range(0, 40);
        for (int i = 0; i < len; ++i)
            name.push_back(static_cast<char>(r.range(1, 255)));
        const std::string a = savePatch(run, name.c_str());
        char err[256] = {0};
        const int ok = crucible_load_patch(run.e, a.c_str(), err, sizeof err);
        trace("patch with odd name (%d chars) -> %d %s", len, ok, err);
        if (!ok)
            fail("patch", std::string("load of a just-saved patch with an odd name failed: ") + err);
    }
}

void viewStep(Run& run, Rng& r)
{
    const int count = crucible_view_count(run.e);
    const uint32_t what = r.below(6);
    if (what < 2 && count > 0)
    {
        const int v = r.range(0, count - 1);
        const float hz = r.chance(0.2) ? extremeFloat(r) : r.uniform(-10.0f, 200.0f);
        const int s = crucible_view_subscribe(run.e, v, hz);
        trace("view_subscribe %d hz=%s -> %d", v, fmtFloat(hz).c_str(), s);
        if (s > 0)
            run.subs.push_back(s);
    }
    else if (what == 2 && !run.subs.empty())
    {
        const size_t k = r.below(static_cast<uint32_t>(run.subs.size()));
        trace("view_unsubscribe %d", run.subs[k]);
        crucible_view_unsubscribe(run.e, run.subs[k]);
        run.subs.erase(run.subs.begin() + static_cast<long>(k));
    }
    else if (count > 0)
    {
        const int v = r.chance(0.05) ? r.range(-2, count + 2) : r.range(0, count - 1);
        CrucibleViewInfo info{};
        info.struct_size = sizeof info;
        const int has = crucible_view_info(run.e, v, &info);
        const int size = crucible_view_size(run.e, v);
        const int cap = std::max(1, r.chance(0.2) ? r.range(1, 8) : size);
        trace("view_read %d (%s/%s kind=%d) cap=%d", v, has ? info.instance : "?", has ? info.id : "?", has ? info.kind : -1,
              cap);
        uint64_t seq = 0;
        if (has && info.kind == CRUCIBLE_VIEW_MASK)
        {
            std::vector<uint32_t> w(static_cast<size_t>(cap));
            crucible_view_read_mask(run.e, v, w.data(), cap, &seq);
        }
        else
        {
            std::vector<float> f(static_cast<size_t>(cap));
            const int got = crucible_view_read(run.e, v, f.data(), cap, r.chance(0.5) ? &seq : nullptr);
            if (got > cap)
                fail("view", "view_read returned more elements than the capacity");
        }
    }
    if (r.chance(0.2) && !run.params.empty())
    {
        const auto& p = run.params[r.below(static_cast<uint32_t>(run.params.size()))];
        const int v = crucible_view_find(run.e, "params", (p.id + ".voice@" + std::to_string(r.range(-1, 70))).c_str());
        trace("view_find params/%s.voice@ -> %d", p.id.c_str(), v);
    }
}

void queryStep(Run& run, Rng& r)
{
    const int np = static_cast<int>(run.params.size());
    switch (r.below(8))
    {
        case 0:
        {
            const int i = r.chance(0.05) ? r.range(-2, np + 2) : r.range(0, std::max(0, np - 1));
            const float v = (i >= 0 && i < np) ? randomValue(r, run.params[static_cast<size_t>(i)], true) : 0.0f;
            const size_t cap = r.chance(0.3) ? r.below(6) : 64;
            std::vector<char> buf(std::max<size_t>(cap, 1), '#');
            const int len = crucible_value_to_text(run.e, i, v, cap ? buf.data() : nullptr, cap);
            trace("value_to_text %d %s cap=%zu -> %d", i, fmtFloat(v).c_str(), cap, len);
            if (cap > 0 && len >= 0 && std::memchr(buf.data(), 0, cap) == nullptr)
                fail("api", "value_to_text not NUL terminated");
            if (len >= 0 && cap == 64 && len < 63)
            {
                float back = 0.0f;
                crucible_text_to_value(run.e, i, buf.data(), &back);
            }
            break;
        }
        case 1:
        {
            const int cap = r.chance(0.5) ? 0 : r.range(1, 4096);
            std::vector<char> buf(static_cast<size_t>(std::max(cap, 1)));
            const int len = crucible_describe_json(run.e, cap ? buf.data() : nullptr, static_cast<size_t>(cap));
            trace("describe_json cap=%d -> %d", cap, len);
            break;
        }
        case 2:
        {
            const int cap = r.range(0, 16);
            std::vector<int> idx(static_cast<size_t>(std::max(cap, 1))), src(static_cast<size_t>(std::max(cap, 1)));
            const int got = crucible_param_changes(run.e, idx.data(), r.chance(0.5) ? src.data() : nullptr, cap);
            trace("param_changes cap=%d -> %d", cap, got);
            if (got < 0 || got > cap)
                fail("api", "param_changes returned " + std::to_string(got) + " for cap " + std::to_string(cap));
            for (int k = 0; k < got; ++k)
                if (idx[static_cast<size_t>(k)] < CRUCIBLE_CHANGE_TAIL || idx[static_cast<size_t>(k)] >= np)
                    fail("api", "param_changes index out of range: " + std::to_string(idx[static_cast<size_t>(k)]));
            break;
        }
        case 3:
        {
            const int lat = crucible_latency_samples(run.e);
            const double tail = crucible_tail_seconds(run.e);
            trace("latency %d tail %g", lat, tail);
            if (lat < 0 || lat > 10 * 192000)
                fail("api", "latency out of range: " + std::to_string(lat));
            if (std::isnan(tail) || tail < 0.0)
                fail("api", "tail_seconds is " + std::to_string(tail));
            break;
        }
        case 4:
        {
            const int count = r.chance(0.1) ? r.range(-5, 5000) : r.range(1, 4096);
            const int c = std::max(count, 1);
            std::vector<float> a(static_cast<size_t>(c)), b(static_cast<size_t>(c));
            const int got = crucible_scope(run.e, a.data(), b.data(), count);
            trace("scope %d -> %d", count, got);
            if (got > std::max(count, 0))
                fail("api", "scope wrote more than asked");
            break;
        }
        case 5:
        {
            const int fft = r.chance(0.2) ? r.range(-1, 9000) : (256 << r.below(5));
            const int cap = r.chance(0.2) ? r.range(0, 100) : fft / 2 + 1;
            std::vector<float> bins(static_cast<size_t>(std::max(cap, 1)));
            const int got = crucible_spectrum(run.e, fft, bins.data(), cap);
            trace("spectrum %d cap=%d -> %d", fft, cap, got);
            if (got > cap)
                fail("api", "spectrum wrote more than its capacity");
            break;
        }
        case 6:
        {
            const CrucibleMeter m = crucible_meter(run.e);
            const CrucibleLoudness l = crucible_loudness(run.e);
            trace("meter %g %g loudness %g", m.peak_left_db, m.peak_right_db, l.momentary_lufs);
            if (std::isnan(m.peak_left_db) || std::isnan(m.peak_right_db))
                fail("api", "meter is NaN");
            if (r.chance(0.3))
                crucible_reset_clip(run.e);
            if (r.chance(0.2))
                crucible_set_analyzer_enabled(run.e, r.range(0, 1));
            break;
        }
        default:
        {
            int mode = 0, root = 0, following = 0, inKey[12];
            crucible_key_scale(run.e, &mode, &root, inKey, &following);
            int minor = 0;
            float conf = 0;
            crucible_key_detect(run.e, &root, &minor, &conf);
            if (r.chance(0.1))
                crucible_key_detect_reset(run.e);
            if (r.chance(0.3))
            {
                if (run.bus == nullptr)
                {
                    run.bus = crucible_keybus_open_local();
                    crucible_attach_keybus(run.e, run.bus);
                }
                crucible_keybus_publish(run.bus, r.range(-2, 14), static_cast<int>(r.below(1u << 13)), r.range(0, 1));
            }
            trace("key queries");
            break;
        }
    }
}

void contentStep(Run& run, Rng& r)
{
    const int slot = r.chance(0.05) ? r.range(-1, 10) : r.range(0, 2);
    // The hardware preset loads only user wavetables, finite and peak-normalised (64 cycles of 2048 or fewer frames).
    switch (gOpt.hwPreset ? 0u : r.below(6))
    {
        case 0:
        {
            const int frames = r.chance(0.05) ? r.range(0, 3) : r.range(1, 80);
            const int fs = r.chance(0.05) ? r.range(0, 4) : (r.chance(0.5) ? 2048 : r.range(4, 600));
            std::vector<float> data(static_cast<size_t>(std::max(frames * fs, 1)));
            const uint32_t kind = gOpt.hwPreset ? (r.chance(0.5) ? 1u : 3u) : r.below(4);
            for (size_t i = 0; i < data.size(); ++i)
                data[i] = kind == 0   ? 0.0f
                          : kind == 1 ? r.uniform(-1.0f, 1.0f)
                          : kind == 2 ? (r.chance(0.01) ? extremeFloat(r) : r.uniform(-1.0f, 1.0f))
                                      : static_cast<float>(std::sin(6.283185307 * static_cast<double>(i % static_cast<size_t>(std::max(fs, 1))) / std::max(fs, 1)));
            CrucibleWavetableInfo info{};
            info.struct_size = sizeof info;
            char err[256] = {0};
            const int ok = crucible_load_wavetable(run.e, r.range(-1, 8), data.data(), frames, fs, "fuzz", &info, err, sizeof err);
            trace("load_wavetable frames=%d fs=%d kind=%u -> %d %s", frames, fs, kind, ok, err);
            break;
        }
        case 1:
        {
            const int ch = r.range(0, 3);
            const int frames = r.chance(0.05) ? r.range(-1, 2) : r.range(1, 20000);
            std::vector<float> data(static_cast<size_t>(std::max(frames * std::max(ch, 1), 1)));
            for (auto& x : data)
                x = r.chance(0.001) ? extremeFloat(r) : r.uniform(-1.0f, 1.0f);
            const double rate = r.chance(0.1) ? (r.chance(0.5) ? 0.0 : -1.0) : r.uniform(8000.0f, 192000.0f);
            const int ok = crucible_load_sample(run.e, slot, data.data(), frames, ch, rate, "fuzz");
            std::string odd;
            for (size_t i = 0; i < data.size(); ++i)
                if (!(std::fabs(data[i]) <= 1.0f))
                    odd += " [" + std::to_string(i) + "]=" + fmtFloat(data[i]);
            trace("load_sample slot=%d frames=%d ch=%d rate=%g -> %d%s%s", slot, frames, ch, rate, ok,
                  odd.empty() ? "" : " extreme samples:", odd.c_str());
            break;
        }
        case 2:
        {
            const int count = r.range(0, 70);
            std::vector<float> m(static_cast<size_t>(std::max(count, 1)));
            for (auto& x : m)
                x = r.chance(0.05) ? extremeFloat(r) : r.uniform(-0.2f, 1.2f);
            const int ok = crucible_set_sample_slices(run.e, slot, m.data(), count);
            std::vector<float> back(8);
            crucible_get_sample_slices(run.e, slot, back.data(), 8);
            trace("set_sample_slices slot=%d count=%d -> %d", slot, count, ok);
            if (r.chance(0.5))
                crucible_slice_sample_equally(run.e, slot, r.range(-1, 200));
            else
                crucible_slice_sample_transients(run.e, slot, r.range(-1, 200), r.chance(0.1) ? extremeFloat(r) : r.uniform(0.0f, 1.0f));
            break;
        }
        case 3:
        {
            const int n = r.range(0, 70);
            std::vector<CrucibleZone> z(static_cast<size_t>(std::max(n, 1)));
            for (auto& q : z)
            {
                q.sample_id = r.range(-1, 3);
                q.key_lo = r.range(-10, 140);
                q.key_hi = r.range(-10, 140);
                q.vel_lo = r.range(-10, 140);
                q.vel_hi = r.range(-10, 140);
                q.root = r.chance(0.05) ? extremeFloat(r) : r.uniform(0.0f, 127.0f);
                q.gain_db = r.chance(0.05) ? extremeFloat(r) : r.uniform(-60.0f, 24.0f);
                q.tune_cents = r.chance(0.05) ? extremeFloat(r) : r.uniform(-100.0f, 100.0f);
                q.loop_start = r.chance(0.05) ? extremeFloat(r) : r.uniform(-0.1f, 1.1f);
                q.loop_end = r.chance(0.05) ? extremeFloat(r) : r.uniform(-0.1f, 1.1f);
                q.loop_mode = r.range(-2, 4);
                q.group = r.range(-1, 10);
            }
            if (r.chance(0.3))
            {
                std::vector<float> data(static_cast<size_t>(r.range(1, 4000)));
                for (auto& x : data)
                    x = r.uniform(-1.0f, 1.0f);
                crucible_add_zone_sample(run.e, slot, data.data(), static_cast<int>(data.size()), 1, 44100.0, "z");
            }
            const int ok = crucible_set_zones(run.e, slot, z.data(), n);
            std::vector<CrucibleZone> back(4);
            crucible_get_zones(run.e, slot, back.data(), 4);
            crucible_zone_sample_count(run.e, slot);
            trace("set_zones slot=%d n=%d -> %d", slot, n, ok);
            if (r.chance(0.1))
                crucible_clear_zones(run.e, slot);
            break;
        }
        case 4:
        {
            trace("clear_sample slot=%d", slot);
            crucible_clear_sample(run.e, slot);
            break;
        }
        default:
        {
            static const char* clouds[] = {"", "garbage", "cloud 1\n", "\n\n\n", "{}", "points 3\n0 0 0\n1 1 1\nnan nan nan\n"};
            const int ok = crucible_set_cloud(run.e, slot, clouds[r.below(6)]);
            trace("set_cloud slot=%d -> %d", slot, ok);
            break;
        }
    }
    crucible_service(run.e);
}

void destroyEngine(Run& run)
{
    if (run.e != nullptr)
        crucible_destroy(run.e);
    run.e = nullptr;
    run.routes.clear();
    run.subs.clear();
}

// Returns false when the seed's product was (validly) refused and the run has nothing to drive.
bool runSeed(uint64_t seed)
{
    gSeed = seed;
    gStep = -1;
    gStepName = "setup";
    Run run;
    Rng setup(stepSeed(seed, kSetupStream));
    if (!createEngine(run, setup))
        return false;
    prepare(run, setup);

    for (int step = 0; step < gOpt.steps; ++step)
    {
        Rng r(stepSeed(seed, static_cast<uint64_t>(step)));
        uint32_t pick = r.below(1000);
        if (gOpt.hwPreset && pick >= 735 && pick < 790) // no route edits, re-prepare, views or content on this path
            pick = 0;
        if (gOpt.hwPreset && pick >= 855 && pick < 940)
            pick = 0;
        if (gOpt.hwPreset && pick >= 985) // and no product changes
            pick = 0;
        if (gOpt.haveKeep && gOpt.keep.count(step) == 0)
            continue;
        gStep = step;
        if (gOpt.progress)
            std::fprintf(stderr, "FUZZ-STEP %d\n", step);
        if (gOpt.stepTimeout > 0)
            alarm(gOpt.stepTimeout);
        if (pick < 520)
        {
            gStepName = "process";
            processStep(run, r);
        }
        else if (pick < 640)
        {
            gStepName = "set_param";
            setParamStep(run, r);
        }
        else if (pick < 720)
        {
            gStepName = "notes";
            noteStep(run, r);
        }
        else if (pick < 735)
        {
            gStepName = "map_cc";
            mapCcStep(run, r);
        }
        else if (pick < 790)
        {
            gStepName = "routes";
            routeStep(run, r);
        }
        else if (pick < 830)
        {
            gStepName = "patch";
            patchStep(run, r);
        }
        else if (pick < 855)
        {
            gStepName = "reset";
            trace("reset");
            crucible_reset(run.e);
        }
        else if (pick < 870)
        {
            gStepName = "prepare";
            prepare(run, r);
        }
        else if (pick < 900)
        {
            gStepName = "views";
            viewStep(run, r);
        }
        else if (pick < 940)
        {
            gStepName = "query";
            queryStep(run, r);
        }
        else if (pick < 965)
        {
            gStepName = "content";
            contentStep(run, r);
        }
        else if (pick < 985)
        {
            gStepName = "service";
            trace("service");
            crucible_service(run.e);
        }
        else
        {
            // A fresh engine (destroy, then a new product from this step's generator), as a host reloading the plug-in.
            gStepName = "recreate";
            destroyEngine(run);
            if (!createEngine(run, r))
            {
                Rng fallback(stepSeed(seed, kSetupStream));
                if (!createEngine(run, fallback))
                    fail("create", "fallback product refused");
            }
            prepare(run, r);
        }
    }
    alarm(0);
    gStep = gOpt.steps;
    gStepName = "teardown";
    destroyEngine(run);
    if (run.bus != nullptr)
        crucible_keybus_close(run.bus);
    if (gOpt.stats)
        std::printf("seed %" PRIu64 " product=%s blocks=%ld samples=%ld peak=%.3f roundtrips=%ld frames_drift=%ld\n", seed,
                    run.productLabel.c_str(), run.blocks, run.samples, static_cast<double>(run.peak), run.roundTrips,
                    run.framesDrift);
    return true;
}

// --self-test: shows that the allocation counter sees both C and C++ allocations on this build (it exits 0 when both
// are counted, 1 otherwise).
int selfTest()
{
    gTrack = true;
    gAllocs = 0;
    // The fences stop the compiler from reading the counter before the call (it may assume malloc touches no globals).
    void* volatile c = std::malloc(64);
    std::atomic_signal_fence(std::memory_order_seq_cst);
    const long afterMalloc = gAllocs;
    auto* volatile cpp = new int[16];
    std::atomic_signal_fence(std::memory_order_seq_cst);
    const long afterNew = gAllocs;
    gTrack = false;
    std::free(c);
    delete[] cpp;
    std::printf("self-test: malloc counted %ld, new[] counted %ld\n", afterMalloc, afterNew - afterMalloc);
#if CRUCIBLE_FUZZ_ASAN
    return afterMalloc == 1 && afterNew == 2 ? 0 : 1;
#else
    return afterNew - afterMalloc == 1 ? 0 : 1; // without AddressSanitizer only operator new is counted
#endif
}

void usage()
{
    std::fprintf(stderr, "usage: crucible-fuzz --root <repo> [--seed S | --seeds A:B] [--steps N] [--keep i,j,...] [--trace]\n"
                         "                     [--bound X] [--seconds T] [--stats] [--progress] [--no-alloc-stacks]\n"
                         "                     [--strict-frames] [--strict-meta] [--step-timeout S]\n"
                         "                     [--avoid-known] [--product FILE] [--hardware-preset]\n");
    std::exit(2);
}
} // namespace

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        auto next = [&]() -> std::string
        {
            if (i + 1 >= argc)
                usage();
            return argv[++i];
        };
        if (a == "--root")
            gOpt.root = next();
        else if (a == "--seed")
            gOpt.seedFirst = gOpt.seedLast = std::strtoull(next().c_str(), nullptr, 10);
        else if (a == "--seeds")
        {
            const std::string s = next();
            const auto colon = s.find(':');
            if (colon == std::string::npos)
                usage();
            gOpt.seedFirst = std::strtoull(s.substr(0, colon).c_str(), nullptr, 10);
            gOpt.seedLast = std::strtoull(s.substr(colon + 1).c_str(), nullptr, 10);
        }
        else if (a == "--steps")
            gOpt.steps = std::atoi(next().c_str());
        else if (a == "--keep")
        {
            gOpt.haveKeep = true;
            std::stringstream ss(next());
            std::string tok;
            while (std::getline(ss, tok, ','))
                if (!tok.empty())
                    gOpt.keep.insert(std::atoi(tok.c_str()));
        }
        else if (a == "--trace")
            gOpt.trace = true;
        else if (a == "--step-timeout")
            gOpt.stepTimeout = static_cast<unsigned>(std::atoi(next().c_str()));
        else if (a == "--strict-meta")
            gOpt.strictMeta = true;
        else if (a == "--product")
            gOpt.product = next();
        else if (a == "--hardware-preset")
            gOpt.hwPreset = true;
        else if (a == "--avoid-known")
            gOpt.avoidKnown = true;
        else if (a == "--strict-frames")
            gOpt.strictFrames = true;
        else if (a == "--progress")
            gOpt.progress = true;
        else if (a == "--stats")
            gOpt.stats = true;
        else if (a == "--bound")
            gOpt.bound = std::strtof(next().c_str(), nullptr);
        else if (a == "--seconds")
            gOpt.seconds = std::strtod(next().c_str(), nullptr);
        else if (a == "--self-test")
        {
#if CRUCIBLE_FUZZ_ASAN
            gPrintAllocStacks = false;
            __sanitizer_install_malloc_and_free_hooks(fuzzMallocHook, fuzzFreeHook);
#endif
            return selfTest();
        }
        else if (a == "--no-alloc-stacks")
            gPrintAllocStacks = false;
        else
            usage();
    }
#if CRUCIBLE_FUZZ_ASAN
    __sanitizer_install_malloc_and_free_hooks(fuzzMallocHook, fuzzFreeHook);
    __sanitizer_set_death_callback(onSanitizerDeath);
#endif
    std::signal(SIGALRM, onAlarm);
    loadProductFiles();
    const auto start = std::chrono::steady_clock::now();
    long ran = 0, refused = 0;
    for (uint64_t s = gOpt.seedFirst; s <= gOpt.seedLast; ++s)
    {
        if (gOpt.seconds > 0.0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > gOpt.seconds)
            break;
        if (runSeed(s))
            ++ran;
        else
            ++refused;
        std::fflush(stdout);
    }
    std::printf("crucible-fuzz: %ld seeds run, %ld generated products refused (valid refusals), no failures\n", ran,
                refused);
    return 0;
}
