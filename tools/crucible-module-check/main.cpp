// SPDX-License-Identifier: MIT
// crucible-module-check: runs the real-time harness over modules and reports whether they keep the contract of the
// module ABI (crucible_module.h): no allocation on the audio thread, bounded and finite output at parameter extremes,
// denormal safety, determinism per seed, CPU within the budget of the module's class.
//
//   crucible-module-check [options] <library>...     check every module of the shared libraries
//   crucible-module-check --builtin [options]        check every built-in module
//
// Options: --module ID (repeatable), --seed N, --seconds S, --block N, --rate HZ, --no-cpu-limit, --list, --quiet.
// Exit status: 0 all checks passed, 1 a check failed, 2 a library could not be loaded (including an ABI version this
// engine does not speak) or the command line is wrong.

#include <crucible/Version.h>
#include <crucible/sdk/Check.h>
#include <crucible/sdk/Host.h>
#include "../WinArgv.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <vector>

// The static AddressSanitizer / ThreadSanitizer runtimes on Linux define operator new / delete themselves; replacing
// them here would be a duplicate symbol, so the counter stays at zero in those builds.
#if defined(__linux__) && (defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__))
#define CRUCIBLE_CHECK_COUNT_ALLOCATIONS 0
#elif defined(__linux__) && defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define CRUCIBLE_CHECK_COUNT_ALLOCATIONS 0
#endif
#endif
#ifndef CRUCIBLE_CHECK_COUNT_ALLOCATIONS
#define CRUCIBLE_CHECK_COUNT_ALLOCATIONS 1
#endif

// Counts the allocations of the calling thread while tracking is on: the harness switches it on around the audio calls.
namespace
{
thread_local bool gTrack = false;
thread_local long gCount = 0;
} // namespace

#if CRUCIBLE_CHECK_COUNT_ALLOCATIONS
void* operator new(std::size_t n)
{
    if (gTrack)
        ++gCount;
    if (void* p = std::malloc(n == 0 ? 1 : n))
        return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n)
{
    if (gTrack)
        ++gCount;
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
using namespace crucible::sdk;

void usage()
{
    std::printf("usage: crucible-module-check [options] <library>...\n"
                "       crucible-module-check --builtin [options]\n"
                "options:\n"
                "  --builtin        check the built-in modules\n"
                "  --module ID      check only this module (repeatable)\n"
                "  --seed N         seed of the signals and the module instances (default 1)\n"
                "  --seconds S      length of each sweep render (default 1)\n"
                "  --block N        block size (default 128)\n"
                "  --rate HZ        sample rate (default 48000)\n"
                "  --no-cpu-limit   report the CPU load without failing on it\n"
                "  --list           list the modules and exit\n"
                "  --quiet          print failures and the summary only\n"
                "  --version\n");
}

bool wanted(const std::vector<std::string>& only, const std::string& id)
{
    if (only.empty())
        return true;
    for (const auto& o : only)
        if (o == id)
            return true;
    return false;
}

const char* label(check::Status s)
{
    return s == check::Status::Pass ? "pass" : (s == check::Status::Fail ? "FAIL" : "skip");
}
} // namespace

int main(int argc, char** argv)
{
    const crucible::tools::Utf8Args utf8Args(argc, argv);
    argc = utf8Args.argc;
    argv = utf8Args.argv;
    check::Options options;
    options.probe.track = [](bool on)
    {
        gTrack = on;
        if (on)
            gCount = 0;
    };
    options.probe.count = [] { return gCount; };
    bool builtin = false, list = false, quiet = false;
    std::vector<std::string> libraries, only;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        const auto value = [&](const char* name) -> const char*
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "crucible-module-check: %s needs a value\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--builtin")
            builtin = true;
        else if (a == "--module")
            only.emplace_back(value("--module"));
        else if (a == "--seed")
            options.seed = static_cast<std::uint32_t>(std::strtoul(value("--seed"), nullptr, 10));
        else if (a == "--seconds")
            options.seconds = std::atof(value("--seconds"));
        else if (a == "--block")
            options.blockSize = std::atoi(value("--block"));
        else if (a == "--rate")
            options.sampleRate = std::atof(value("--rate"));
        else if (a == "--no-cpu-limit")
            options.enforceCpu = false;
        else if (a == "--list")
            list = true;
        else if (a == "--quiet")
            quiet = true;
        else if (a == "--version")
        {
            std::printf("crucible-module-check %s (module ABI %u)\n", CRUCIBLE_VERSION_STRING, CRUCIBLE_MODULE_ABI_VERSION);
            return 0;
        }
        else if (a == "--help" || a == "-h")
        {
            usage();
            return 0;
        }
        else if (!a.empty() && a[0] == '-')
        {
            std::fprintf(stderr, "crucible-module-check: unknown option %s\n", a.c_str());
            usage();
            return 2;
        }
        else
            libraries.push_back(a);
    }
    if (!builtin && libraries.empty())
    {
        usage();
        return 2;
    }
    // CRUCIBLE_SKIP_CPU_TESTS (see CONTRIBUTING.md): on a shared machine the timing-based checks are reported, not enforced.
    if (const char* skip = std::getenv("CRUCIBLE_SKIP_CPU_TESTS");
        skip != nullptr && skip[0] != '\0' && std::strcmp(skip, "0") != 0)
    {
        options.enforceCpu = false;
        options.enforceDenormalTime = false;
    }
    if (options.seconds <= 0.0 || options.blockSize < 1 || options.sampleRate < 8000.0)
    {
        std::fprintf(stderr, "crucible-module-check: seconds, block and rate must be positive (rate at least 8000)\n");
        return 2;
    }

    int exitCode = 0;
    std::vector<check::Subject> subjects;
    if (builtin)
        subjects = check::builtInSubjects();

    // The libraries stay loaded until the checks are done: the set owns them.
    ModuleSet modules;
    modules.allowLibraries(true);
    for (const auto& path : libraries)
    {
        const int r = modules.loadLibrary(path);
        if (r != CRUCIBLE_MODULE_OK)
        {
            std::fprintf(stderr, "error: %s\n", modules.error().c_str());
            exitCode = 2;
        }
    }
    if (modules.count() > 0)
    {
        crucible::graph::NodeRegistry nodes;
        modules.addTo(nodes);
        for (auto& s : check::sdkSubjects(nodes))
            subjects.push_back(std::move(s));
    }

    int checked = 0, failedModules = 0, failedChecks = 0;
    for (const auto& s : subjects)
    {
        if (!wanted(only, s.id))
            continue;
        if (list)
        {
            std::printf("%s (%s)\n", s.id.c_str(), s.kind.c_str());
            continue;
        }
        const check::Report r = check::run(s, options);
        ++checked;
        failedChecks += r.failures();
        failedModules += r.passed() ? 0 : 1;
        if (quiet && r.passed())
            continue;
        std::printf("%s (%s)\n", r.id.c_str(), r.kind.c_str());
        for (const auto& c : r.checks)
            if (!quiet || c.status == check::Status::Fail)
                std::printf("  %-4s %-12s %s\n", label(c.status), c.name.c_str(), c.detail.c_str());
    }
    if (list)
        return exitCode;
    if (checked == 0 && exitCode == 0)
    {
        std::fprintf(stderr, "crucible-module-check: no module matched\n");
        return 2;
    }
    std::printf("\n%d module(s) checked, %d failed (%d check(s))\n", checked, failedModules, failedChecks);
    if (failedModules > 0 && exitCode == 0)
        exitCode = 1;
    return exitCode;
}
