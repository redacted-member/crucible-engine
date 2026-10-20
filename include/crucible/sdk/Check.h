// SPDX-License-Identifier: MIT
#pragma once

// The real-time harness behind tools/crucible-module-check: runs a module through the contract of the module ABI and
// reports a pass, fail or skip per check.
//
//   metadata      parameter and port tables are consistent (ranges, defaults, choices, unique ids)
//   allocation    no allocation while the module renders, plays notes and has its parameters changed every block
//   extremes      the output stays finite and below Options::maxPeak at the minimum, the maximum and seeded corners of
//                 the parameter ranges, for a loud noise input and for silence
//   finite        no NaN or infinity out of any render, also for denormal input and a hot (+18 dBFS) input
//   denormals     the output does not depend on flush-to-zero (compared against a flushed render) and a decaying tail
//                 does not slow the module down by orders of magnitude
//   determinism   two fresh instances with the same seed render identical bits, and so does one instance after reset()
//   cpu           one second of audio costs less than the budget of the module's CPU class
//
// The allocation check needs a counter of operator new: the program that runs the harness replaces operator new and
// passes the hooks in Options::probe (the tool and the tests do). Without it the check is skipped.

#include <crucible/graph/Node.h>
#include <crucible/module/Module.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace crucible::sdk::check
{

struct AllocationProbe
{
    std::function<void(bool)> track; // counting on / off for the calling thread; switching on resets the count
    std::function<long()> count;     // allocations counted since it was switched on
    bool available() const { return static_cast<bool>(track) && static_cast<bool>(count); }
};

struct Options
{
    double sampleRate = 48000.0;
    int blockSize = 128;
    std::uint32_t seed = 1;
    double seconds = 1.0;   // length of each render of the sweeps
    double maxPeak = 100.0; // bound of the extremes check (+40 dBFS)
    int corners = 12;       // seeded random corners of the parameter space
    bool enforceCpu = true; // false: the CPU load is reported but never fails
    double cpuLight = 0.04; // budgets, as a fraction of real time on one core
    double cpuMedium = 0.10;
    double cpuHeavy = 0.40;
    AllocationProbe probe;
};

enum class Status
{
    Pass,
    Fail,
    Skip
};

struct CheckResult
{
    std::string name;
    Status status = Status::Pass;
    std::string detail;
};

struct Report
{
    std::string id;
    std::string kind;
    std::vector<CheckResult> checks;
    double cpuLoad = 0.0; // fraction of real time, 0 when not measured
    bool passed() const;
    int failures() const;
};

// Something the harness can drive block by block. Parameter values come in natural units, one per Subject::params.
class Runner
{
public:
    virtual ~Runner() = default;
    // Control thread. False refuses the configuration.
    virtual bool prepare(double sampleRate, int maxBlock, std::uint32_t seed) = 0;
    virtual void reset() = 0;
    virtual void noteOn(float note, float velocity, std::uint32_t seed)
    {
        (void)note;
        (void)velocity;
        (void)seed;
    }
    virtual void noteOff() {}
    // Renders n samples. The input is ignored by sources; effects and filters read it. outL / outR are written.
    virtual void process(const float* inL, const float* inR, float* outL, float* outR, int n, const float* values) = 0;
};

struct Subject
{
    std::string id;
    std::string kind; // moduleKindName
    CpuClass cpu = CpuClass::Light;
    std::vector<ParamDef> params; // all parameters, in the order process() gets the values
    std::vector<char> frozen;     // per parameter: not swept (chain position and the like); empty = none
    int onParam = -1;             // index of a bypass switch that is held on during the checks, or -1
    bool voiced = false;          // the module plays notes (oscillators, nodes)
    std::vector<PortDef> inputs, outputs;
    // Creates a runner; empty for modules that process nothing by themselves (the voice, performance controls).
    std::function<std::unique_ptr<Runner>()> make;
};

// Every built-in module of the neutral product (effects, oscillators, node types); modules without a processor get the
// metadata check only.
std::vector<Subject> builtInSubjects();
// The modules the SDK added to a node registry (effects and node types that are not built in).
std::vector<Subject> sdkSubjects(const graph::NodeRegistry& nodes);

Report run(const Subject& subject, const Options& options);

} // namespace crucible::sdk::check
