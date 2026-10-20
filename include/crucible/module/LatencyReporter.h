// SPDX-License-Identifier: MIT
#pragma once

// Capability interface for effects that delay their output (oversampling filters, lookahead, linear phase, block
// convolution) or ring on after the input stops (docs/plans/ENGINE-FX-PLAN.md, section 4.2). A module instance
// implements it next to EffectModule; hosts find it with ModuleRack::findCapability<LatencyReporter>() and
// compensate. Until the graph executor (P4) sums latencies along every path, this is the only way a module reports
// them.
//
// Latency and tail change only on the control thread (prepare, quality change, lookahead or phase-mode parameter).
// The change is announced through takeLatencyChanged(), which hosts poll after a parameter update.

namespace crucible
{

class EffectInputs;

class LatencyReporter
{
public:
    virtual ~LatencyReporter() = default;

    // Whole samples between an input sample and the output sample that carries it, at the current settings.
    virtual int latencySamples() const { return 0; }
    // Time the output keeps ringing after the input goes silent (reverb, delay feedback), in seconds.
    virtual double tailSeconds() const { return 0.0; }
    // True for freeze or infinite feedback: the tail never ends on its own.
    virtual bool tailInfinite() const { return false; }
    // Control thread, no audio running (Engine::prepare, after the modules were prepared): take the latency-relevant
    // parameters from `inputs` so latencySamples() is right before the first block. Default: nothing to adopt.
    virtual void adoptLatencyParams(const EffectInputs& /*inputs*/) {}
    // True once after the latency changed since the previous call (control thread).
    virtual bool takeLatencyChanged() { return false; }
};

} // namespace crucible
