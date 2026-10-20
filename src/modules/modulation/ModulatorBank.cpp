// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/Modulators.h>
#include <crucible/module/ParamBuilder.h>

#include <cmath>
#include <cstdio>
#include <string>

// The modulator bank (P12, docs/plans/ENGINE-COMMUNITY-PLAN.md section 2.10) as parameters: eight modulators `bankN_*`,
// route sources kSourceModulator + n. Registered after everything else so that the indices (and the generic destination
// numbers) of the existing parameters do not move. Every modulator defaults to Off; a bank with no active modulator costs
// nothing.

namespace crucible::modules
{

namespace
{
std::vector<std::string> inputChoices(int index)
{
    auto names = modSourceNamesFor(kNumMacros);
    for (int n = 0; n < index; ++n)
        names.push_back("Modulator " + std::to_string(n + 1));
    return names;
}
} // namespace

ModuleDescriptor modulatorBank(const VoiceGraphShape&)
{
    ModuleDescriptor d;
    d.id = "mod.bank";
    d.name = "Modulator Bank";
    d.kind = ModuleKind::Voice;
    d.category = "Modulation";
    d.cpu = CpuClass::Light;
    d.sectionHint = "mod";
    d.description =
        "Eight modulators evaluated in index order, each reading ModSources or earlier modulators: combine, remap "
        "curve, lag, quantise, sample and hold, Turing, binary, chaos, note gate, follower, pitch tracker and LFO "
        "table. Voice, layer or global scope. Route sources 4000 to 4007.";
    std::vector<std::string> types;
    for (int t = 0; t < static_cast<int>(ModulatorType::Count); ++t)
        types.emplace_back(modulatorTypeName(static_cast<ModulatorType>(t)));
    for (int n = 1; n <= kNumBankModulators; ++n)
    {
        const std::string p = "bank" + std::to_string(n);
        const std::string label = "Modulator " + std::to_string(n);
        const std::string main = p + "_main", timing = p + "_timing";
        d.pages.push_back(
            {main, label, "mod", label,
             "Type, scope and the two inputs (only earlier modulators are offered). The mode picks the variant: "
             "Combine add / multiply / min / max / crossfade / difference; Quantise nearest / down / up; "
             "S&H, Turing and Binary clock from rate / input B / note-on; Chaos logistic / Lorenz; Note Gate "
             "gate / velocity / trigger; Follower audio / input A. Amount is the crossfade mix, the Turing "
             "randomness, the Binary probability, the logistic r or the LFO table position."});
        d.pages.push_back(
            {timing, label + " Timing", "mod", label + " Timing",
             "Clock or LFO rate, rise and fall times, steps (Quantise) or length (Turing), and the random seed."});
        d.params.push_back(params::choice(p + "_type", label + " Type", main, types, 0));
        d.params.push_back(params::choice(p + "_scope", label + " Scope", main, {"Voice", "Layer", "Global"}, 0));
        d.params.push_back(params::choice(p + "_in_a", label + " Input A", main, inputChoices(n - 1), 0));
        d.params.push_back(params::choice(p + "_in_b", label + " Input B", main, inputChoices(n - 1), 0));
        d.params.push_back(params::integer(p + "_mode", label + " Mode", main, 0, 5, 0));
        d.params.push_back(
            params::floating(p + "_amount", label + " Amount", main, 0.0f, 1.0f, 0.5f, {}, 1.0f, Smoothing::None));
        d.params.push_back(
            params::floating(p + "_rate", label + " Rate", timing, 0.01f, 2000.0f, 4.0f, "Hz", 0.3f, Smoothing::None));
        d.params.push_back(
            params::floating(p + "_rise", label + " Rise", timing, 0.0f, 10.0f, 0.0f, "s", 0.4f, Smoothing::None));
        d.params.push_back(
            params::floating(p + "_fall", label + " Fall", timing, 0.0f, 10.0f, 0.0f, "s", 0.4f, Smoothing::None));
        d.params.push_back(params::integer(p + "_steps", label + " Steps", timing, 2, 64, 8));
        d.params.push_back(params::integer(p + "_seed", label + " Seed", timing, 1, 9999, 1));
        for (int page = 0; page < 3; ++page)
        {
            const std::string curve = p + "_curve" + std::to_string(page + 1);
            d.pages.push_back({curve, label + " Curve " + std::to_string(page + 1), "mod", label + " Remap Curve",
                               "The 16 points of the Remap curve, spread evenly over the input range (-1 to 1 for a bipolar "
                               "input, 0 to 1 for a unipolar one); linear between points."});
            for (int k = page * 6; k < std::min(page * 6 + 6, 16); ++k)
            {
                char id[16];
                std::snprintf(id, sizeof id, "_pt%02d", k + 1);
                d.params.push_back(params::floating(p + id, label + " Point " + std::to_string(k + 1), curve, -1.0f, 1.0f,
                                                    std::round(static_cast<float>(k) * 1000.0f / 15.0f) / 1000.0f, {}, 1.0f,
                                                    Smoothing::None));
            }
        }
    }
    return d;
}

} // namespace crucible::modules
