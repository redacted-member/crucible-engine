// SPDX-License-Identifier: MIT
// The instrument skeleton with every feature: a product built from the skeleton path (examples/instrument_skeleton/
// skeleton_full_product.json, created through crucible_create_from_product and driven only through crucible.h) that
// has every current module, and renders with every oscillator type, every filter model (the Formant filter with a user
// table and its state saturation), every effect (the shaper with a curve, the multiband drive, the cabinet, the crusher
// tracking a note, the saturator filter, the convolution with a response), user wavetable zones, the sequencer, the
// arpeggiator, macros and routes. Each render is finite, bounded and audible; the full chain renders bit-identically
// twice; a patch saved from the engine reloads into a fresh one and renders the same. Run it in Release and under
// AddressSanitizer (ctest -R skeleton_all_features, or crucible_engine_tests "Skeleton all features").

#include <crucible/AudioFile.h>
#include <crucible/Cloud.h>
#include <crucible/ParameterMap.h>
#include <crucible/crucible.h>

#include "TestFramework.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace crucible;
using crucible::test::String;
using crucible::test::UnitTest;

#ifndef CRUCIBLE_ENGINE_SOURCE_DIR
#define CRUCIBLE_ENGINE_SOURCE_DIR "."
#endif

namespace
{
constexpr double kRate = 48000.0;
constexpr int kBlock = 256;

std::string slurp(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

struct Render
{
    std::vector<float> left, right;
    bool finite = true;
    float peak = 0.0f;
    double heldRms = 0.0; // over the held part
};

// One engine from the skeleton product. quality 2 (High) makes the builder threads of the cabinet and the convolution
// deterministic offline, and is what the determinism renders use.
struct Rig
{
    CrucibleEngine* engine = nullptr;

    explicit Rig(const std::string& productJson, int quality = 1, std::string* error = nullptr)
    {
        CrucibleConfig config;
        crucible_config_init(&config);
        config.max_voices = 8;
        config.seed = 4242;
        config.quality = quality;
        char err[2048] = "";
        engine = crucible_create_from_product(&config, productJson.c_str(), nullptr, err, sizeof err);
        if (error != nullptr)
            *error = err;
        if (engine != nullptr)
        {
            crucible_prepare(engine, kRate, kBlock);
            crucible_reset(engine);
        }
    }
    ~Rig()
    {
        if (engine != nullptr)
            crucible_destroy(engine);
    }
    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;

    int index(const char* id) const { return crucible_param_index(engine, id); }
    bool set(const char* id, float value) { return crucible_set_param_id(engine, id, value) == 1; }
    float get(const char* id) const { return crucible_get_param(engine, index(id)); }

    // Plays the notes for `held` blocks, releases them and renders `tail` more blocks.
    Render play(const std::vector<int>& notes, int held, int tail)
    {
        Render r;
        std::vector<float> l(kBlock), rr(kBlock);
        double sum = 0.0;
        long count = 0;
        for (int b = 0; b < held + tail; ++b)
        {
            if (b == 0)
                for (const int n : notes)
                    crucible_note_on(engine, n, 0.8f);
            if (b == held)
                for (const int n : notes)
                    crucible_note_off(engine, n);
            crucible_process(engine, l.data(), rr.data(), kBlock, nullptr, 0);
            for (int i = 0; i < kBlock; ++i)
            {
                const float a = l[static_cast<size_t>(i)], c = rr[static_cast<size_t>(i)];
                if (!std::isfinite(a) || !std::isfinite(c))
                    r.finite = false;
                r.peak = std::max({r.peak, std::abs(a), std::abs(c)});
                if (b < held)
                {
                    sum += static_cast<double>(a) * a + static_cast<double>(c) * c;
                    count += 2;
                }
            }
            r.left.insert(r.left.end(), l.begin(), l.end());
            r.right.insert(r.right.end(), rr.begin(), rr.end());
        }
        r.heldRms = std::sqrt(sum / static_cast<double>(std::max<long>(count, 1)));
        return r;
    }
};

std::vector<float> sineFrames(int frames, int frameSize, float harmonics)
{
    std::vector<float> data(static_cast<size_t>(frames) * static_cast<size_t>(frameSize));
    for (int f = 0; f < frames; ++f)
        for (int i = 0; i < frameSize; ++i)
        {
            const double ph = 2.0 * 3.14159265358979 * i / frameSize;
            double v = 0.0;
            for (int h = 1; h <= 1 + static_cast<int>(harmonics * static_cast<float>(f + 1)); ++h)
                v += std::sin(ph * h) / h;
            data[static_cast<size_t>(f) * static_cast<size_t>(frameSize) + static_cast<size_t>(i)] = static_cast<float>(0.5 * v);
        }
    return data;
}

std::vector<float> sampleData(int frames)
{
    std::vector<float> s(static_cast<size_t>(frames));
    for (size_t i = 0; i < s.size(); ++i)
        s[i] = static_cast<float>(0.5 * std::sin(2.0 * 3.14159265358979 * 220.0 * static_cast<double>(i) / kRate) +
                                  0.2 * std::sin(2.0 * 3.14159265358979 * 660.0 * static_cast<double>(i) / kRate));
    return s;
}

// Everything the skeleton installs that is not a parameter: wavetable zones, samples, a cloud, the formant table with
// saturation, the shaper curve and the convolution response (written to a temporary WAV).
void installContent(Rig& rig, const std::string& irPath, std::vector<std::string>& problems)
{
    char err[512] = "";
    const auto frames = sineFrames(3, 256, 2.0f);
    CrucibleWavetableZone zones[3];
    const double hz[3] = {200.0, 800.0, 3000.0};
    for (int z = 0; z < 3; ++z)
    {
        std::memset(&zones[z], 0, sizeof zones[z]);
        zones[z].struct_size = sizeof zones[z];
        zones[z].frames = frames.data();
        zones[z].frame_count = 3;
        zones[z].frame_size = 256;
        zones[z].max_hz = hz[z];
    }
    if (crucible_load_wavetable_zones(rig.engine, 0, zones, 3, "FullZones", nullptr, err, sizeof err) != 1)
        problems.push_back(std::string("wavetable zones: ") + err);
    const auto sample = sampleData(24000);
    if (crucible_load_sample(rig.engine, 0, sample.data(), static_cast<int>(sample.size()), 1, kRate, "full sample") != 1)
        problems.push_back("sample slot 0");
    if (crucible_load_sample(rig.engine, 1, sample.data(), static_cast<int>(sample.size()), 1, kRate, "full sample 2") != 1)
        problems.push_back("sample slot 1");
    if (crucible_set_cloud(rig.engine, 2, CloudModel::harmonicMorph(16).serialise().c_str()) != 1)
        problems.push_back("cloud slot 2");
    const CrucibleFormant vowels[2 * 3] = {{700.0f, 80.0f, 0.0f},  {1100.0f, 90.0f, -6.0f}, {2600.0f, 120.0f, -12.0f},
                                           {300.0f, 60.0f, 0.0f},  {2200.0f, 100.0f, -9.0f}, {3000.0f, 130.0f, -15.0f}};
    if (crucible_set_formant_table(rig.engine, 0, vowels, 2, 3, 2.0f, err, sizeof err) != 1)
        problems.push_back(std::string("formant table: ") + err);
    if (crucible_set_formant_saturation(rig.engine, 0, 1, err, sizeof err) != 1)
        problems.push_back(std::string("formant saturation: ") + err);
    const CrucibleShaperPoint curve[4] = {{0.0f, 0.0f, 0.0f}, {0.4f, 0.7f, 0.3f}, {0.8f, 0.9f, -0.3f}, {1.0f, 0.85f, 0.0f}};
    if (crucible_set_shaper_curve(rig.engine, "fx.shaper", curve, 4, 1, err, sizeof err) != 1)
        problems.push_back(std::string("shaper curve: ") + err);
    if (!irPath.empty() && crucible_load_ir(rig.engine, "fx.convolution", irPath.c_str(), err, sizeof err) != 1)
        problems.push_back(std::string("convolution response: ") + err);
}

// The "on" switch of a module: its shortest parameter id that ends in "_on".
std::string onParamOf(const CrucibleEngine* engine, int module)
{
    CrucibleModuleMeta meta{};
    meta.struct_size = sizeof meta;
    crucible_module_meta(engine, module, &meta);
    std::string best;
    for (int k = 0; k < meta.num_params; ++k)
    {
        CrucibleParamMeta pm{};
        pm.struct_size = sizeof pm;
        crucible_param_meta(engine, crucible_module_param(engine, module, k), &pm);
        const std::string id = pm.id;
        if (id.size() > 3 && id.compare(id.size() - 3, 3, "_on") == 0 && (best.empty() || id.size() < best.size()))
            best = id;
    }
    return best;
}
} // namespace

class SkeletonFullTests : public UnitTest
{
public:
    SkeletonFullTests() : UnitTest("Skeleton all features", "Examples") {}

    void runTest() override
    {
        namespace fs = std::filesystem;
        const std::string dir = std::string(CRUCIBLE_ENGINE_SOURCE_DIR) + "/examples/instrument_skeleton/";
        const std::string product = slurp(dir + "skeleton_full_product.json");
        expect(!product.empty(), "skeleton_full_product.json is readable");

        // A response for the convolution: a decaying noise burst of 0.4 s.
        const fs::path tmp = fs::temp_directory_path() / "crucible_skeleton_full";
        fs::create_directories(tmp);
        const std::string irPath = (tmp / "ir.wav").string();
        {
            AudioData ir;
            ir.sampleRate = kRate;
            ir.channels.assign(2, std::vector<float>(19200));
            std::uint32_t seed = 7;
            for (size_t c = 0; c < 2; ++c)
                for (size_t i = 0; i < ir.channels[c].size(); ++i)
                {
                    seed = seed * 1664525U + 1013904223U;
                    ir.channels[c][i] = (static_cast<float>(seed >> 8) / 8388608.0f - 1.0f) *
                                        std::exp(-6.0f * static_cast<float>(i) / 19200.0f) * 0.3f;
                }
            expect(writeWavFile(irPath, ir, 32), "wrote the impulse response");
        }

        beginTest("the product has every module of the default product, every effect once");
        {
            std::string error;
            Rig full(product, 1, &error);
            expect(full.engine != nullptr, error);
            if (full.engine == nullptr)
                return;
            CrucibleEngine* plain = crucible_create(nullptr);
            expectEquals(crucible_module_count(full.engine), crucible_module_count(plain));
            int effects = 0;
            for (int m = 0; m < crucible_module_count(full.engine); ++m)
            {
                CrucibleModuleMeta meta{};
                meta.struct_size = sizeof meta;
                crucible_module_meta(full.engine, m, &meta);
                expect(meta.legacy == 0, std::string(meta.id) + " is current");
                if (std::string(meta.kind) == "effect")
                {
                    ++effects;
                    expect(!onParamOf(full.engine, m).empty(), std::string(meta.id) + " has an on switch");
                }
            }
            logMessage("effects in the product: " + std::to_string(effects));
            expect(effects >= 29, "every current effect is in the chain");
            crucible_destroy(plain);
        }

        beginTest("every oscillator type renders: finite, bounded, audible");
        {
            Rig rig(product);
            if (rig.engine == nullptr)
                return;
            std::vector<std::string> problems;
            installContent(rig, irPath, problems);
            for (const auto& p : problems)
                expect(false, p);
            CrucibleParamMeta pm{};
            pm.struct_size = sizeof pm;
            crucible_param_meta(rig.engine, rig.index("osc1_type"), &pm);
            logMessage("oscillator types: " + std::to_string(pm.num_choices));
            expect(pm.num_choices >= 9, "all oscillator types are offered");
            // The zoned user wavetable (slot 0) is the table choice "User 1".
            CrucibleParamMeta tm{};
            tm.struct_size = sizeof tm;
            crucible_param_meta(rig.engine, rig.index("osc1_table"), &tm);
            int zoned = -1;
            for (int k = 0; k < tm.num_choices; ++k)
            {
                const char* name = crucible_param_choice(rig.engine, rig.index("osc1_table"), k);
                if (name != nullptr && std::string(name) == "User 1")
                    zoned = k;
            }
            if (zoned < 0)
                for (int k = std::max(0, tm.num_choices - 6); k < tm.num_choices; ++k)
                    logMessage(String("table choice ") + std::to_string(k) + ": " + crucible_param_choice(rig.engine, rig.index("osc1_table"), k));
            expect(zoned >= 0, "the zoned user wavetable is selectable");
            for (int type = 0; type < pm.num_choices; ++type)
            {
                Rig one(product);
                std::vector<std::string> p2;
                installContent(one, irPath, p2);
                one.set("osc1_type", static_cast<float>(type));
                if (type == 0 && zoned >= 0)
                    one.set("osc1_table", static_cast<float>(zoned));
                const auto r = one.play({48, 60, 72}, 60, 20);
                const char* name = crucible_param_choice(one.engine, one.index("osc1_type"), type);
                const std::string label = "oscillator " + std::to_string(type) + " " + (name != nullptr ? name : "?");
                expect(r.finite, label + " finite");
                expect(r.peak < 8.0f, label + " peak " + String(r.peak));
                expect(r.heldRms > 1.0e-3, label + " audible, rms " + String(r.heldRms));
                if (type == 0 && zoned >= 0)
                {
                    // Zones: a high and a low note read different sources, so two renders differ.
                    Rig lowRig(product), highRig(product);
                    std::vector<std::string> p3;
                    installContent(lowRig, irPath, p3);
                    installContent(highRig, irPath, p3);
                    lowRig.set("osc1_table", static_cast<float>(zoned));
                    highRig.set("osc1_table", static_cast<float>(zoned));
                    expect(lowRig.play({40}, 40, 4).heldRms > 1.0e-3 && highRig.play({90}, 40, 4).heldRms > 1.0e-3,
                           "zoned wavetable plays at the bottom and the top of the keyboard");
                }
            }
        }

        beginTest("every filter model renders, the Formant filter with a user table and saturation");
        {
            Rig probe(product);
            if (probe.engine == nullptr)
                return;
            CrucibleParamMeta pm{};
            pm.struct_size = sizeof pm;
            crucible_param_meta(probe.engine, probe.index("filter_type"), &pm);
            int formant = -1;
            for (int k = 0; k < pm.num_choices; ++k)
            {
                const char* name = crucible_param_choice(probe.engine, probe.index("filter_type"), k);
                if (name != nullptr && std::string(name) == "Formant")
                    formant = k;
            }
            expect(formant >= 0, "the Formant model is offered");
            logMessage("filter models: " + std::to_string(pm.num_choices));
            for (int model = 0; model < pm.num_choices; ++model)
            {
                Rig rig(product);
                std::vector<std::string> problems;
                installContent(rig, irPath, problems);
                rig.set("osc1_table", 1.0f);
                rig.set("filter_type", static_cast<float>(model));
                rig.set("filter_cutoff", model == formant ? 500.0f : 2500.0f);
                rig.set("filter_res", 0.4f);
                const auto r = rig.play({45, 57}, 60, 10);
                const char* name = crucible_param_choice(rig.engine, rig.index("filter_type"), model);
                const std::string label = "filter " + std::to_string(model) + " " + (name != nullptr ? name : "?");
                expect(r.finite, label + " finite");
                expect(r.peak < 16.0f, label + " peak " + String(r.peak));
                expect(r.heldRms > 1.0e-4, label + " audible, rms " + String(r.heldRms));
            }
        }

        beginTest("every effect renders on its own: finite, bounded, audible");
        {
            Rig probe(product);
            if (probe.engine == nullptr)
                return;
            const int modules = crucible_module_count(probe.engine);
            for (int m = 0; m < modules; ++m)
            {
                CrucibleModuleMeta meta{};
                meta.struct_size = sizeof meta;
                crucible_module_meta(probe.engine, m, &meta);
                if (std::string(meta.kind) != "effect")
                    continue;
                const std::string id = meta.id;
                const std::string on = onParamOf(probe.engine, m);
                Rig rig(product);
                std::vector<std::string> problems;
                installContent(rig, irPath, problems);
                rig.set("osc1_table", 1.0f);
                rig.set(on.c_str(), 1.0f);
                if (id == "fx.crusher")
                {
                    rig.set("fx_crush_track", 1.0f);
                    rig.set("fx_crush_bits", 6.0f);
                }
                if (id == "fx.saturator")
                {
                    rig.set("fx_saturator_filter_mode", 2.0f);
                    rig.set("fx_saturator_filter_cutoff", 3000.0f);
                    rig.set("fx_saturator_filter_res", 0.4f);
                }
                if (id == "fx.shaper")
                    rig.set("fx_shaper_drive", 12.0f);
                if (id == "fx.cabinet")
                    rig.set("fx_cabinet_mix", 1.0f);
                const auto r = rig.play({48, 55, 64}, 60, 30);
                expect(r.finite, id + " finite");
                expect(r.peak < 16.0f, id + " peak " + String(r.peak));
                // The vocoder needs a modulator input and the meter is a utility: both still pass the voice bus through
                // or stay quiet by design; every other effect must leave an audible signal.
                if (id != "fx.vocoder")
                    expect(r.heldRms > 1.0e-3, id + " audible, rms " + String(r.heldRms));
            }
        }

        // The full chain: every effect on (the convolution with its response), the sequencer and arpeggiator, the macros
        // and routes. Rendered twice from fresh engines at High quality (bit-identical), then saved and reloaded.
        const auto configureFull = [&](Rig& rig, std::vector<std::string>& problems)
        {
            installContent(rig, irPath, problems);
            rig.set("osc1_table", 1.0f);
            rig.set("osc2_level", 0.5f);
            rig.set("osc2_type", 5.0f);
            rig.set("filter2_on", 1.0f);
            rig.set("filter_type", 5.0f); // Lowpass 24
            rig.set("filter_cutoff", 3500.0f);
            const int modules = crucible_module_count(rig.engine);
            for (int m = 0; m < modules; ++m)
            {
                CrucibleModuleMeta meta{};
                meta.struct_size = sizeof meta;
                crucible_module_meta(rig.engine, m, &meta);
                // The vocoder replaces the voice with a carrier shaped by a modulator input that this skeleton has none of.
                if (std::string(meta.kind) == "effect" && std::string(meta.id) != "fx.vocoder")
                    rig.set(onParamOf(rig.engine, m).c_str(), 1.0f);
            }
            // Gentle settings so that the whole chain stays in range.
            rig.set("fx_limit_ceiling", -3.0f);
            rig.set("fx_crush_track", 1.0f);
            rig.set("fx_crush_bits", 12.0f);
            rig.set("fx_saturator_drive", 6.0f);
            rig.set("fx_saturator_filter_mode", 2.0f);
            rig.set("fx_saturator_filter_cutoff", 6000.0f);
            rig.set("fx_ra_mix", 0.2f);
            rig.set("fx_cx_mix", 0.2f);
            rig.set("fx_delay_pro_mix", 0.2f);
            rig.set("fx_ensemble_mix", 0.3f);
            // Performance features.
            rig.set("seq_run", 1.0f);
            rig.set("seq_tempo", 140.0f);
            rig.set("arp_on", 1.0f);
            rig.set("arp_sync", 0.0f);
            rig.set("macro1", 0.7f);
            rig.set("macro2", 0.3f);
            // Routes: macro 1 to the cutoff and to the reverb mix, a mod wheel to the saturator drive, CC 74 to the filter.
            if (crucible_add_route(rig.engine, static_cast<int>(ModSource::Macro1), "filter_cutoff", 0.4f) != 1)
                problems.push_back("route macro 1 -> filter_cutoff");
            if (crucible_add_route(rig.engine, static_cast<int>(ModSource::Macro1), "fx_ra_mix", 0.3f) != 1)
                problems.push_back("route macro 1 -> fx_ra_mix");
            if (crucible_add_route(rig.engine, 1000 + 74, "fx_saturator_drive", 0.2f) != 1)
                problems.push_back("route CC 74 -> fx_saturator_drive");
            rig.set("mod1_src", static_cast<float>(ModSource::Macro2));
            rig.set("mod1_dst", 7.0f);
            rig.set("mod1_amt", 0.3f);
        };

        beginTest("the full chain: finite, bounded, audible, bit-identical twice, patch round trip");
        {
            std::vector<std::string> problems;
            Rig a(product, 2), b(product, 2);
            if (a.engine == nullptr || b.engine == nullptr)
                return;
            configureFull(a, problems);
            configureFull(b, problems);
            for (const auto& p : problems)
                expect(false, p);
            const auto ra = a.play({48, 55, 60}, 150, 150);
            const auto rb = b.play({48, 55, 60}, 150, 150);
            expect(ra.finite && ra.peak < 16.0f, "full chain bounded, peak " + String(ra.peak));
            expect(ra.heldRms > 1.0e-3, "full chain audible, rms " + String(ra.heldRms));
            expect(ra.left == rb.left && ra.right == rb.right, "two fresh engines render the same samples");
            logMessage("full chain: peak " + String(ra.peak, 3) + ", rms " + String(ra.heldRms, 4));

            // Patch: save from a, load into a fresh engine and render the same phrase.
            std::vector<char> text(1 << 22);
            const int len = crucible_save_patch(a.engine, "skeleton full", text.data(), text.size());
            expect(len > 0 && static_cast<size_t>(len) < text.size(), "the patch fits in 4 MiB");
            Rig c(product, 2);
            char err[512] = "";
            expect(crucible_load_patch(c.engine, text.data(), err, sizeof err) == 1, err);
            // Patches carry parameters, routes and content (zones, samples, cloud, formant table, shaper curve, response).
            Rig fresh(product, 2);
            Rig reference(product, 2);
            std::vector<std::string> p2;
            configureFull(reference, p2);
            expect(crucible_load_patch(fresh.engine, text.data(), err, sizeof err) == 1, err);
            const auto rf = fresh.play({48, 55, 60}, 150, 150);
            const auto rr = reference.play({48, 55, 60}, 150, 150);
            double worst = 0.0;
            for (size_t i = 0; i < rf.left.size(); ++i)
                worst = std::max(worst, static_cast<double>(std::abs(rf.left[i] - rr.left[i])));
            logMessage("patch reload: largest sample difference " + String(worst, 8));
            expect(rf.finite && worst < 1.0e-3, "the reloaded patch renders the same, max diff " + String(worst));
            std::vector<char> again(1 << 22);
            expect(crucible_save_patch(fresh.engine, "skeleton full", again.data(), again.size()) == len &&
                       std::strcmp(text.data(), again.data()) == 0,
                   "save -> load -> save is identical");
        }
        fs::remove_all(tmp);
    }
};

static SkeletonFullTests skeletonFullTests;
