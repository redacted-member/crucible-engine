// SPDX-License-Identifier: MIT
// Bit-exact regression: a 64-bit FNV-1a hash of every rendered sample for a set of scenarios that cover the voice
// (oscillators, warps, sub, noise, filter routing, matrix, user tables) and the effects. Unlike the RMS
// fingerprints in QaTests.cpp (0.05 dB tolerance), any change of a single bit fails here, so refactors of the
// voice graph can prove they keep existing patches identical.
//
// Hashes depend on the compiler and the CPU architecture (libm, FMA contraction), so the file records the build
// it was made with; other builds skip the comparison and only check determinism. After an intended change of
// sound, regenerate with
//   CRUCIBLE_UPDATE_BITEXACT=1 build/crucible_engine_tests Bit-exact
// and document the change.

#include <crucible/Engine.h>
#include <crucible/Simd.h>
#include "TestSupport.h"

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

#ifndef CRUCIBLE_ENGINE_GOLDEN_DIR
#define CRUCIBLE_ENGINE_GOLDEN_DIR "tests/golden"
#endif

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct BitScenario
{
    std::string name;
    std::vector<std::pair<std::string, float>> params;
    std::vector<int> notes;
    int length = 48000;
    bool cloudTable = false;
    EngineConfig config{};
};

std::vector<BitScenario> bitScenarios()
{
    std::vector<BitScenario> list = {
        {"default_chord", {}, {48, 55, 60}},
        {"warps_filters_matrix",
         {{"osc1_warp", 1.0f},
          {"osc1_warp_a", 0.5f},
          {"osc1_uni_voices", 4.0f},
          {"osc2_level", 0.6f},
          {"osc2_warp", 6.0f},
          {"filter_type", 9.0f},
          {"filter_res", 0.6f},
          {"filter_cutoff", 1200.0f},
          {"filter2_on", 1.0f},
          {"filter2_type", 12.0f},
          {"filter_route", 1.0f},
          {"mod1_src", 1.0f},
          {"mod1_dst", 7.0f},
          {"mod1_amt", 0.4f}},
         {45, 57}},
        {"classic_fx",
         {{"osc1_pos", 1.0f},
          // The first-generation effects of this scenario (drive, chorus, phaser, delay, reverb, comp, eq) became
          // their successors at the closest settings when they were removed (docs/SOUND_CHANGES.md).
          {"fx_saturator_on", 1.0f},
          {"fx_saturator_drive", 12.0f},
          {"fx_saturator_autogain", 0.0f},
          {"fx_saturator_output", -4.2f},
          {"fx_ensemble_on", 1.0f},
          {"fx_ensemble_noise", -100.0f},
          {"fx_ensemble_compander", 0.0f},
          {"fx_phaser2_on", 1.0f},
          {"fx_phaser2_stages", 3.0f},
          {"fx_phaser2_range", 1.8f},
          {"fx_delay_pro_on", 1.0f},
          {"fx_delay_pro_saturation", 0.0f},
          {"fx_ra_on", 1.0f},
          {"fx_ra_algo", 3.0f},
          {"fx_ra_mix", 0.25f},
          {"fx_cc_on", 1.0f},
          {"fx_ep_on", 1.0f},
          {"fx_ep_b1_on", 1.0f},
          {"fx_ep_b1_type", 1.0f},
          {"fx_ep_b1_freq", 120.0f},
          {"fx_ep_b2_on", 1.0f},
          {"fx_ep_b2_freq", 1000.0f},
          {"fx_ep_b3_on", 1.0f},
          {"fx_ep_b3_type", 2.0f},
          {"fx_ep_b3_freq", 6000.0f},
          {"fx_limit_on", 1.0f},
          {"fx_width_on", 1.0f}},
         {52, 59}},
        {"new_fx",
         {{"osc1_pos", 1.0f},
          {"fx_ep_on", 1.0f},
          {"fx_ep_b1_on", 1.0f},
          {"fx_ep_b1_freq", 800.0f},
          {"fx_ep_b1_gain", 6.0f},
          {"fx_ep_b1_q", 0.707f},
          {"fx_filter_on", 1.0f},
          {"fx_filter_lfo_depth", 2.0f},
          {"fx_ensemble_on", 1.0f},
          {"fx_ensemble_mode", 2.0f},
          {"fx_ensemble_compander", 0.0f},
          {"fx_multiband_on", 1.0f},
          {"fx_trance_on", 1.0f},
          {"fx_tremolo_on", 1.0f},
          {"fx_ring_on", 1.0f},
          {"fx_ring_mix", 0.3f}},
         {50, 62}},
        {"arp_and_sequencer",
         {{"arp_on", 1.0f},
          {"arp_sync", 0.0f},
          {"arp_mode", 2.0f},
          {"arp_octaves", 2.0f},
          {"seq_run", 1.0f},
          {"seq_tempo", 120.0f},
          {"amp_release", 0.05f}},
         {60, 64, 67}},
        {"split_routes",
         {{"filter_route", 2.0f},
          {"osc2_level", 0.5f},
          {"osc2_route", 1.0f},
          {"osc3_level", 0.4f},
          {"osc3_route", 3.0f},
          {"sub_level", 0.5f},
          {"sub_route", 0.0f},
          {"noise_level", 0.2f},
          {"noise_type", 1.0f},
          {"noise_route", 1.0f},
          {"filter2_on", 1.0f},
          {"filter2_type", 5.0f},
          {"filter_pan", -0.5f},
          {"filter2_pan", 0.5f},
          {"filter2_mix", 0.7f},
          {"filter_cutoff", 900.0f}},
         {43, 55}},
        {"serial_two_filters",
         {{"filter_route", 0.0f},
          {"filter2_on", 1.0f},
          {"filter2_type", 1.0f},
          {"filter_type", 8.0f},
          {"filter_drive", 0.6f},
          {"filter2_drive", 0.3f},
          {"osc2_level", 0.5f},
          {"sub_level", 0.3f},
          {"sub_wave", 2.0f},
          {"noise_level", 0.1f},
          {"noise_type", 2.0f},
          {"filter_env", 0.5f},
          {"filter_key", 0.5f}},
         {40, 52}},
        {"parallel_filter1_off",
         {{"filter_route", 1.0f},
          {"filter_on", 0.0f},
          {"filter2_on", 1.0f},
          {"osc3_level", 0.5f},
          {"noise_type", 3.0f},
          {"noise_level", 0.3f},
          {"sub_level", 0.4f},
          {"sub_wave", 1.0f},
          {"sub_octave", 1.0f}},
         {48, 60}},
        {"fm_ring_sync",
         {{"osc1_warp", 1.0f},
          {"osc1_warp_a", 0.4f},
          {"osc1_warp_b", 0.3f},
          {"osc2_level", 0.5f},
          {"osc2_warp", 13.0f},
          {"osc2_warp_a", 0.7f},
          {"osc2_warp_src", 2.0f},
          {"osc3_level", 0.5f},
          {"osc3_warp", 2.0f},
          {"osc3_warp_a", 0.3f},
          {"osc3_uni_voices", 3.0f},
          {"osc3_uni_mode", 1.0f},
          {"osc2_table", 3.0f},
          {"osc3_table", 5.0f}},
         {45, 52}},
        {"mono_legato_glide",
         {{"voice_mode", 2.0f}, {"voice_glide", 0.1f}, {"osc1_uni_voices", 3.0f}, {"voice_drift", 0.5f}},
         {48, 55, 60}},
        {"matrix_voice_dests",
         {{"filter2_on", 1.0f},     {"filter_route", 1.0f}, {"osc2_level", 0.5f}, {"osc3_level", 0.5f}, {"mod1_src", 1.0f},
          {"mod1_dst", 23.0f},      {"mod1_amt", 0.8f},     {"mod2_src", 3.0f},   {"mod2_dst", 30.0f},  {"mod2_amt", -0.5f},
          {"mod3_src", 4.0f},       {"mod3_dst", 16.0f},    {"mod3_amt", 0.4f},   {"mod4_src", 2.0f},   {"mod4_dst", 20.0f},
          {"mod4_amt", 0.6f},       {"mod5_src", 5.0f},     {"mod5_dst", 33.0f},  {"mod5_amt", 0.1f},   {"mod6_src", 13.0f},
          {"mod6_dst", 32.0f},      {"mod6_amt", 0.7f},     {"mod7_src", 1.0f},   {"mod7_dst", 12.0f},  {"mod7_amt", 0.5f},
          {"osc1_uni_voices", 2.0f}},
         {50, 57}},
        {"drift_stacks_sub",
         {{"voice_drift", 1.0f},
          {"osc1_uni_voices", 7.0f},
          {"osc1_uni_mode", 3.0f},
          {"osc1_uni_scatter", 0.5f},
          {"sub_level", 0.4f},
          {"sub_octave", 1.0f},
          {"sub_wave", 3.0f},
          {"osc2_level", 0.3f},
          {"osc2_uni_voices", 5.0f},
          {"osc2_uni_mode", 7.0f}},
         {36, 48}},
    };
    BitScenario cloud{"user_cloud_table", {{"osc1_pos", 0.5f}, {"osc1_warp", 9.0f}, {"osc1_warp_a", 0.5f}}, {57}};
    cloud.cloudTable = true;
    list.push_back(cloud);
    for (int w = 1; w < 15; ++w)
    {
        BitScenario s;
        s.name = "warp_" + std::to_string(w);
        s.params = {{"osc1_warp", static_cast<float>(w)},
                    {"osc1_warp_a", 0.6f},
                    {"osc1_warp_b", 0.3f},
                    {"osc1_pos", 0.3f},
                    {"osc2_level", 0.3f}};
        s.notes = {69};
        s.length = 12000;
        list.push_back(s);
    }
    BitScenario small{"small_pool", {{"osc1_uni_voices", 2.0f}}, {40, 44, 47, 52}};
    small.config.maxVoices = 3;
    list.push_back(small);
    return list;
}

std::uint64_t fnv1a(std::uint64_t h, const void* data, size_t bytes)
{
    const auto* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i)
    {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

std::uint64_t renderHash(const BitScenario& s)
{
    Engine e(s.config);
    e.prepare(kRate, 512);
    if (s.cloudTable)
        e.loadCloudWavetable(0, CloudModel::harmonicMorph(48));
    for (const auto& [id, v] : s.params)
        e.setParam(id, v);
    if (s.cloudTable)
        e.setParam("osc1_table", static_cast<float>(WavetableBank::numTables));
    CRUCIBLE_HEAP(SeqBank, bank);
    bank.pattern[0].steps[0] = {true, 36, 110, 1, 0.5f, 1.0f, {}};
    bank.pattern[0].steps[6] = {true, 43, 90, 2, 0.5f, 1.0f, {}};
    e.sequencer().replace(bank);
    const int total = s.length;
    const int releaseAt = total * 5 / 8;
    std::vector<float> l(static_cast<size_t>(total)), r(static_cast<size_t>(total));
    std::vector<MidiInput> on, off;
    for (const int n : s.notes)
        on.push_back({0, 0x90, static_cast<std::uint8_t>(n), 100});
    std::uint64_t h = 1469598103934665603ULL;
    constexpr int block = 256;
    for (int pos = 0; pos < total; pos += block)
    {
        const int n = std::min(block, total - pos);
        off.clear();
        if (pos <= releaseAt && pos + n > releaseAt)
            for (const int note : s.notes)
                off.push_back({releaseAt - pos, 0x80, static_cast<std::uint8_t>(note), 0});
        const auto& ev = pos == 0 ? on : off;
        e.process(l.data() + pos, r.data() + pos, n, ev.empty() ? nullptr : ev.data(), static_cast<int>(ev.size()));
    }
    h = fnv1a(h, l.data(), l.size() * sizeof(float));
    h = fnv1a(h, r.data(), r.size() * sizeof(float));
    return h;
}

std::string buildKey()
{
    std::string arch =
#if defined(__aarch64__)
        "arm64";
#elif defined(__x86_64__) || defined(_M_X64)
        "x86_64";
#else
        "other";
#endif
#if defined(__clang__)
    arch += std::string("/clang ") + __clang_version__;
#elif defined(__GNUC__)
    arch += std::string("/gcc ") + __VERSION__;
#endif
    // The scalar fallback and the AVX2 build take other arithmetic paths than the default vector kernels.
#if !CRUCIBLE_SIMD_NATIVE
    arch += "/scalar";
#elif defined(__AVX2__)
    arch += "/avx2";
#endif
    // Sanitizer builds are compiled at another optimisation level (different FMA contraction), so they only check
    // determinism, like any other build the hashes were not recorded with.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
    arch += "/sanitized";
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
    arch += "/sanitized";
#endif
#endif
    for (auto& c : arch)
        if (c == ' ')
            c = '_';
    return arch;
}

std::string hex(std::uint64_t v)
{
    std::ostringstream os;
    os << std::hex << v;
    return os.str();
}
} // namespace

class BitExactTests : public UnitTest
{
public:
    BitExactTests() : UnitTest("Bit-exact renders", "Engine") {}

    void runTest() override
    {
        const std::string path = std::string(CRUCIBLE_ENGINE_GOLDEN_DIR) + "/bitexact.txt";
        const auto scenarios = bitScenarios();
        std::map<std::string, std::uint64_t> now;
        for (const auto& s : scenarios)
            now[s.name] = renderHash(s);

        beginTest("rendering is deterministic (same input, same bits)");
        for (const auto& s : scenarios)
            if (s.name.rfind("warp_", 0) != 0)
                expect(renderHash(s) == now[s.name], s.name);

        beginTest("existing patches render bit-identically to the recorded build");
        const char* update = std::getenv("CRUCIBLE_UPDATE_BITEXACT");
        if (update != nullptr && std::string(update) == "1")
        {
            std::ofstream f(path);
            f << "# FNV-1a 64 of the rendered float samples per scenario; see tests/BitExactTests.cpp\n";
            f << "build " << buildKey() << "\n";
            for (const auto& s : scenarios)
                f << s.name << ' ' << hex(now[s.name]) << '\n';
            logMessage("bit-exact hashes rewritten: " + path);
        }
        std::ifstream f(path);
        std::string line, key;
        std::map<std::string, std::string> golden;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream is(line);
            std::string a, b;
            is >> a >> b;
            if (a == "build")
                key = b;
            else
                golden[a] = b;
        }
        if (key != buildKey())
        {
            logMessage("bit-exact comparison skipped: hashes were recorded with " + key + ", this build is " + buildKey());
            expect(true);
            return;
        }
        for (const auto& s : scenarios)
        {
            const auto it = golden.find(s.name);
            expect(it != golden.end(), "missing hash for " + s.name);
            if (it != golden.end())
                expect(it->second == hex(now[s.name]), s.name + " changed: " + it->second + " -> " + hex(now[s.name]));
        }
    }
};

static BitExactTests bitExactTests;
