// SPDX-License-Identifier: MIT
// crucible-render: offline renderer and inspector for CrucibleEngine.
//
//   crucible-render [options] -o out.wav
//     --product FILE        play a product definition (crucible-product JSON, see products/) instead of the default
//     --patch FILE          load a patch (JSON)
//     --midi FILE           play a standard MIDI file
//     --score FILE          play a text score (see below)
//     --note N --length S   play one note (default: note 60 for 2 s when no MIDI or score is given)
//     --velocity V          velocity 1..127 for --note (default 100)
//     --set ID=VALUE        set a parameter (repeatable, applied after the patch)
//     --rate HZ             sample rate (default 48000)
//     --block N             block size (default 256)
//     --tail S              seconds rendered after the last event (default 2)
//     --bpm B               tempo for scores and tempo-synced modules (default 120 or the MIDI file's first tempo)
//     --seed N              engine seed (deterministic output per seed)
//     --voices N            voice pool size
//     --bits 16|32          output format (default 32-bit float)
//     --save-patch FILE     write the engine state (after --patch / --set) as a patch
//   crucible-render --list-modules        modules with kind, category, CPU class and parameter count
//   crucible-render --params [--markdown] parameter table (JSON, or Markdown for docs/PARAMETERS.md)
//
// Score format: one event per line, '#' starts a comment; times and lengths in beats.
//   tempo 128
//   note <beat> <midi note> <velocity 1-127> <length>
//   cc <beat> <controller> <value 0-127>
//   bend <beat> <-1..1>

#include <crucible/AudioFile.h>
#include <crucible/Version.h>
#include <crucible/Engine.h>
#include <crucible/graph/Compiler.h>
#include <crucible/Json.h>
#include <crucible/MidiFile.h>
#include <crucible/Modulation.h>
#include <crucible/Patch.h>
#include <crucible/module/Module.h>
#include "../WinArgv.h"
#include "../../src/Utf8Path.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using namespace crucible;

struct Options
{
    std::string patch, midi, score, out, savePatch, product;
    std::vector<std::pair<std::string, float>> sets;
    int note = -1;
    double length = 2.0;
    int velocity = 100;
    double rate = 48000.0;
    int block = 256;
    double tail = 2.0;
    double bpm = 0.0;
    std::uint32_t seed = 0;
    int voices = 16;
    int bits = 32;
};

int usage()
{
    std::fprintf(stderr, "usage: crucible-render [--patch F] [--midi F | --score F | --note N --length S] [--set id=v]... "
                         "-o out.wav\n       crucible-render --list-modules | --params [--markdown]\n");
    return 2;
}

bool readScore(const std::string& path, double& bpm, std::vector<TimedMidi>& out, std::string& error)
{
    std::ifstream f(crucible::utf8Path(path));
    if (!f)
    {
        error = "cannot read " + path;
        return false;
    }
    struct Beat
    {
        double beat;
        std::uint8_t s, d1, d2;
    };
    std::vector<Beat> beats;
    std::string line;
    int lineNo = 0;
    while (std::getline(f, line))
    {
        ++lineNo;
        const auto hash = line.find('#');
        if (hash != std::string::npos)
            line.erase(hash);
        std::istringstream is(line);
        is.imbue(std::locale::classic());
        std::string kind;
        if (!(is >> kind))
            continue;
        if (kind == "tempo")
            is >> bpm;
        else if (kind == "note")
        {
            double at, len;
            int note, vel;
            if (!(is >> at >> note >> vel >> len))
            {
                error = "line " + std::to_string(lineNo) + ": note <beat> <note> <velocity> <length>";
                return false;
            }
            beats.push_back({at, 0x90, static_cast<std::uint8_t>(std::clamp(note, 0, 127)),
                             static_cast<std::uint8_t>(std::clamp(vel, 1, 127))});
            beats.push_back({at + len, 0x80, static_cast<std::uint8_t>(std::clamp(note, 0, 127)), 0});
        }
        else if (kind == "cc")
        {
            double at;
            int cc, value;
            if (!(is >> at >> cc >> value))
            {
                error = "line " + std::to_string(lineNo) + ": cc <beat> <controller> <value>";
                return false;
            }
            beats.push_back({at, 0xB0, static_cast<std::uint8_t>(std::clamp(cc, 0, 127)),
                             static_cast<std::uint8_t>(std::clamp(value, 0, 127))});
        }
        else if (kind == "bend")
        {
            double at, v;
            if (!(is >> at >> v))
            {
                error = "line " + std::to_string(lineNo) + ": bend <beat> <-1..1>";
                return false;
            }
            const int raw = std::clamp(static_cast<int>(std::lround((v + 1.0) * 8192.0)), 0, 16383);
            beats.push_back({at, 0xE0, static_cast<std::uint8_t>(raw & 0x7f), static_cast<std::uint8_t>(raw >> 7)});
        }
        else
        {
            error = "line " + std::to_string(lineNo) + ": unknown event '" + kind + "'";
            return false;
        }
    }
    std::stable_sort(beats.begin(), beats.end(), [](const Beat& a, const Beat& b) { return a.beat < b.beat; });
    for (const auto& b : beats)
        out.push_back({b.beat * 60.0 / bpm, b.s, b.d1, b.d2});
    return true;
}

int listModules()
{
    for (const auto& m : ModuleRegistry::instance().modules())
    {
        static const char* cpu[] = {"light", "medium", "heavy"};
        std::printf("%-18s %-12s %-12s %-7s v%d  %3zu params  %s\n", m.id.c_str(), moduleKindName(m.kind),
                    m.category.c_str(), cpu[static_cast<int>(m.cpu)], m.version, m.params.size(), m.name.c_str());
    }
    return 0;
}

int dumpParams(bool markdown)
{
    const auto& defs = ParameterMap::params();
    if (!markdown)
    {
        std::printf("%s", ParameterMap::toJson().c_str());
        return 0;
    }
    static const char* types[] = {"float", "int", "choice", "bool"};
    std::printf("# CrucibleEngine parameters\n\nGenerated by `crucible-render --params --markdown`; do not edit by hand. "
                "Values are natural units. The default product has %zu parameters, %zu modulation targets.\n",
                defs.size(), ParameterMap::modTargets().size());
    std::string module;
    for (const auto& p : defs)
    {
        if (p.module != module)
        {
            module = p.module;
            const auto* m = ModuleRegistry::instance().find(module);
            std::printf("\n## %s (`%s`)\n\n%s\n\n| id | name | type | range | default | unit | modulation "
                        "|\n|---|---|---|---|---|---|---|\n",
                        m != nullptr ? m->name.c_str() : module.c_str(), module.c_str(),
                        m != nullptr ? m->description.c_str() : "");
        }
        std::string range;
        char buf[96];
        if (p.type == ParamType::Choice)
            range = std::to_string(p.choices.size()) + " choices";
        else
        {
            std::snprintf(buf, sizeof(buf), "%g .. %g", p.min, p.max);
            range = buf;
        }
        std::snprintf(buf, sizeof(buf), "%g", p.defaultValue);
        std::string def = buf;
        if (p.type == ParamType::Choice && static_cast<size_t>(p.defaultValue) < p.choices.size())
            def = p.choices[static_cast<size_t>(p.defaultValue)];
        std::printf("| `%s` | %s | %s | %s | %s | %s | %s |\n", p.id.c_str(), p.name.c_str(),
                    types[static_cast<int>(p.type)], range.c_str(), def.c_str(), p.unit.c_str(),
                    p.modulatable ? "global" : (routeVoiceDestination(p.id) != ModDest::None ? "voice" : ""));
    }
    return 0;
}
} // namespace

int main(int argc, char** argv)
{
    const crucible::tools::Utf8Args utf8Args(argc, argv);
    argc = utf8Args.argc;
    argv = utf8Args.argv;
    Options o;
    bool markdown = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--version")
        {
            std::printf("%s (Crucible Engine) %s\n", "crucible-render", CRUCIBLE_VERSION_STRING);
            return 0;
        }
        const auto next = [&]() -> std::string
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "missing value after %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--list-modules")
            return listModules();
        if (a == "--markdown")
            markdown = true;
        else if (a == "--params")
        {
            for (int k = i + 1; k < argc; ++k)
                markdown = markdown || std::string(argv[k]) == "--markdown";
            return dumpParams(markdown);
        }
        else if (a == "--product")
            o.product = next();
        else if (a == "--patch")
            o.patch = next();
        else if (a == "--midi")
            o.midi = next();
        else if (a == "--score")
            o.score = next();
        else if (a == "-o" || a == "--out")
            o.out = next();
        else if (a == "--save-patch")
            o.savePatch = next();
        else if (a == "--note")
            o.note = std::atoi(next().c_str());
        else if (a == "--length")
            o.length = std::atof(next().c_str());
        else if (a == "--velocity")
            o.velocity = std::atoi(next().c_str());
        else if (a == "--rate")
            o.rate = std::atof(next().c_str());
        else if (a == "--block")
            o.block = std::max(1, std::atoi(next().c_str()));
        else if (a == "--tail")
            o.tail = std::atof(next().c_str());
        else if (a == "--bpm")
            o.bpm = std::atof(next().c_str());
        else if (a == "--seed")
            o.seed = static_cast<std::uint32_t>(std::strtoul(next().c_str(), nullptr, 10));
        else if (a == "--voices")
            o.voices = std::atoi(next().c_str());
        else if (a == "--bits")
            o.bits = std::atoi(next().c_str()) == 16 ? 16 : 32;
        else if (a == "--set")
        {
            const auto kv = next();
            const auto eq = kv.find('=');
            if (eq == std::string::npos)
                return usage();
            o.sets.emplace_back(kv.substr(0, eq), static_cast<float>(std::atof(kv.c_str() + eq + 1)));
        }
        else
            return usage();
    }
    if (o.out.empty() && o.savePatch.empty())
        return usage();

    EngineConfig config;
    config.maxVoices = o.voices;
    config.maxBlockSize = std::max(o.block, 1);
    config.seed = o.seed;
    config.quality = Quality::High;
    std::string error;
    std::shared_ptr<const graph::CompiledProduct> product;
    if (!o.product.empty())
    {
        graph::ProductDef def;
        if (!graph::loadProduct(o.product, def, error) || (product = graph::compile(def, error)) == nullptr)
        {
            std::fprintf(stderr, "product: %s\n", error.c_str());
            return 1;
        }
        config.compiled = product;
    }
    Engine engine(config);
    engine.prepare(o.rate, o.block);
    if (product)
        for (const auto& [id, v] : product->def.defaults)
            engine.setParam(id, static_cast<float>(v));
    if (!o.patch.empty())
    {
        Patch p;
        if (!loadPatchFile(o.patch, p, error))
        {
            std::fprintf(stderr, "patch: %s\n", error.c_str());
            return 1;
        }
        applyPatch(engine, p);
    }
    for (const auto& [id, v] : o.sets)
        if (!engine.setParam(id, v))
        {
            std::fprintf(stderr, "unknown parameter %s\n", id.c_str());
            return 1;
        }
    if (!o.savePatch.empty())
    {
        PatchMeta meta;
        meta.name = o.savePatch.substr(o.savePatch.find_last_of("/\\") + 1);
        if (const auto dot = meta.name.rfind('.'); dot != std::string::npos)
            meta.name.erase(dot);
        if (!savePatchFile(o.savePatch, capturePatch(engine, meta)))
        {
            std::fprintf(stderr, "cannot write %s\n", o.savePatch.c_str());
            return 1;
        }
        if (o.out.empty())
            return 0;
    }

    std::vector<TimedMidi> events;
    double bpm = o.bpm > 0.0 ? o.bpm : 120.0;
    if (!o.midi.empty())
    {
        MidiFileInfo info;
        if (!readMidiFile(o.midi, events, info, error))
        {
            std::fprintf(stderr, "midi: %s\n", error.c_str());
            return 1;
        }
        if (o.bpm <= 0.0)
            bpm = info.firstTempoBpm;
    }
    else if (!o.score.empty())
    {
        if (!readScore(o.score, bpm, events, error))
        {
            std::fprintf(stderr, "score: %s\n", error.c_str());
            return 1;
        }
    }
    else
    {
        const int note = o.note >= 0 ? o.note : 60;
        events.push_back(
            {0.0, 0x90, static_cast<std::uint8_t>(note), static_cast<std::uint8_t>(std::clamp(o.velocity, 1, 127))});
        events.push_back({o.length, 0x80, static_cast<std::uint8_t>(note), 0});
    }

    const double end = (events.empty() ? 0.0 : events.back().seconds) + o.tail;
    const auto total = static_cast<long long>(end * o.rate);
    AudioData out;
    out.sampleRate = o.rate;
    out.channels.assign(2, std::vector<float>(static_cast<size_t>(total), 0.0f));
    std::vector<MidiInput> block;
    size_t next = 0;
    SeqHostTime host;
    host.hasBpm = true;
    host.bpm = bpm;
    for (long long pos = 0; pos < total; pos += o.block)
    {
        const int n = static_cast<int>(std::min<long long>(o.block, total - pos));
        block.clear();
        while (next < events.size() && static_cast<long long>(events[next].seconds * o.rate) < pos + n)
        {
            const auto& e = events[next++];
            const auto at = std::max<long long>(static_cast<long long>(e.seconds * o.rate) - pos, 0);
            block.push_back({static_cast<int>(at), e.status, e.data1, e.data2});
        }
        engine.process(out.channels[0].data() + pos, out.channels[1].data() + pos, n, block.data(),
                       static_cast<int>(block.size()), &host);
    }
    float peak = 0.0f;
    for (const auto& ch : out.channels)
        for (const float x : ch)
            peak = std::max(peak, std::abs(x));
    if (!writeWavFile(o.out, out, o.bits))
    {
        std::fprintf(stderr, "cannot write %s\n", o.out.c_str());
        return 1;
    }
    std::printf("%s: %.2f s at %.0f Hz, %zu events, peak %.1f dBFS\n", o.out.c_str(), end, o.rate, events.size(),
                peak > 0.0f ? 20.0 * std::log10(peak) : -120.0);
    return 0;
}
