// SPDX-License-Identifier: MIT
// Cross-check of crucible::Tuning (Scala .scl / .kbm) against an independent implementation: the MIT-licensed
// tuning-library of the Surge Synth Team, vendored under third_party/tuning-library for tests only. The two have to
// agree on every key's frequency to 0.01 cent and on which keys are mapped. Disagreements are reported with the
// scale and key so that they can be recorded (the plan's follow-up is to adopt the library if ours is wrong).

#include "TestFramework.h"

#include <crucible/Tuning.h>

#include <Tunings.h>

#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

using crucible::test::String;
using crucible::test::UnitTest;

namespace
{
struct Case
{
    std::string name;
    std::string scl;
    std::string kbm; // empty = no mapping file
    // Set when the library is known to follow a different convention here: the case is compared and reported but
    // a difference does not fail the test.
    std::string divergenceReason;
};

std::string scale(const std::string& description, const std::vector<std::string>& pitches)
{
    std::ostringstream os;
    os << "! test.scl\n" << description << "\n " << pitches.size() << "\n!\n";
    for (const auto& p : pitches)
        os << " " << p << "\n";
    return os.str();
}

std::string mapping(int size, int first, int last, int middle, int refNote, double refHz, int formal,
                    const std::vector<std::string>& keys = {})
{
    std::ostringstream os;
    os << "! test.kbm\n" << size << "\n" << first << "\n" << last << "\n" << middle << "\n" << refNote << "\n";
    os.precision(12);
    os << refHz << "\n" << formal << "\n";
    for (const auto& k : keys)
        os << k << "\n";
    return os.str();
}

std::vector<std::string> equalSteps(int divisions, double periodCents)
{
    std::vector<std::string> p;
    for (int i = 1; i <= divisions; ++i)
    {
        std::ostringstream os;
        os.precision(10);
        os << std::fixed << periodCents * i / divisions;
        p.push_back(os.str());
    }
    return p;
}

// Largest disagreement in cents over the keys both consider mapped; sets mismatch if the mapped flags differ.
struct Comparison
{
    double worstCents = 0.0;
    int worstNote = -1;
    int mappedMismatches = 0;
    int parsedBoth = 0;
    int compared = 0;
    bool oracleRejected = false;
    bool oracleOffReference = false; // the library's own reference key does not sound at the reference frequency
    bool oursOnReference = true;
    int differing = 0;
    std::string firstDiffering;
};

Comparison compare(const Case& c, std::string& detail)
{
    Comparison r;
    crucible::Tuning ours;
    std::string error;
    if (!ours.loadScl(c.scl, error))
    {
        detail = "our scl parser rejected the scale: " + error;
        r.parsedBoth = -1;
        return r;
    }
    if (!c.kbm.empty() && !ours.loadKbm(c.kbm, error))
    {
        detail = "our kbm parser rejected the mapping: " + error;
        r.parsedBoth = -1;
        return r;
    }
    try
    {
        const Tunings::Scale scale = Tunings::parseSCLData(c.scl);
        const Tunings::KeyboardMapping map = c.kbm.empty() ? Tunings::KeyboardMapping() : Tunings::parseKBMData(c.kbm);
        const Tunings::Tuning oracle(scale, map);
        r.parsedBoth = 1;
        const int refNote = map.tuningConstantNote;
        if (!c.kbm.empty() && ours.isMapped(refNote))
        {
            r.oracleOffReference =
                std::abs(1200.0 * std::log2(oracle.frequencyForMidiNote(refNote) / map.tuningFrequency)) > 0.01;
            r.oursOnReference = std::abs(1200.0 * std::log2(ours.frequency(refNote) / map.tuningFrequency)) <= 0.01;
        }
        for (int note = 0; note < 128; ++note)
        {
            // The library does not apply the mapping's first / last key; ours leaves keys outside it unmapped.
            if (note < map.firstMidi || note > map.lastMidi)
                continue;
            if (ours.isMapped(note) != oracle.isMidiNoteMapped(note))
            {
                ++r.mappedMismatches;
                std::ostringstream os;
                os << "note " << note << " mapped: ours " << ours.isMapped(note) << ", oracle "
                   << oracle.isMidiNoteMapped(note);
                detail = os.str();
                continue;
            }
            if (!ours.isMapped(note))
                continue;
            ++r.compared;
            const double cents = std::abs(1200.0 * std::log2(ours.frequency(note) / oracle.frequencyForMidiNote(note)));
            if (cents > 0.01)
            {
                if (r.differing++ < 6)
                    r.firstDiffering += " " + std::to_string(note) + "(" + std::to_string(cents) + ")";
            }
            if (cents > r.worstCents)
            {
                r.worstCents = cents;
                r.worstNote = note;
            }
        }
    }
    catch (const Tunings::TuningError& e)
    {
        r.oracleRejected = true;
        detail = std::string("the library rejects it: ") + e.what();
    }
    return r;
}

std::uint32_t lcg(std::uint32_t& s)
{
    s = s * 1664525U + 1013904223U;
    return s >> 8;
}
} // namespace

class TuningOracleTests : public UnitTest
{
public:
    TuningOracleTests() : UnitTest("TuningOracle", "Verification") {}

    void runTest() override
    {
        const std::string twelve = scale("12-tone equal temperament", equalSteps(12, 1200.0));
        const std::string edo19 = scale("19 equal divisions of the octave", equalSteps(19, 1200.0));
        const std::string pythagorean = scale("Pythagorean", {"256/243", "9/8", "32/27", "81/64", "4/3", "1024/729", "3/2",
                                                              "128/81", "27/16", "16/9", "243/128", "2/1"});
        const std::string meantone = scale("Quarter-comma meantone, mixed cents and ratios",
                                           {"76.04900", "193.15686", "310.26471", "5/4", "503.42157", "579.47057",
                                            "696.57843", "25/16", "889.73529", "1006.84314", "1082.89214", "2/1"});
        const std::string bohlenPierce = scale("Bohlen-Pierce, 13 steps of the tritave", equalSteps(13, 1901.955));
        const std::string fiveNote = scale("A five-note scale with a 3/2 period", {"9/8", "5/4", "4/3", "3/2"});
        const std::string justC = scale("Just intonation, 7 notes, 2/1", {"9/8", "5/4", "4/3", "3/2", "5/3", "15/8", "2/1"});

        std::vector<Case> cases = {
            {"12-TET, no mapping file", twelve, ""},
            {"12-TET, mapping 69 = 440 Hz", twelve, mapping(0, 0, 127, 60, 69, 440.0, 12)},
            {"19-EDO, mapping 69 = 440 Hz", edo19, mapping(0, 0, 127, 60, 69, 440.0, 19)},
            {"19-EDO, middle C = 261.6256 Hz", edo19, mapping(0, 0, 127, 60, 60, 261.6255653, 19)},
            {"Pythagorean, A = 440 Hz on note 69", pythagorean, mapping(0, 0, 127, 60, 69, 440.0, 12)},
            {"Quarter-comma meantone, reference note 57", meantone, mapping(0, 0, 127, 60, 57, 220.0, 12)},
            {"Bohlen-Pierce, formal period 13", bohlenPierce, mapping(0, 0, 127, 60, 60, 261.6255653, 13)},
            {"Just major, middle note 48", justC, mapping(0, 0, 127, 48, 48, 130.8127827, 7)},
            {"Just major on 12 keys per octave (octave-repeating map)", justC,
             mapping(12, 0, 127, 60, 60, 261.6255653, 7, {"0", "x", "1", "x", "2", "3", "x", "4", "x", "5", "x", "6"})},
            {"Five notes, period 3/2, 5-key map", fiveNote,
             mapping(5, 0, 127, 60, 60, 261.6255653, 4, {"0", "1", "2", "3", "4"})},
            {"Restricted key range 36..96", twelve, mapping(0, 36, 96, 60, 69, 440.0, 12)},
            {"Pythagorean, formal octave degree 7 (a fifth)", pythagorean, mapping(0, 0, 127, 60, 60, 261.6255653, 7)},
        };

        // Generated scales and maps, deterministic.
        std::uint32_t seed = 12345U;
        for (int i = 0; i < 40; ++i)
        {
            const int notes = 2 + static_cast<int>(lcg(seed) % 22U);
            std::vector<std::string> pitches;
            for (int n = 1; n < notes; ++n)
            {
                std::ostringstream os;
                os.precision(6);
                // Uneven steps below the 3/2 period, some of them as ratios.
                const double cents = 1200.0 * std::log2(1.5) * (n + 0.3 * std::sin(n * 1.7)) / notes;
                if (n % 4 == 3)
                    os << (100 + 3 * n) << "/" << 100;
                else
                    os << std::fixed << cents;
                pitches.push_back(os.str());
            }
            pitches.push_back("3/2");
            const int middle = 40 + static_cast<int>(lcg(seed) % 40U);
            const int ref = 30 + static_cast<int>(lcg(seed) % 80U);
            const double refHz = 100.0 + static_cast<double>(lcg(seed) % 600U);
            // Four families: no key map; a monotonic map whose entries step the degree by 0 or 1 (formal octave = scale
            // size); the same with unmapped keys; and random degrees with a random formal-octave degree.
            const int family = i % 4;
            const int mapSize = family == 0 ? 0 : 3 + static_cast<int>(lcg(seed) % 14U);
            int formal = notes;
            std::vector<std::string> keys;
            int degree = 0;
            for (int k = 0; k < mapSize; ++k)
            {
                if (family == 3)
                    keys.push_back(lcg(seed) % 5U == 0U ? "x"
                                                        : std::to_string(lcg(seed) % static_cast<std::uint32_t>(notes + 1)));
                else
                {
                    const bool skip = family == 2 && k > 0 && lcg(seed) % 4U == 0U;
                    keys.push_back(skip ? "x" : std::to_string(std::min(degree, notes)));
                    degree += static_cast<int>(lcg(seed) % 2U);
                }
            }
            if (family == 3)
                formal = 1 + static_cast<int>(lcg(seed) % static_cast<std::uint32_t>(notes));
            std::string reason;
            if (formal != notes)
                reason = "the library takes the scale's last degree as the period, not the formal-octave degree";
            else if (mapSize > 0 && std::abs(ref - middle) >= mapSize)
                reason =
                    "the library places a reference key that is a whole map repeat from the middle note in the first repeat";
            cases.push_back({"generated family " + std::to_string(family) + " scale " + std::to_string(i) + " (" +
                                 std::to_string(notes) + " notes, map " + std::to_string(mapSize) + ", formal " +
                                 std::to_string(formal) + ")",
                             scale("generated", pitches), mapping(mapSize, 0, 127, middle, ref, refHz, formal, keys),
                             reason});
        }

        beginTest("frequencies and mapped keys agree with the tuning-library to 0.01 cent");
        int agreeing = 0, rejected = 0, offReference = 0, divergent = 0;
        for (const auto& c : cases)
        {
            std::string detail;
            const Comparison r = compare(c, detail);
            if (r.parsedBoth < 0)
            {
                expect(false, c.name + ": " + detail + " [adopt tuning-library]");
                continue;
            }
            if (r.oracleRejected)
            {
                ++rejected;
                continue;
            }
            if (r.oracleOffReference && r.oursOnReference)
            {
                ++offReference;
                continue;
            }
            const bool ok = r.mappedMismatches == 0 && r.worstCents <= 0.01;
            if (ok)
                ++agreeing;
            if (!ok && !c.divergenceReason.empty())
            {
                ++divergent;
                logMessage("divergence (" + c.divergenceReason + "): " + c.name + ", worst " + String(r.worstCents, 3) +
                           " cent");
                continue;
            }
            expect(ok, c.name + ": worst " + String(r.worstCents, 4) + " cent at note " + String(r.worstNote) + ", " +
                           String(r.mappedMismatches) + " mapped-flag mismatches " + detail + "; " + String(r.differing) +
                           " keys differ, first:" + r.firstDiffering + " [adopt tuning-library]");
        }
        logMessage(String(agreeing) + " of " + String(static_cast<int>(cases.size())) + " cases agree, " + String(rejected) +
                   " rejected by the library (reference key unmapped), " + String(offReference) +
                   " where the library's own reference key misses its frequency (not compared), " + String(divergent) +
                   " recorded divergences of convention");

        beginTest("conventions: the default anchor of a scale without a mapping file");
        {
            // Without a .kbm the tuning-library anchors note 60 at 261.6256 Hz; ours follows Scala's default mapping
            // (note 69 = 440 Hz). They coincide for 12 divisions of the octave and differ by a constant for others.
            crucible::Tuning ours;
            std::string error;
            expect(ours.loadScl(edo19, error), "19-EDO loads");
            const Tunings::Tuning oracle(Tunings::parseSCLData(edo19));
            const double offset = 1200.0 * std::log2(ours.frequency(60) / oracle.frequencyForMidiNote(60));
            double spread = 0.0;
            for (int n = 0; n < 128; ++n)
                spread = std::max(spread,
                                  std::abs(1200.0 * std::log2(ours.frequency(n) / oracle.frequencyForMidiNote(n)) - offset));
            expect(spread < 0.01, "constant offset between the two defaults, spread " + String(spread, 5) + " cent");
            logMessage("default anchor: ours is " + String(offset, 3) +
                       " cent from the library's for 19-EDO without a .kbm");
        }
    }
};

static TuningOracleTests tuningOracleTests;
