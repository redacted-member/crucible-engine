// SPDX-License-Identifier: MIT
// File APIs take UTF-8 paths on every platform (on MSVC a plain std::string path would go through the ANSI code page).
#include <crucible/ArpPatternIO.h>
#include <crucible/AudioFile.h>
#include <crucible/Patch.h>
#include <crucible/PresetLibrary.h>
#include <crucible/Zones.h>
#include "TestSupport.h"

#include <chrono>
#include <filesystem>
#include <fstream>

#include "../src/Utf8Path.h"

using namespace crucible;
using namespace crucible::testing;
namespace fs = std::filesystem;

namespace
{
struct UnicodeDir
{
    fs::path path; // built from a UTF-8 literal, independent of crucible::utf8Path
    UnicodeDir()
    {
        static int counter = 0;
        const std::u8string leaf =
            u8"crucible_utf8_Ünal_日本_" +
            std::u8string(reinterpret_cast<const char8_t*>(
                std::to_string(static_cast<long long>(std::chrono::steady_clock::now().time_since_epoch().count()))
                    .c_str())) +
            u8"_" + std::u8string(reinterpret_cast<const char8_t*>(std::to_string(counter++).c_str()));
        path = fs::temp_directory_path() / fs::path(leaf);
        fs::create_directories(path);
    }
    ~UnicodeDir()
    {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    std::string utf8(const std::string& sub = {}) const
    {
        const auto u =
            (sub.empty() ? path : path / fs::path(std::u8string(reinterpret_cast<const char8_t*>(sub.c_str())))).u8string();
        return std::string(reinterpret_cast<const char*>(u.data()), u.size());
    }
};
} // namespace

class Utf8PathTests : public UnitTest
{
public:
    Utf8PathTests() : UnitTest("UTF-8 file paths", "Engine") {}

    void runTest() override
    {
        beginTest("helpers round trip");
        {
            const std::string s = "a/\xC3\x9C"
                                  "nal_\xE6\x97\xA5\xE6\x9C\xAC/x.wav";
            expectEquals(pathToUtf8(utf8Path(s)), s);
        }

        UnicodeDir dir;

        beginTest("patch file in a non-ASCII folder");
        {
            Patch p;
            p.meta.name = "Utf8";
            p.params.emplace_back("filter_cutoff", 1234.0f);
            const std::string file = dir.utf8("p.crucible-patch");
            expect(savePatchFile(file, p), "save");
            expect(fs::exists(dir.path / "p.crucible-patch"), "file is in the unicode folder");
            Patch back;
            std::string error;
            expect(loadPatchFile(file, back, error), error);
            expectEquals(back.param("filter_cutoff", 0.0f), 1234.0f);
        }

        beginTest("WAV and arp pattern files");
        {
            AudioData a;
            a.sampleRate = 48000.0;
            a.channels.assign(1, std::vector<float>(64, 0.25f));
            const std::string wav = dir.utf8("s.wav");
            expect(writeWavFile(wav, a, 32), "write wav");
            AudioData b;
            std::string error;
            expect(readWavFile(wav, b, error), error);
            expectEquals(b.numFrames(), 64);

            ArpPattern pat;
            pat.length = 5;
            const std::string arp = dir.utf8("a.arp");
            expect(saveArpPatternFile(arp, pat), "save arp");
            ArpPattern back;
            expect(loadArpPatternFile(arp, back, error), error);
            expectEquals(back.length, 5);
        }

        beginTest("zone folder import and preset library under a non-ASCII root");
        {
            fs::create_directories(dir.path / "wavs");
            AudioData a;
            a.channels.assign(1, std::vector<float>(256, 0.1f));
            expect(writeWavFile(dir.utf8("wavs/c4.wav"), a, 16), "write");
            ZoneBank bank;
            std::string error;
            expectEquals(bank.importFolder(0, dir.utf8("wavs"), error), 1);

            PresetLibrary lib("", dir.utf8("user"));
            Patch p;
            p.meta.name = "Caf\xC3\xA9 Pad";
            p.meta.created = "2026-01-01T00:00:00Z";
            p.params.emplace_back("filter_cutoff", 777.0f);
            PresetSaveOptions opt;
            opt.bank = "Banc\xC3\xA9";
            const std::string id = lib.save(p, opt, error);
            expect(!id.empty(), error);
            PresetLibrary lib2("", dir.utf8("user"));
            expect(lib2.scan() >= 1, "rescan finds it");
            Patch back;
            expect(lib2.load(id, back, error), error);
            expectEquals(back.param("filter_cutoff", 0.0f), 777.0f);
        }
    }
};

static Utf8PathTests utf8PathTests;
