// SPDX-License-Identifier: MIT

// Consumer smoke test: the installed headers and library work together, the version agrees with the package version,
// a note produces sound, and a module written with the SDK headers registers with the host.

#include <crucible/Engine.h>
#include <crucible/Version.h>
#include <crucible/sdk/Host.h>
#include <crucible/sdk/Module.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

class Gain
{
public:
    static crucible::sdk::Info info()
    {
        crucible::sdk::Info i;
        i.id = "consumer.gain";
        i.name = "Gain";
        i.params = {crucible::sdk::Param::floating("level", "Level", 0.0f, 2.0f, 1.0f)};
        return i;
    }
    void process(crucible::sdk::Process& p)
    {
        for (int i = 0; i < p.frames(); ++i)
        {
            p.left()[i] *= p.param(0);
            p.right()[i] *= p.param(0);
        }
    }
};

int main(int argc, char** argv)
{
    if (argc > 1 && std::strcmp(argv[1], CRUCIBLE_VERSION_STRING) != 0)
    {
        std::printf("header version %s does not match package version %s\n", CRUCIBLE_VERSION_STRING, argv[1]);
        return 1;
    }
    if (std::strcmp(crucible::versionString(), CRUCIBLE_VERSION_STRING) != 0)
    {
        std::printf("library version %s does not match the header %s\n", crucible::versionString(), CRUCIBLE_VERSION_STRING);
        return 1;
    }
    crucible::sdk::ModuleSet modules;
    if (modules.add(crucible::sdk::Module<Gain>::descriptor()) != CRUCIBLE_MODULE_OK || modules.count() != 1)
    {
        std::printf("module registration failed: %s\n", modules.error().c_str());
        return 1;
    }
    crucible::Engine engine;
    engine.prepare(48000.0, 256);
    engine.noteOn(60, 0.8f);
    std::vector<float> left(256), right(256);
    float peak = 0.0f;
    for (int block = 0; block < 100; ++block)
    {
        engine.process(left.data(), right.data(), 256);
        for (int i = 0; i < 256; ++i)
            peak = std::max(peak, std::fabs(left[static_cast<size_t>(i)]));
    }
    std::printf("Crucible Engine %s, peak %.3f\n", crucible::versionString(), static_cast<double>(peak));
    return peak > 1.0e-3f ? 0 : 1;
}
