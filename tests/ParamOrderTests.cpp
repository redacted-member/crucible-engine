// SPDX-License-Identifier: MIT
// Frozen parameter order: every parameter id of the default product keeps its index and its generic modulation
// destination number, and new parameters only append. Regenerate the snapshot (after an intended addition) with
//   CRUCIBLE_UPDATE_PARAM_ORDER=1 build/crucible_engine_tests "Parameter order"
//
// The snapshot shrank once, on 2026-10-19, when the twelve first-generation effects were removed (and with them the five
// fixed FX modulation destinations, which moved every generic destination number down by five). That is allowed before
// v1 and never after: from v1 on, parameters and destinations are never removed or renumbered.

#include "TestFramework.h"

#include <crucible/ModMatrix.h>
#include <crucible/ParameterMap.h>
#include <crucible/module/Module.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace crucible;
using namespace crucible::test;

namespace
{
struct Entry
{
    std::string id;
    int index = 0;
    int dest = 0; // matrix destination number, 0 when the parameter is not a generic modulation target
};

std::vector<Entry> currentOrder()
{
    std::vector<int> destOf(ParameterMap::params().size(), 0);
    const auto& targets = ParameterMap::modTargets();
    for (size_t k = 0; k < targets.size(); ++k)
        destOf[static_cast<size_t>(targets[k])] = kNumModDests + static_cast<int>(k);
    std::vector<Entry> out;
    for (size_t i = 0; i < ParameterMap::params().size(); ++i)
        out.push_back({ParameterMap::params()[i].id, static_cast<int>(i), destOf[i]});
    return out;
}

ModuleDescriptor appendedModule(const std::string& id, int sequence)
{
    ModuleDescriptor d;
    d.id = id;
    d.name = id;
    d.kind = ModuleKind::Effect;
    d.sectionHint = "fx";
    for (const char* suffix : {"_a", "_b"})
    {
        ParamDef p;
        p.id = "zz_" + id.substr(id.find('.') + 1) + suffix;
        p.name = p.id;
        p.pageId = "zz_page";
        p.min = 0.0f;
        p.max = 1.0f;
        p.modulatable = true;
        p.appended = sequence;
        d.params.push_back(p);
    }
    d.pages.push_back({"zz_page", "Test", "fx", "Test", "Test page."});
    return d;
}
} // namespace

class ParamOrderTests : public UnitTest
{
public:
    ParamOrderTests() : UnitTest("Parameter order", "Engine") {}

    void runTest() override
    {
        const std::string path = std::string(CRUCIBLE_ENGINE_GOLDEN_DIR) + "/param_order.txt";
        const auto now = currentOrder();

        beginTest("the frozen snapshot: every parameter keeps its index and destination, new ones only append");
        {
            const char* update = std::getenv("CRUCIBLE_UPDATE_PARAM_ORDER");
            if (update != nullptr && std::string(update) == "1")
            {
                std::ofstream f(path);
                f << "# parameter id, index, generic modulation destination (0 = none) of the default product; "
                     "see tests/ParamOrderTests.cpp\n";
                for (const auto& e : now)
                    f << e.id << ' ' << e.index << ' ' << e.dest << '\n';
                logMessage("parameter order rewritten: " + path);
            }
            std::vector<Entry> frozen;
            std::ifstream f(path);
            std::string line;
            while (std::getline(f, line))
            {
                if (line.empty() || line[0] == '#')
                    continue;
                std::istringstream is(line);
                is.imbue(std::locale::classic());
                Entry e;
                is >> e.id >> e.index >> e.dest;
                frozen.push_back(e);
            }
            expect(!frozen.empty(), "snapshot file " + path);
            expect(now.size() >= frozen.size(), "parameters were removed");
            for (size_t i = 0; i < frozen.size() && i < now.size(); ++i)
            {
                expect(now[i].id == frozen[i].id,
                       "index " + std::to_string(i) + " is " + now[i].id + ", frozen " + frozen[i].id);
                expect(now[i].dest == frozen[i].dest, frozen[i].id + " destination " + std::to_string(now[i].dest) +
                                                          ", frozen " + std::to_string(frozen[i].dest));
            }
            // Appended parameters sit after the frozen ones, so their destinations follow the last frozen number.
            int lastDest = 0;
            for (const auto& e : frozen)
                lastDest = std::max(lastDest, e.dest);
            for (size_t i = frozen.size(); i < now.size(); ++i)
                expect(now[i].dest == 0 || now[i].dest > lastDest, now[i].id + " reuses a frozen destination");
        }

        beginTest("a module added later with the next append sequence moves no existing index or modulation number");
        {
            const auto base = ModuleRegistry::builder(ModuleRegistry::neutralLayout());
            base->finalize();
            const auto grown = ModuleRegistry::builder(ModuleRegistry::neutralLayout());
            grown->add(appendedModule("test.late_two", 5));
            grown->add(appendedModule("test.late_one", 4));
            grown->finalize();
            const auto& a = base->parameters();
            const auto& b = grown->parameters();
            expect(b.size() == a.size() + 4, "four new parameters");
            bool same = true;
            for (int i = 0; i < a.size(); ++i)
                same = same && a.params()[static_cast<size_t>(i)].id == b.params()[static_cast<size_t>(i)].id;
            expect(same, "every existing index keeps its parameter");
            expect(b.modTargets().size() == a.modTargets().size() + 4, "four new modulation targets");
            bool sameTargets = true;
            for (size_t k = 0; k < a.modTargets().size(); ++k)
                sameTargets = sameTargets && a.modTargets()[k] == b.modTargets()[k];
            expect(sameTargets, "every existing modulation number keeps its target");
            // Sequence beats registration order: late_one (4) lists before late_two (5) although it was added second.
            expect(b.indexOf("zz_late_one_a") == a.size() && b.indexOf("zz_late_two_a") == a.size() + 2,
                   "ordered by (sequence, registration)");
        }
    }
};

static ParamOrderTests paramOrderTests;
