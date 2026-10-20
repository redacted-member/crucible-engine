// SPDX-License-Identifier: MIT
#include "TestFramework.h"

#include <crucible/module/Module.h>

#include <cstdio>

// Runs the tests that need a non-default voice graph shape (5 oscillators, 4 filters). The shape has to be chosen
// before the parameter model is first used, which is why these tests are a separate executable.
int main()
{
    if (!crucible::ModuleRegistry::configureVoiceGraph({5, 4}))
    {
        std::printf("could not configure the voice graph\n");
        return 1;
    }
    int failures = 0;
    int passes = 0;
    for (auto* test : crucible::test::UnitTest::registry())
    {
        std::printf("%s\n", test->getName().c_str());
        failures += test->execute();
        passes += test->passCount();
    }
    std::printf("\n%d expectations passed, %d failed\n", passes, failures);
    return failures == 0 ? 0 : 1;
}
