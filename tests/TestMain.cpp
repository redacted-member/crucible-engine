// SPDX-License-Identifier: MIT
#include "TestFramework.h"

#include <cstdio>
#include <cstring>

// Runs every registered test class (optionally only those whose name contains argv[1]).
int main(int argc, char** argv)
{
    int failures = 0;
    int passes = 0;
    for (auto* test : crucible::test::UnitTest::registry())
    {
        if (argc > 1 && std::strstr(test->getName().c_str(), argv[1]) == nullptr)
            continue;
        std::printf("%s\n", test->getName().c_str());
        failures += test->execute();
        passes += test->passCount();
    }
    std::printf("\n%d expectations passed, %d failed\n", passes, failures);
    return failures == 0 ? 0 : 1;
}
