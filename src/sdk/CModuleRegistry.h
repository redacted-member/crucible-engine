// SPDX-License-Identifier: MIT
#pragma once

// The object behind the opaque crucible_module_registry handle of crucible_module.h.

#include <crucible/sdk/Host.h>

struct crucible_module_registry
{
    crucible::sdk::ModuleSet set;
};
