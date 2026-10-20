// SPDX-License-Identifier: MIT
#include <crucible/crucible_module.h>

#include "CModuleRegistry.h"

#include <algorithm>
#include <cstring>
#include <new>
#include <string>

// The host-side C functions of crucible_module.h. Exceptions never cross this boundary.

extern "C"
{

    crucible_module_registry* crucible_module_registry_create(void)
    {
        try
        {
            return new crucible_module_registry();
        }
        catch (...)
        {
            return nullptr;
        }
    }

    void crucible_module_registry_destroy(crucible_module_registry* registry)
    {
        delete registry;
    }

    void crucible_module_registry_allow_libraries(crucible_module_registry* registry, int allow)
    {
        if (registry != nullptr)
            registry->set.allowLibraries(allow != 0);
    }

    int crucible_register_module(crucible_module_registry* registry, const crucible_module_descriptor* descriptor)
    {
        if (registry == nullptr)
            return CRUCIBLE_MODULE_ERR_ARGUMENT;
        try
        {
            return registry->set.add(descriptor);
        }
        catch (const std::exception&)
        {
            return CRUCIBLE_MODULE_ERR_INVALID;
        }
    }

    int crucible_load_module_library(crucible_module_registry* registry, const char* path)
    {
        if (registry == nullptr || path == nullptr)
            return CRUCIBLE_MODULE_ERR_ARGUMENT;
        try
        {
            return registry->set.loadLibrary(path);
        }
        catch (const std::exception&)
        {
            return CRUCIBLE_MODULE_ERR_LOAD;
        }
    }

    int crucible_module_registry_error(const crucible_module_registry* registry, char* buffer, size_t buffer_size)
    {
        const std::string text = registry != nullptr ? registry->set.error() : std::string();
        if (buffer != nullptr && buffer_size > 0)
        {
            const size_t n = std::min(buffer_size - 1, text.size());
            std::memcpy(buffer, text.data(), n);
            buffer[n] = '\0';
        }
        return static_cast<int>(text.size());
    }

    int crucible_module_registry_count(const crucible_module_registry* registry)
    {
        return registry != nullptr ? registry->set.count() : 0;
    }

    const char* crucible_module_registry_id(const crucible_module_registry* registry, int k)
    {
        // The ids live as long as the registry, in registration order.
        if (registry == nullptr || k < 0 || k >= registry->set.count())
            return nullptr;
        return registry->set.idOf(k).c_str();
    }

} // extern "C"
