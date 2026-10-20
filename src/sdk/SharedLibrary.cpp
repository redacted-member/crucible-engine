// SPDX-License-Identifier: MIT
#include "SharedLibrary.h"

#if defined(CRUCIBLE_NO_MODULE_LOADING)
// Module loading is compiled out.
namespace crucible::sdk
{
std::shared_ptr<SharedLibrary> SharedLibrary::open(const std::string&, std::string& error)
{
    error = "this build of the engine has shared-library module loading switched off";
    return nullptr;
}
SharedLibrary::~SharedLibrary() = default;
void* SharedLibrary::symbol(const char*) const
{
    return nullptr;
}
} // namespace crucible::sdk
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <string>
#include <vector>

namespace crucible::sdk
{
namespace
{
std::wstring widen(const std::string& utf8)
{
    if (utf8.empty())
        return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), nullptr, 0);
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), w.data(), n);
    return w;
}

// The full path of `path` (relative paths resolve against the current directory), or an empty string.
std::wstring absolute(const std::wstring& path)
{
    DWORD n = GetFullPathNameW(path.c_str(), 0, nullptr, nullptr);
    if (n == 0)
        return {};
    std::vector<wchar_t> buffer(static_cast<size_t>(n));
    n = GetFullPathNameW(path.c_str(), static_cast<DWORD>(buffer.size()), buffer.data(), nullptr);
    return n == 0 || n >= buffer.size() ? std::wstring() : std::wstring(buffer.data(), n);
}
} // namespace

std::shared_ptr<SharedLibrary> SharedLibrary::open(const std::string& path, std::string& error)
{
    // An absolute path, loaded with the module's own directory and the default system directories as the only places
    // its dependencies are searched: never the current directory or %PATH%, where another program could plant a DLL.
    const std::wstring full = absolute(widen(path));
    if (full.empty())
    {
        error = path + ": cannot be resolved to a full path";
        return nullptr;
    }
    HMODULE h = LoadLibraryExW(full.c_str(), nullptr, LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
    if (h == nullptr)
    {
        error = path + ": cannot be loaded (error " + std::to_string(GetLastError()) + ")";
        return nullptr;
    }
    std::shared_ptr<SharedLibrary> lib(new SharedLibrary());
    lib->handle = h;
    lib->libraryPath = path;
    return lib;
}

SharedLibrary::~SharedLibrary()
{
    if (handle != nullptr)
        FreeLibrary(static_cast<HMODULE>(handle));
}

void* SharedLibrary::symbol(const char* name) const
{
    return handle != nullptr ? reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(handle), name)) : nullptr;
}
} // namespace crucible::sdk
#else
#include <dlfcn.h>

namespace crucible::sdk
{
std::shared_ptr<SharedLibrary> SharedLibrary::open(const std::string& path, std::string& error)
{
    void* h = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (h == nullptr)
    {
        const char* why = dlerror();
        error = path + ": cannot be loaded" + (why != nullptr ? std::string(" (") + why + ")" : std::string());
        return nullptr;
    }
    std::shared_ptr<SharedLibrary> lib(new SharedLibrary());
    lib->handle = h;
    lib->libraryPath = path;
    return lib;
}

SharedLibrary::~SharedLibrary()
{
    if (handle != nullptr)
        dlclose(handle);
}

void* SharedLibrary::symbol(const char* name) const
{
    return handle != nullptr ? dlsym(handle, name) : nullptr;
}
} // namespace crucible::sdk
#endif
