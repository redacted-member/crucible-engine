// SPDX-License-Identifier: MIT
#pragma once

// A loaded shared library (dlopen on POSIX, LoadLibrary on Windows). Internal to the SDK host.

#include <memory>
#include <string>

namespace crucible::sdk
{

class SharedLibrary
{
public:
    // path is UTF-8. Returns null with `error` set when the library cannot be opened (or when the build has module
    // loading switched off).
    static std::shared_ptr<SharedLibrary> open(const std::string& path, std::string& error);
    ~SharedLibrary();
    SharedLibrary(const SharedLibrary&) = delete;
    SharedLibrary& operator=(const SharedLibrary&) = delete;

    void* symbol(const char* name) const;
    const std::string& path() const { return libraryPath; }

private:
    SharedLibrary() = default;
    void* handle = nullptr;
    std::string libraryPath;
};

} // namespace crucible::sdk
