// SPDX-License-Identifier: MIT
// Internal to the command-line tools: UTF-8 arguments on every platform.
// On Windows main() receives argv in the ANSI code page, so a path such as "Ünal_日本/out.wav" arrives mangled. The tools
// construct a Utf8Args at the top of main(); it re-reads the command line as UTF-16 (GetCommandLineW +
// CommandLineToArgvW) and converts every argument to UTF-8, so the rest of the program sees exactly what it would on
// POSIX. On other platforms it passes argc / argv through unchanged. Files are then opened through crucible::utf8Path.
//
//   int main(int argc, char** argv)
//   {
//       const crucible::tools::Utf8Args args(argc, argv);
//       argc = args.argc;
//       argv = args.argv;
#pragma once

#include <string>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#endif

namespace crucible::tools
{
class Utf8Args
{
public:
    int argc;
    char** argv;

    Utf8Args(int argcIn, char** argvIn) : argc(argcIn), argv(argvIn)
    {
#if defined(_WIN32)
        int n = 0;
        wchar_t** wide = CommandLineToArgvW(GetCommandLineW(), &n);
        if (wide == nullptr)
            return;
        if (n == argcIn)
        {
            for (int i = 0; i < n; ++i)
                storage.push_back(toUtf8(wide[i]));
            for (auto& s : storage)
                pointers.push_back(s.data());
            pointers.push_back(nullptr);
            argc = n;
            argv = pointers.data();
        }
        LocalFree(wide);
#endif
    }
    Utf8Args(const Utf8Args&) = delete;
    Utf8Args& operator=(const Utf8Args&) = delete;

private:
#if defined(_WIN32)
    static std::string toUtf8(const wchar_t* w)
    {
        const int len = static_cast<int>(lstrlenW(w));
        if (len == 0)
            return {};
        const int n = WideCharToMultiByte(CP_UTF8, 0, w, len, nullptr, 0, nullptr, nullptr);
        std::string s(static_cast<size_t>(n > 0 ? n : 0), '\0');
        if (n > 0)
            WideCharToMultiByte(CP_UTF8, 0, w, len, s.data(), n, nullptr, nullptr);
        return s;
    }
    std::vector<std::string> storage;
    std::vector<char*> pointers;
#endif
};
} // namespace crucible::tools
