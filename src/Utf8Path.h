// SPDX-License-Identifier: MIT
// Internal: UTF-8 <-> std::filesystem::path helpers.
// On MSVC a std::string path handed to std::ifstream / std::filesystem::path is interpreted in the ANSI code page, so
// folders with non-ASCII names fail. The engine API takes UTF-8 paths everywhere; these helpers make Windows treat them
// as such. On POSIX they are plain std::filesystem::path(s) / p.string(): behaviour unchanged.
#pragma once

#include <filesystem>
#include <string>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace crucible
{
inline std::filesystem::path utf8Path(const std::string& s)
{
#if defined(_WIN32)
    if (s.empty())
        return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    if (n <= 0)
        return std::filesystem::path(s);
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return std::filesystem::path(w);
#else
    return std::filesystem::path(s);
#endif
}

inline std::string pathToUtf8(const std::filesystem::path& p)
{
#if defined(_WIN32)
    const std::wstring w = p.wstring();
    if (w.empty())
        return {};
    const int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), nullptr, 0, nullptr, nullptr);
    if (n <= 0)
        return p.string();
    std::string s(static_cast<size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), s.data(), n, nullptr, nullptr);
    return s;
#else
    return p.string();
#endif
}
} // namespace crucible
