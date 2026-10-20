// SPDX-License-Identifier: MIT
#pragma once

// Minimal self-registering unit test framework (no external dependencies). A test class derives from
// crucible::test::UnitTest, implements runTest() with beginTest()/expect...() calls and is instantiated as a static
// object; TestMain.cpp runs every registered test and returns non-zero on any failure.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

// Declares `Type& name` bound to a heap object that lives to the end of the scope. The big engine objects (SynthEngine
// 83 KB, EngineCore 231 KB, SeqBank 135 KB) must not be locals of a test body: MSVC gives every local its own stack
// slot and a runTest() with dozens of them needs megabytes, but a default Windows stack is 1 MB.
#define CRUCIBLE_HEAP(Type, name, ...)                                                                                      \
    auto name##CrucibleHeap_ = std::make_unique<Type>(__VA_ARGS__);                                                         \
    Type& name = *name##CrucibleHeap_

namespace crucible::test
{

// Message string that formats numbers on construction, so "x " + String(1.5f) reads naturally in expectations.
class String
{
public:
    String() = default;
    String(const char* s) : text(s != nullptr ? s : "") {}
    String(std::string s) : text(std::move(s)) {}
    template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0> explicit String(T v)
    {
        std::ostringstream os;
        os << v;
        text = os.str();
    }
    template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0> String(T v, int decimals)
    {
        std::ostringstream os;
        os << std::fixed << std::setprecision(decimals) << v;
        text = os.str();
    }

    const std::string& str() const { return text; }

    friend String operator+(const String& a, const String& b) { return String(a.text + b.text); }
    friend String operator+(const char* a, const String& b) { return String(std::string(a) + b.text); }
    friend String operator+(const String& a, const char* b) { return String(a.text + b); }
    friend String operator+(const std::string& a, const String& b) { return String(a + b.text); }
    friend String operator+(const String& a, const std::string& b) { return String(a.text + b); }
    String& operator+=(const String& b)
    {
        text += b.text;
        return *this;
    }

private:
    std::string text;
};

class UnitTest
{
public:
    UnitTest(std::string testName, std::string testCategory) : name(std::move(testName)), category(std::move(testCategory))
    {
        registry().push_back(this);
    }
    virtual ~UnitTest() = default;

    virtual void runTest() = 0;

    static std::vector<UnitTest*>& registry()
    {
        static std::vector<UnitTest*> tests;
        return tests;
    }

    // Runs one test class; returns the number of failed expectations.
    int execute()
    {
        failures = 0;
        passes = 0;
        runTest();
        endSection();
        return failures;
    }

    // True when CRUCIBLE_SKIP_CPU_TESTS is set (to anything but "" or "0"). Hosted CI runners share their cores, so a
    // CPU-budget or timing expectation is not meaningful there; local runs (tools/ci.sh) leave the variable unset and
    // keep enforcing every budget.
    static bool cpuChecksSkipped()
    {
        static const bool skipped = []
        {
            const char* v = std::getenv("CRUCIBLE_SKIP_CPU_TESTS");
            return v != nullptr && v[0] != '\0' && !(v[0] == '0' && v[1] == '\0');
        }();
        return skipped;
    }

    const std::string& getName() const { return name; }
    int failureCount() const { return failures; }
    int passCount() const { return passes; }

protected:
    void beginTest(const std::string& section)
    {
        endSection();
        current = section;
        std::printf("  %s / %s\n", name.c_str(), section.c_str());
    }

    void expect(bool ok, const String& message = {})
    {
        if (ok)
        {
            ++passes;
            return;
        }
        ++failures;
        std::printf("!!! FAILED: %s / %s%s%s\n", name.c_str(), current.c_str(), message.str().empty() ? "" : ": ",
                    message.str().c_str());
    }

    template <typename A, typename B> void expectEquals(const A& actual, const B& expected, const String& message = {})
    {
        std::ostringstream os;
        os << "expected " << expected << ", got " << actual;
        expect(actual == expected, message.str().empty() ? String(os.str()) : message + " (" + os.str() + ")");
    }

    template <typename A, typename B, typename C>
    void expectWithinAbsoluteError(const A& actual, const B& expected, const C& tolerance, const String& message = {})
    {
        const double diff = std::abs(static_cast<double>(actual) - static_cast<double>(expected));
        std::ostringstream os;
        os << "expected " << expected << " +/- " << tolerance << ", got " << actual;
        expect(diff <= static_cast<double>(tolerance),
               message.str().empty() ? String(os.str()) : message + " (" + os.str() + ")");
    }

    // An expectation about CPU cost or timing. Behaves like expect(), except under CRUCIBLE_SKIP_CPU_TESTS, where it
    // logs the outcome as a skip and never fails.
    void expectCpu(bool ok, const String& message = {})
    {
        if (!cpuChecksSkipped())
        {
            expect(ok, message);
            return;
        }
        std::printf("    skipped CPU check (CRUCIBLE_SKIP_CPU_TESTS), would %s: %s\n", ok ? "pass" : "fail",
                    message.str().c_str());
    }

    void logMessage(const String& message) { std::printf("    %s\n", message.str().c_str()); }

private:
    void endSection() {}

    std::string name;
    std::string category;
    std::string current;
    int failures = 0;
    int passes = 0;
};

} // namespace crucible::test
