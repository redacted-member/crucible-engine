// SPDX-License-Identifier: MIT
#pragma once

// Minimal JSON value, parser and writer (UTF-8 passed through, \uXXXX escapes decoded to UTF-8). Used for patch
// files and tool output; not for the audio thread.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace crucible::json
{

class Value
{
public:
    enum class Type
    {
        Null,
        Bool,
        Number,
        String,
        Array,
        Object
    };

    Value() = default;
    Value(std::nullptr_t) {}
    Value(bool b) : kind(Type::Bool), boolean(b) {}
    Value(double n) : kind(Type::Number), number(n) {}
    Value(float n) : kind(Type::Number), number(n) {}
    Value(int n) : kind(Type::Number), number(n) {}
    Value(std::int64_t n) : kind(Type::Number), number(static_cast<double>(n)) {}
    Value(const char* s) : kind(Type::String), str(s) {}
    Value(std::string s) : kind(Type::String), str(std::move(s)) {}

    static Value array()
    {
        Value v;
        v.kind = Type::Array;
        return v;
    }
    static Value object()
    {
        Value v;
        v.kind = Type::Object;
        return v;
    }

    Type type() const { return kind; }
    bool isNull() const { return kind == Type::Null; }
    bool isBool() const { return kind == Type::Bool; }
    bool isNumber() const { return kind == Type::Number; }
    bool isString() const { return kind == Type::String; }
    bool isArray() const { return kind == Type::Array; }
    bool isObject() const { return kind == Type::Object; }

    bool asBool(bool fallback = false) const { return kind == Type::Bool ? boolean : fallback; }
    double asNumber(double fallback = 0.0) const { return kind == Type::Number ? number : fallback; }
    const std::string& asString() const;
    std::string asString(const std::string& fallback) const { return kind == Type::String ? str : fallback; }

    // Arrays.
    std::size_t size() const { return kind == Type::Array ? items.size() : (kind == Type::Object ? members.size() : 0); }
    const Value& operator[](std::size_t i) const;
    Value& push(Value v);
    const std::vector<Value>& elements() const { return items; }

    // Objects (keys keep insertion order).
    bool has(const std::string& key) const;
    const Value& operator[](const std::string& key) const; // Null when missing
    Value& set(const std::string& key, Value v);
    const std::vector<std::pair<std::string, Value>>& entries() const { return members; }

    // indent < 0: compact; otherwise pretty-printed with that many spaces per level.
    std::string dump(int indent = 2) const;

private:
    void write(std::string& out, int indent, int depth) const;

    Type kind = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string str;
    std::vector<Value> items;
    std::vector<std::pair<std::string, Value>> members;
};

// Returns false and sets error (with the byte offset) on malformed input.
bool parse(const std::string& text, Value& out, std::string& error);

// Base64 (RFC 4648) for embedding binary data such as wavetables in patches.
std::string base64Encode(const std::uint8_t* data, std::size_t size);
bool base64Decode(const std::string& text, std::vector<std::uint8_t>& out);
// Little-endian float32 arrays as base64 (wavetable frames, sample audio). Decode fails on a bad length.
std::string floatsToBase64(const std::vector<float>& values);
bool base64ToFloats(const std::string& text, std::vector<float>& out);

} // namespace crucible::json
