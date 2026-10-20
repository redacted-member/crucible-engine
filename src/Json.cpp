// SPDX-License-Identifier: MIT
#include <crucible/Json.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <locale>
#include <sstream>

namespace crucible::json
{
namespace
{
const Value& nullValue()
{
    static const Value v;
    return v;
}

void escape(std::string& out, const std::string& s)
{
    out.push_back('"');
    for (const char ch : s)
    {
        const auto c = static_cast<unsigned char>(ch);
        switch (c)
        {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (c < 0x20)
            {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            }
            else
                out.push_back(ch);
        }
    }
    out.push_back('"');
}

std::string formatNumber(double n)
{
    if (!std::isfinite(n))
        return "0";
    if (n == std::floor(n) && std::abs(n) < 1.0e15)
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.0f", n);
        return buf;
    }
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os.precision(9);
    os << n;
    return os.str();
}

struct Parser
{
    const std::string& s;
    std::size_t i = 0;
    std::string error;

    void ws()
    {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r'))
            ++i;
    }

    bool fail(const char* what)
    {
        if (error.empty())
            error = std::string(what) + " at byte " + std::to_string(i);
        return false;
    }

    bool literal(const char* word)
    {
        const std::size_t n = std::strlen(word);
        if (s.compare(i, n, word) != 0)
            return fail("invalid literal");
        i += n;
        return true;
    }

    static void utf8(std::string& out, std::uint32_t cp)
    {
        if (cp < 0x80)
            out.push_back(static_cast<char>(cp));
        else if (cp < 0x800)
        {
            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
        else if (cp < 0x10000)
        {
            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
        else
        {
            out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
    }

    bool hex4(std::uint32_t& v)
    {
        if (i + 4 > s.size())
            return fail("short \\u escape");
        v = 0;
        for (int k = 0; k < 4; ++k)
        {
            const char c = s[i++];
            v <<= 4;
            if (c >= '0' && c <= '9')
                v |= static_cast<std::uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f')
                v |= static_cast<std::uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F')
                v |= static_cast<std::uint32_t>(c - 'A' + 10);
            else
                return fail("bad \\u escape");
        }
        return true;
    }

    bool string(std::string& out)
    {
        if (s[i] != '"')
            return fail("expected string");
        ++i;
        while (i < s.size() && s[i] != '"')
        {
            char c = s[i++];
            if (c != '\\')
            {
                out.push_back(c);
                continue;
            }
            if (i >= s.size())
                return fail("bad escape");
            c = s[i++];
            switch (c)
            {
            case '"':
                out.push_back('"');
                break;
            case '\\':
                out.push_back('\\');
                break;
            case '/':
                out.push_back('/');
                break;
            case 'b':
                out.push_back('\b');
                break;
            case 'f':
                out.push_back('\f');
                break;
            case 'n':
                out.push_back('\n');
                break;
            case 'r':
                out.push_back('\r');
                break;
            case 't':
                out.push_back('\t');
                break;
            case 'u':
            {
                std::uint32_t cp;
                if (!hex4(cp))
                    return false;
                if (cp >= 0xD800 && cp < 0xDC00 && i + 6 <= s.size() && s[i] == '\\' && s[i + 1] == 'u')
                {
                    i += 2;
                    std::uint32_t lo;
                    if (!hex4(lo))
                        return false;
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                }
                utf8(out, cp);
                break;
            }
            default:
                return fail("bad escape");
            }
        }
        if (i >= s.size())
            return fail("unterminated string");
        ++i;
        return true;
    }

    bool value(Value& out, int depth)
    {
        if (depth > 200)
            return fail("nesting too deep");
        ws();
        if (i >= s.size())
            return fail("unexpected end");
        const char c = s[i];
        if (c == '{')
        {
            ++i;
            out = Value::object();
            ws();
            if (i < s.size() && s[i] == '}')
            {
                ++i;
                return true;
            }
            for (;;)
            {
                ws();
                std::string key;
                if (i >= s.size() || !string(key))
                    return fail("expected key");
                ws();
                if (i >= s.size() || s[i] != ':')
                    return fail("expected ':'");
                ++i;
                Value v;
                if (!value(v, depth + 1))
                    return false;
                out.set(key, std::move(v));
                ws();
                if (i < s.size() && s[i] == ',')
                {
                    ++i;
                    continue;
                }
                if (i < s.size() && s[i] == '}')
                {
                    ++i;
                    return true;
                }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[')
        {
            ++i;
            out = Value::array();
            ws();
            if (i < s.size() && s[i] == ']')
            {
                ++i;
                return true;
            }
            for (;;)
            {
                Value v;
                if (!value(v, depth + 1))
                    return false;
                out.push(std::move(v));
                ws();
                if (i < s.size() && s[i] == ',')
                {
                    ++i;
                    continue;
                }
                if (i < s.size() && s[i] == ']')
                {
                    ++i;
                    return true;
                }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"')
        {
            std::string str;
            if (!string(str))
                return false;
            out = Value(std::move(str));
            return true;
        }
        if (c == 't')
        {
            out = Value(true);
            return literal("true");
        }
        if (c == 'f')
        {
            out = Value(false);
            return literal("false");
        }
        if (c == 'n')
        {
            out = Value();
            return literal("null");
        }
        // Number.
        const std::size_t start = i;
        if (s[i] == '-')
            ++i;
        while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) != 0 || s[i] == '.' || s[i] == 'e' ||
                                s[i] == 'E' || s[i] == '+' || s[i] == '-'))
            ++i;
        if (i == start)
            return fail("unexpected character");
        std::istringstream is(s.substr(start, i - start));
        is.imbue(std::locale::classic());
        double d = 0.0;
        if (!(is >> d))
            return fail("bad number");
        out = Value(d);
        return true;
    }
};
} // namespace

const std::string& Value::asString() const
{
    static const std::string empty;
    return kind == Type::String ? str : empty;
}

const Value& Value::operator[](std::size_t i) const
{
    return kind == Type::Array && i < items.size() ? items[i] : nullValue();
}

Value& Value::push(Value v)
{
    if (kind != Type::Array)
    {
        *this = array();
    }
    items.push_back(std::move(v));
    return items.back();
}

bool Value::has(const std::string& key) const
{
    for (const auto& [k, v] : members)
        if (k == key)
            return true;
    return false;
}

const Value& Value::operator[](const std::string& key) const
{
    if (kind == Type::Object)
        for (const auto& [k, v] : members)
            if (k == key)
                return v;
    return nullValue();
}

Value& Value::set(const std::string& key, Value v)
{
    if (kind != Type::Object)
        *this = object();
    for (auto& [k, existing] : members)
        if (k == key)
        {
            existing = std::move(v);
            return existing;
        }
    members.emplace_back(key, std::move(v));
    return members.back().second;
}

void Value::write(std::string& out, int indent, int depth) const
{
    const auto newline = [&](int d)
    {
        if (indent < 0)
            return;
        out.push_back('\n');
        out.append(static_cast<size_t>(indent * d), ' ');
    };
    switch (kind)
    {
    case Type::Null:
        out += "null";
        break;
    case Type::Bool:
        out += boolean ? "true" : "false";
        break;
    case Type::Number:
        out += formatNumber(number);
        break;
    case Type::String:
        escape(out, str);
        break;
    case Type::Array:
        out.push_back('[');
        for (size_t i = 0; i < items.size(); ++i)
        {
            if (i > 0)
                out.push_back(',');
            newline(depth + 1);
            items[i].write(out, indent, depth + 1);
        }
        if (!items.empty())
            newline(depth);
        out.push_back(']');
        break;
    case Type::Object:
        out.push_back('{');
        for (size_t i = 0; i < members.size(); ++i)
        {
            if (i > 0)
                out.push_back(',');
            newline(depth + 1);
            escape(out, members[i].first);
            out += indent < 0 ? ":" : ": ";
            members[i].second.write(out, indent, depth + 1);
        }
        if (!members.empty())
            newline(depth);
        out.push_back('}');
        break;
    }
}

std::string Value::dump(int indent) const
{
    std::string out;
    write(out, indent, 0);
    return out;
}

bool parse(const std::string& text, Value& out, std::string& error)
{
    Parser p{text, 0, {}};
    Value v;
    if (!p.value(v, 0))
    {
        error = p.error;
        return false;
    }
    p.ws();
    if (p.i != text.size())
    {
        error = "trailing characters at byte " + std::to_string(p.i);
        return false;
    }
    out = std::move(v);
    return true;
}

namespace
{
constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
}

std::string base64Encode(const std::uint8_t* data, std::size_t size)
{
    std::string out;
    out.reserve((size + 2) / 3 * 4);
    for (std::size_t i = 0; i < size; i += 3)
    {
        const std::uint32_t b0 = data[i];
        const std::uint32_t b1 = i + 1 < size ? data[i + 1] : 0;
        const std::uint32_t b2 = i + 2 < size ? data[i + 2] : 0;
        const std::uint32_t triple = (b0 << 16) | (b1 << 8) | b2;
        out.push_back(kAlphabet[(triple >> 18) & 63]);
        out.push_back(kAlphabet[(triple >> 12) & 63]);
        out.push_back(i + 1 < size ? kAlphabet[(triple >> 6) & 63] : '=');
        out.push_back(i + 2 < size ? kAlphabet[triple & 63] : '=');
    }
    return out;
}

bool base64Decode(const std::string& text, std::vector<std::uint8_t>& out)
{
    out.clear();
    std::uint32_t acc = 0;
    int bits = 0;
    for (const char c : text)
    {
        if (c == '=' || c == '\n' || c == '\r' || c == ' ')
            continue;
        const char* p = std::strchr(kAlphabet, c);
        if (p == nullptr || c == '\0')
            return false;
        acc = (acc << 6) | static_cast<std::uint32_t>(p - kAlphabet);
        bits += 6;
        if (bits >= 8)
        {
            bits -= 8;
            out.push_back(static_cast<std::uint8_t>((acc >> bits) & 0xFFU));
        }
    }
    return true;
}

std::string floatsToBase64(const std::vector<float>& v)
{
    std::vector<std::uint8_t> bytes(v.size() * 4);
    for (size_t i = 0; i < v.size(); ++i)
    {
        std::uint32_t u;
        std::memcpy(&u, &v[i], 4);
        for (int b = 0; b < 4; ++b)
            bytes[i * 4 + static_cast<size_t>(b)] = static_cast<std::uint8_t>((u >> (8 * b)) & 0xffU);
    }
    return base64Encode(bytes.data(), bytes.size());
}

bool base64ToFloats(const std::string& text, std::vector<float>& out)
{
    std::vector<std::uint8_t> bytes;
    if (!base64Decode(text, bytes) || bytes.size() % 4 != 0)
        return false;
    out.resize(bytes.size() / 4);
    for (size_t i = 0; i < out.size(); ++i)
    {
        const std::uint32_t u =
            static_cast<std::uint32_t>(bytes[i * 4]) | (static_cast<std::uint32_t>(bytes[i * 4 + 1]) << 8) |
            (static_cast<std::uint32_t>(bytes[i * 4 + 2]) << 16) | (static_cast<std::uint32_t>(bytes[i * 4 + 3]) << 24);
        std::memcpy(&out[i], &u, 4);
    }
    return true;
}

} // namespace crucible::json
