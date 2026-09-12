// json_lite.cpp — see json_lite.h.

#include "core/json_lite.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace mac_shell {

namespace {

// Bounded so a hostile payload can't blow the stack on nesting alone.
constexpr int MAX_DEPTH = 16;

struct reader {
    const std::string &in;
    size_t i = 0;
    std::string error;

    explicit reader(const std::string &s) : in(s) {}

    void skip_ws() {
        while (i < in.size() && (in[i] == ' ' || in[i] == '\t' ||
                                 in[i] == '\n' || in[i] == '\r'))
            i++;
    }
    bool eat(char c) {
        skip_ws();
        if (i < in.size() && in[i] == c) {
            i++;
            return true;
        }
        return false;
    }
    bool fail(const char *why) {
        if (error.empty())
            error = why;
        return false;
    }
    bool literal(const char *word) {
        size_t n = 0;
        while (word[n])
            n++;
        if (in.compare(i, n, word) != 0)
            return false;
        i += n;
        return true;
    }
    bool parse_hex4(unsigned &out) {
        if (i + 4 > in.size())
            return false;
        out = 0;
        for (int k = 0; k < 4; k++) {
            char c = in[i + k];
            int d;
            if (c >= '0' && c <= '9') d = c - '0';
            else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
            else return false;
            out = out * 16 + (unsigned)d;
        }
        i += 4;
        return true;
    }
    // Encodes one code point as UTF-8. Lone surrogates become U+FFFD rather
    // than invalid bytes, so a bad \u escape can't produce a malformed title.
    static void push_utf8(unsigned cp, std::string &out) {
        if (cp >= 0xD800 && cp <= 0xDFFF)
            cp = 0xFFFD;
        if (cp < 0x80) {
            out += (char)cp;
        } else if (cp < 0x800) {
            out += (char)(0xC0 | (cp >> 6));
            out += (char)(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += (char)(0xE0 | (cp >> 12));
            out += (char)(0x80 | ((cp >> 6) & 0x3F));
            out += (char)(0x80 | (cp & 0x3F));
        } else {
            out += (char)(0xF0 | (cp >> 18));
            out += (char)(0x80 | ((cp >> 12) & 0x3F));
            out += (char)(0x80 | ((cp >> 6) & 0x3F));
            out += (char)(0x80 | (cp & 0x3F));
        }
    }
    bool parse_string(std::string &out) {
        if (!eat('"'))
            return fail("expected a string");
        out.clear();
        while (i < in.size()) {
            char c = in[i++];
            if (c == '"')
                return true;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (i >= in.size())
                break;
            char e = in[i++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned hi = 0;
                    if (!parse_hex4(hi))
                        return fail("bad \\u escape");
                    if (hi >= 0xD800 && hi <= 0xDBFF && i + 1 < in.size() &&
                        in[i] == '\\' && in[i + 1] == 'u') {
                        size_t save = i;
                        i += 2;
                        unsigned lo = 0;
                        if (parse_hex4(lo) && lo >= 0xDC00 && lo <= 0xDFFF)
                            hi = 0x10000 + ((hi - 0xD800) << 10) + (lo - 0xDC00);
                        else
                            i = save;
                    }
                    push_utf8(hi, out);
                    break;
                }
                default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }
    bool parse_value(json_value &v, int depth) {
        if (depth > MAX_DEPTH)
            return fail("too deeply nested");
        skip_ws();
        if (i >= in.size())
            return fail("empty input");
        char c = in[i];
        if (c == '{') {
            i++;
            v.type = json_value::kind::object;
            skip_ws();
            if (eat('}'))
                return true;
            while (true) {
                std::string key;
                skip_ws();
                if (!parse_string(key))
                    return false;
                if (!eat(':'))
                    return fail("expected ':'");
                json_value child;
                if (!parse_value(child, depth + 1))
                    return false;
                v.keys.emplace_back(std::move(key), std::move(child));
                if (eat(','))
                    continue;
                if (eat('}'))
                    return true;
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            i++;
            v.type = json_value::kind::array;
            skip_ws();
            if (eat(']'))
                return true;
            while (true) {
                json_value child;
                if (!parse_value(child, depth + 1))
                    return false;
                v.items.push_back(std::move(child));
                if (eat(','))
                    continue;
                if (eat(']'))
                    return true;
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') {
            v.type = json_value::kind::string;
            return parse_string(v.str);
        }
        if (literal("true")) {
            v.type = json_value::kind::boolean;
            v.boolean = true;
            return true;
        }
        if (literal("false")) {
            v.type = json_value::kind::boolean;
            v.boolean = false;
            return true;
        }
        if (literal("null")) {
            v.type = json_value::kind::null;
            return true;
        }
        const char *start = in.c_str() + i;
        char *end = nullptr;
        double d = std::strtod(start, &end);
        if (end == start || !std::isfinite(d))
            return fail("expected a value");
        i += (size_t)(end - start);
        v.type = json_value::kind::number;
        v.number = d;
        return true;
    }
};

}  // namespace

const json_value *json_value::find(const std::string &key) const {
    if (type != kind::object)
        return nullptr;
    for (const auto &kv : keys)
        if (kv.first == key)
            return &kv.second;
    return nullptr;
}

bool json_value::floats(float *out, int n) const {
    if (type != kind::array || (int)items.size() != n)
        return false;
    for (int i = 0; i < n; i++) {
        if (!items[(size_t)i].is_number())
            return false;
        out[i] = (float)items[(size_t)i].number;
    }
    return true;
}

bool json_parse(const std::string &in, json_value &out, std::string &error) {
    reader r(in);
    if (!r.parse_value(out, 0)) {
        error = r.error.empty() ? "malformed json" : r.error;
        return false;
    }
    r.skip_ws();
    if (r.i != in.size()) {
        error = "trailing characters after the json value";
        return false;
    }
    return true;
}

void json_escape(const std::string &in, std::string &out) {
    for (char c : in) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if ((unsigned char)c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
}

}  // namespace mac_shell
