// json_lite.h — tiny recursive-descent JSON reader for the control plane.
//
// The control socket now takes JSON payloads (`note`, `note-update`) and the
// layout store reads back files it wrote, so the scene core needs a parser.
// Deliberately minimal: no streaming, no comments, no big-number games. Input
// is bounded by the control server's line limit and the layout file size.

#pragma once

#include <string>
#include <utility>
#include <vector>

namespace mac_shell {

struct json_value {
    enum class kind { null, boolean, number, string, array, object };

    kind type = kind::null;
    bool boolean = false;
    double number = 0.0;
    std::string str;
    std::vector<json_value> items;                        // array
    std::vector<std::pair<std::string, json_value>> keys; // object

    bool is_string() const { return type == kind::string; }
    bool is_number() const { return type == kind::number; }
    bool is_object() const { return type == kind::object; }
    bool is_array() const { return type == kind::array; }
    // Null for a missing key or a non-object.
    const json_value *find(const std::string &key) const;
    // Reads a [n]-long float array; false unless every element is a number.
    bool floats(float *out, int n) const;
};

// Parses one complete JSON value. On failure `error` gets a short reason.
bool json_parse(const std::string &in, json_value &out, std::string &error);

// Appends `in` to `out` with JSON string escaping (no surrounding quotes).
void json_escape(const std::string &in, std::string &out);

}  // namespace mac_shell
