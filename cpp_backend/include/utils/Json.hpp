/*
    - This is a simple helper for writing JSON by hand.
*/
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

namespace uts::json {

inline std::string escape(const std::string& in); // Encode a string for JSON: escape quotes, backslashes, control chars, etc.

inline std::string str(const std::string& s); // Wrap a string in quotes and escape it for JSON.

inline std::string num(double v, int precision = 2); // Format a number for JSON with the given precision.

}