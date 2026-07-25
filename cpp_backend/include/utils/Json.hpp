/*
    - This is a simple helper for writing JSON by hand.
    - Note: We deliberately use this hand-rolled JSON helper here instead of 
      the vendored nlohmann/json library. This is to avoid pulling a heavy 
      header into a hot path, keeping serialization fast and lightweight.
*/
#pragma once

#include <string>

namespace json {

std::string escape(const std::string& in); // Encode a string for JSON: escape quotes, backslashes, control chars, etc.

std::string str(const std::string& s); // Wrap a string in quotes and escape it for JSON.

std::string num(double v, int precision); // Format a number for JSON with the given precision.

}