#include "utils/Json.hpp"

#include <cmath>
#include <cstdio>

using namespace std;

string json::escape(const string& in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

string json::str(const std::string& s) 
{ 
    return "\"" + escape(s) + "\""; 
}

string json::num(double v, int precision) 
{
    if (!std::isfinite(v)) return "null";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", precision, v);
    return std::string(buf);
}
