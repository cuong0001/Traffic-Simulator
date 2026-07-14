#pragma once
#include <string>

// Vùng nền chỉ dùng để hiển thị ("CITY CENTER", "PARK", ...).
// File map lưu vùng dưới dạng polygon; ta chỉ giữ hình chữ nhật bao của nó.
struct Zone {
    std::string name;
    std::string kind;
    double x = 0.0;  // góc trái-trên
    double y = 0.0;
    double w = 0.0;
    double h = 0.0;
};
