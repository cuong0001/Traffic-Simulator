#pragma once
#include <string>

// Display-only background region ("CITY CENTER", "PARK", ...).
// The map stores a zone as a polygon; we keep only its bounding box.
struct Zone {
    std::string name;
    std::string kind;
    double x = 0.0;  // top-left corner
    double y = 0.0;
    double w = 0.0;
    double h = 0.0;
};
