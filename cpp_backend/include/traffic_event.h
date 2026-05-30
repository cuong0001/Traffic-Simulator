#ifndef TRAFFIC_EVENT_H
#define TRAFFIC_EVENT_H
#include <string>

struct traffic_event {
    std::string type; // Ví dụ: "Accident", "RoadWork"
    std::string location;
    int severity;
    void execute();
};

#endif
