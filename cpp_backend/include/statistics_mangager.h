#ifndef STATISTICS_MANAGER_H
#define STATISTICS_MANAGER_H
#include <map>
#include <string>

class statistics_manager {
private:
    std::map<std::string, double> metrics;
public:
    void record_metric(std::string key, double value);
    void report();
};

#endif
