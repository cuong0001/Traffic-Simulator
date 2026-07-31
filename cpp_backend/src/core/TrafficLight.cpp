#include "core/TrafficLight.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

// Default cycle (seconds), matching the map file's default values.
static const double DEFAULT_GREEN   = 25.0;
static const double DEFAULT_AMBER   = 3.0;
static const double DEFAULT_ALL_RED = 2.0;

TrafficLight::TrafficLight()
    : TrafficLight(DEFAULT_GREEN, DEFAULT_AMBER, DEFAULT_ALL_RED, 2) {}

TrafficLight::TrafficLight(double green, double amber, double all_red, int groups)
    : green_dur(std::max(1.0, green)),
      amber_dur(std::max(0.0, amber)),
      all_red_dur(std::max(0.0, all_red)),
      groups(std::clamp(groups, 1, 4)),
      phase_offset(0.0) {}

void TrafficLight::set_groups(int n) { groups = std::clamp(n, 1, 4); }

// Hash the id into an offset spread over one full minute. Using the id rather
// than a counter or a random draw means a saved map reloads with the identical
// timings, which is what makes a bug at t = 137 s reproducible.
void TrafficLight::set_phase_offset_from(const std::string& id) {
    const std::size_t h = std::hash<std::string>{}(id);
    phase_offset = static_cast<double>(h % 1000) / 1000.0 * 60.0;
}

TrafficLight::State TrafficLight::state_at(int group, double sim_time) const {
    if (group < 0 || group >= groups) return Red;  // unassigned: fail closed

    const double slot = green_dur + amber_dur + all_red_dur;
    const double cycle = slot * groups;
    if (cycle <= 0.0) return Green;  // misconfigured: do not trap anyone

    double t = std::fmod(sim_time + phase_offset, cycle);
    if (t < 0.0) t += cycle;

    // Which group owns the cycle right now. min() guards the case where fmod
    // returns exactly `cycle` through rounding.
    const int active = std::min(groups - 1, static_cast<int>(t / slot));
    if (active != group) return Red;

    const double u = t - active * slot;
    if (u < green_dur) return Green;
    if (u < green_dur + amber_dur) return Amber;
    return Red;  // clearance interval: everyone is red
}

double TrafficLight::remaining_at(int group, double sim_time) const {
    if (group < 0 || group >= groups) return 0.0;

    const double slot = green_dur + amber_dur + all_red_dur;
    const double cycle = slot * groups;
    if (cycle <= 0.0) return 0.0;

    double t = std::fmod(sim_time + phase_offset, cycle);
    if (t < 0.0) t += cycle;

    const double mine = group * slot;          // start of this group's slot
    const double u = t - mine;
    if (u < 0.0) return -u;                    // waiting for our slot to open
    if (u < green_dur) return green_dur - u;
    if (u < green_dur + amber_dur) return green_dur + amber_dur - u;
    if (u < slot) return slot - u;
    return cycle - u + mine;                   // our slot already passed
}

void TrafficLight::set_green_duration(double time) {
    // A light-failure event takes effect immediately. With a stateless cycle
    // that is automatic: the next state_at() already uses the new duration,
    // so there is no current phase left over to cut short.
    green_dur = std::max(1.0, time);
}
