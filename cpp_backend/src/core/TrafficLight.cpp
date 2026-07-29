#include "core/TrafficLight.hpp"

// Default cycle (seconds), matching the map file's default values.
static const double DEFAULT_GREEN  = 25.0;
static const double DEFAULT_YELLOW = 3.0;
static const double DEFAULT_RED    = 2.0;

TrafficLight::TrafficLight()
    : TrafficLight(DEFAULT_GREEN, DEFAULT_YELLOW, DEFAULT_RED, 2) {}

TrafficLight::TrafficLight(double green, double yellow, double red, int groups)
    : phase_dur{red, yellow, green},  // index by state: [0]=red, [1]=yellow, [2]=green
      groups(groups),
      state(2),               // start in the green phase
      remaining_time(green) {}

double TrafficLight::get_green_duration() const { return phase_dur[2]; }
double TrafficLight::get_yellow_duration() const { return phase_dur[1]; }
double TrafficLight::get_red_duration() const { return phase_dur[0]; }
int TrafficLight::get_groups() const { return groups; }

unsigned int TrafficLight::get_state() const { return state; }
double TrafficLight::get_remaining_time() const { return remaining_time; }

// Count down the current phase; when it runs out, switch to the next color and
// reload its duration. The while loop + "+=" handles a dt larger than the phase
// (e.g. a big time step) without drifting off the cycle.
void TrafficLight::update(double dt) {
    const double cycle = phase_dur[0] + phase_dur[1] + phase_dur[2];
    if (cycle <= 0.0) return;  // misconfigured light: nothing to advance

    remaining_time -= dt;
    while (remaining_time <= 0.0) {
        state = (state == 2) ? 1 : (state == 1) ? 0 : 2;  // green -> yellow -> red -> green
        remaining_time += phase_dur[state];
    }
}

void TrafficLight::set_green_duration (double time)
{
    phase_dur[2] = time;
    //The light failure event takes effect immediately, without waiting for the current green light cycle to finish.
    if (state == 2 && remaining_time > time) {
        remaining_time = time;
    }
}
