#include "../../include/TrafficLight.h"

// Default cycle (seconds), matching the map file's default values.
static const double DEFAULT_GREEN  = 25.0;
static const double DEFAULT_YELLOW = 3.0;
static const double DEFAULT_RED    = 2.0;

TrafficLight::TrafficLight()
    : TrafficLight(DEFAULT_GREEN, DEFAULT_YELLOW, DEFAULT_RED, 2) {}

TrafficLight::TrafficLight(double green, double yellow, double red, int groups)
    : yellow(yellow), green(green), red(red), groups(groups),
      state(2), // start in the green phase
      remaining_yellow_time(yellow),
      remaining_red_time(red),
      remaining_green_time(green),
      coordinate(0, 0) {}

double TrafficLight::get_green_duration() const { return green; }
double TrafficLight::get_yellow_duration() const { return yellow; }
double TrafficLight::get_red_duration() const { return red; }
int TrafficLight::get_groups() const { return groups; }
unsigned int TrafficLight::get_state() const { return state; }

double TrafficLight::get_remaining_yellow() { return remaining_yellow_time; }
double TrafficLight::get_remaining_green() { return remaining_green_time; }
double TrafficLight::get_remaining_red() { return remaining_red_time; }

// Advance the light state by a time step dt (seconds)
// TODO: decrease remaining_*_time by dt and switch green/yellow/red when a phase ends
void TrafficLight::update(double dt) {
    (void)dt;
}
