#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <string>

// Signal cycle for one junction.
//
// The colour is a PURE FUNCTION of simulation time, not a countdown that has to
// be ticked. That matters for three reasons:
//
//   * Opposing approaches must be able to disagree. A junction runs `groups`
//     phase groups that take turns, so while group 0 is green every other group
//     is red. A single stored `state` can only say one thing at a time, so with
//     it a junction is either open to everyone or closed to everyone at once.
//   * Every junction must not switch at the same instant. `phase_offset` is
//     derived from the junction id, so the city's lights spread across the
//     cycle instead of blinking in unison.
//   * A stateless cycle cannot drift. Asking at any sim time gives the same
//     colour, so /api/state and the vehicle physics can never disagree about
//     what a light was showing.
//
// One group's slot is green -> amber -> all-red. The all-red clearance interval
// is what lets vehicles already inside the junction get out before the crossing
// stream is released.
class TrafficLight {
  private:
    double green_dur;     // seconds of green per group
    double amber_dur;     // seconds of amber per group
    double all_red_dur;   // clearance interval at the end of each group's slot
    int groups;           // number of phase groups taking turns
    double phase_offset;  // seconds, derived from the junction id

  public:
    // Colours, matching the numbers /api/state has always sent the frontend.
    enum State : unsigned int { Red = 0, Amber = 1, Green = 2 };

    TrafficLight();
    // Light loaded from the map file: green/amber/all-red durations and groups.
    TrafficLight(double green, double amber, double all_red, int groups = 2);

    // Read the phase durations (config), e.g. for saving the map or the UI.
    double get_green_duration() const { return green_dur; }
    double get_yellow_duration() const { return amber_dur; }
    double get_red_duration() const { return all_red_dur; }
    int get_groups() const { return groups; }
    void set_groups(int n);

    // Spread this junction across the cycle. Derived from the id, so reloading
    // the same map reproduces exactly the same timings.
    void set_phase_offset_from(const std::string& id);
    double get_phase_offset() const { return phase_offset; }

    // Colour shown to approach `group` at `sim_time`. A group outside
    // [0, groups) is red, so an approach that never got a group assigned fails
    // closed instead of being waved through.
    State state_at(int group, double sim_time) const;

    // Seconds until this group's colour changes next (UI countdown).
    double remaining_at(int group, double sim_time) const;

    // Behaviour for TrafficEvent: a broken light runs a shorter green.
    void set_green_duration(double time);
};

#endif
