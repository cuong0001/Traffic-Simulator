#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

class TrafficLight {
  private:
    // Duration of each phase in seconds, indexed by state: [0]=red, [1]=yellow, [2]=green.
    double phase_dur[3];

    // Number of phase groups: approaches on the same axis share a group and go
    // green together.
    int groups;

    // --- Runtime state: changes every tick. ---
    unsigned int state;    // current color: 0=red, 1=yellow, 2=green
    double remaining_time; // seconds left in the current phase

  public:
    TrafficLight();
    // Light loaded from the map file: green/yellow/red durations and group count.
    TrafficLight(double green, double yellow, double red, int groups = 2);

    // Read the phase durations (config), e.g. for saving the map or the UI.
    double get_green_duration() const;
    double get_yellow_duration() const;
    double get_red_duration() const;
    int get_groups() const;

    // Read the live state.
    unsigned int get_state() const;      // 0=red, 1=yellow, 2=green
    double get_remaining_time() const;   // seconds left in the current phase

    // Advance the light by a time step dt (seconds): count down the current
    // phase and switch green -> yellow -> red -> green when it runs out.
    void update(double dt);
};

#endif
