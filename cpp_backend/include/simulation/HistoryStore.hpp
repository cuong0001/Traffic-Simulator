#pragma once
// Persistent history of the simulated world within a single run, captured
// once per REAL second so a client can scrub back through the last
// kWindowSeconds of traffic without the live simulation ever pausing for
// anyone. Written to disk as it happens (not just held in RAM) so a crash
// mid-run doesn't lose everything captured before it -- but reset() wipes
// whatever a PRIOR run left behind: every launch of the simulator starts
// its own playback history from empty, it never resumes an old one.
//
// Indexed by REAL capture time ("seconds ago from now"), not sim_time_ --
// sim_time_ resets to 0 on every TrafficSimulator::load(), so two different
// real moments could otherwise both claim "t = 120s" and collide. Eviction
// ("delete anything older than the window") is also inherently a real-time
// operation regardless of what the speed slider is doing. sim_time_ still
// rides along in every frame purely for DISPLAY -- what the scrubber's clock
// label shows -- and if the map got reloaded mid-window that label will
// visibly jump backward at the reload point. That is honest, not a bug.
//
// A frame carries exactly what TrafficSimulator::state_json() already emits
// (vehicles/lights/roads/events/stats), so a query result can be handed to
// the frontend's existing applyState() unchanged -- no parallel rendering
// path, no new shape for the client to learn.
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

struct HistoryVehicleRecord {
    std::string id;
    std::string road;
    float pos = 0.0f;
    float speed = 0.0f;
    int lane = 0;
    float lateral_offset = 0.0f;
    int recalcs = 0;
    int state = 0;
    std::string type;
};

// One approach's signal head. `id` is the approach ROAD's id, matching
// state_json()'s lights[].id (a junction shows different colours to
// different approaches, so it can't be keyed by junction alone). Carries the
// same geometry state_json() does (junction, group, stop distance, and the
// optional head position) even though it is static per road, because
// applyState() reads hx/hy/hux/huy straight off EVERY payload it receives
// (it has no separate "fetch geometry once" step) -- omitting it here would
// mean historical lights report a colour but never draw a head.
struct HistoryLightRecord {
    std::string id;
    std::string junction;    // "n": junction id
    int group = -1;          // "g": signal group
    float stop_from_end = 0.0f;  // "d"
    unsigned int state = 0;
    bool has_head = false;
    float hx = 0.0f, hy = 0.0f, hux = 0.0f, huy = 0.0f;
};

// Only roads worth mentioning -- state_json() itself only ever sends
// congested or closed roads, and history follows the same filter.
struct HistoryRoadRecord {
    std::string id;
    float congestion = 0.0f;
    bool closed = false;
};

struct HistoryEventRecord {
    std::string id;
    std::string type;
    std::string target;
    std::string description;
    double remaining = 0.0;
};

struct HistoryStatsRecord {
    double avg_congestion = 0.0;
    int active_vehicles = 0;
    double avg_travel_time = 0.0;
    int completed_trips = 0;   // cumulative, as of this instant
    int spawned = 0;           // cumulative, as of this instant
};

// One instant of the world. `roads` on a captured frame is always the FULL
// current list (state_json()'s own filtered view, congested-or-closed only);
// HistoryStore itself decides internally whether to persist that as a fresh
// baseline or diff it against the segment's running road state to store only
// what changed -- callers never need to think about deltas.
struct HistoryFrame {
    double real_time = 0.0;   // seconds since epoch -- the scrub axis
    double sim_time = 0.0;    // TrafficSimulator::sim_time_ at capture -- display only
    // A virtual calendar timestamp (seconds since epoch): real time when the
    // CURRENT run started (TrafficSimulator::load()) plus sim_time_ elapsed
    // since. It runs at real-world speed only while speed_ == 1 -- running
    // faster/slower makes it drift ahead of or behind the real clock, which
    // is the point: the scrubber is meant to show "the simulator's own
    // clock," not literally when the capture happened on the wall.
    double calendar_time = 0.0;
    std::vector<HistoryVehicleRecord> vehicles;
    std::vector<HistoryLightRecord> lights;
    std::vector<HistoryRoadRecord> roads;
    std::vector<HistoryEventRecord> events;
    HistoryStatsRecord stats;
};

// Not thread-safe on its own -- TrafficSimulator calls capture() from inside
// tick() with mtx_ already held, the same convention tick_statistics() uses,
// and the query path (at()/range()) is likewise only ever called with mtx_
// held by the caller. No lock of its own is needed because of that, same
// reasoning as JunctionArbiter/MapEditor being plain reference-held members
// rather than each protecting themselves.
class HistoryStore {
public:
    // `dir` is created if it doesn't exist.
    explicit HistoryStore(std::string dir, double window_seconds = 86400.0);

    // Deletes whatever segment files are already in `dir` -- left over from a
    // previous run of the simulator -- and starts with an empty index. Every
    // launch gets its own playback history; it never resumes a prior run's.
    // Call once at startup, before the first capture().
    void reset();

    // Appends one frame, rotating into a new hourly segment and evicting
    // anything that just aged out of the window as needed.
    void capture(const HistoryFrame& frame);

    struct Range {
        double oldest_ago = 0.0;     // seconds ago of the OLDEST retained frame
        double newest_ago = 0.0;     // seconds ago of the NEWEST retained frame (~0)
        double window_seconds = 0.0;
        bool empty = true;           // nothing captured yet
    };
    Range range() const;

    // Reconstructs the frame nearest `seconds_ago` (real time), applying any
    // road deltas on top of the most recent baseline at or before it.
    // Clamped to what's retained: ago <= newest_ago returns the newest frame,
    // ago >= oldest_ago returns the oldest. std::nullopt only if range().empty.
    std::optional<HistoryFrame> at(double seconds_ago) const;

    // JSON matching TrafficSimulator::state_json()'s shape, for the query
    // API to hand back verbatim. Empty vehicles/lights/etc if no frame exists.
    static std::string to_state_json(const HistoryFrame& frame);

private:
    struct IndexEntry {
        double real_time = 0.0;
        std::size_t segment = 0;   // index into segments_
        uint64_t byte_offset = 0;  // this frame's own position in the file
    };
    struct Segment {
        std::string path;
        double start_real_time = 0.0;
        double end_real_time = 0.0;
        uint64_t baseline_offset = 0;  // constant: right after the segment header
    };

    void rotate_segment_locked(double real_time);
    void evict_locked(double now_real_time);
    std::optional<HistoryFrame> reconstruct_locked(const IndexEntry& target) const;

    std::string dir_;
    double window_seconds_;
    uint32_t session_id_;

    std::vector<Segment> segments_;
    std::vector<IndexEntry> index_;   // sorted by real_time, one per captured frame
    // Monotonic per-session counter used in segment filenames. NOT the same as
    // segments_.size(): eviction removes from the front of segments_, so its
    // size alone can't be reused as a filename suffix without risking two
    // different segments claiming the same name within one process's run.
    std::size_t next_segment_index_ = 0;

    // Running "what does each road look like right now" for the CURRENT
    // segment, so capture() can diff the next frame against it. Reset (and
    // seeded fresh from the incoming frame) every time a new segment starts,
    // which is what makes each segment reconstructable on its own without
    // reading anything from a prior hour.
    std::vector<HistoryRoadRecord> last_roads_;
    double last_segment_start_ = -1.0;  // real_time a new segment began, or < 0 if none yet
};
