#include "simulation/HistoryStore.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

namespace fs = std::filesystem;

namespace {
constexpr uint32_t kMagic = 0x54484953;  // "THIS" (traffic HIStory), a sentinel
constexpr uint16_t kVersion = 2;  // 2: FrameHeader carries calendar_time
constexpr double kSegmentSpan = 3600.0;  // rotate into a new file every real hour

double now_epoch_seconds() {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// ---- flat byte-buffer writer/reader ----------------------------------------
// Hand-rolled rather than pulling in a serialization library, matching the
// project's existing convention (VehicleSnapshotBin, BinarySnapshotHeader):
// this format has exactly one reader and one writer, both in this file.
void put_u8(std::vector<uint8_t>& b, uint8_t v) { b.push_back(v); }
void put_u16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v & 0xFF));
    b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}
void put_u32(std::vector<uint8_t>& b, uint32_t v) {
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}
void put_i32(std::vector<uint8_t>& b, int32_t v) { put_u32(b, static_cast<uint32_t>(v)); }
void put_f32(std::vector<uint8_t>& b, float v) {
    uint32_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    put_u32(b, bits);
}
void put_f64(std::vector<uint8_t>& b, double v) {
    uint64_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    for (int i = 0; i < 8; ++i) b.push_back(static_cast<uint8_t>((bits >> (8 * i)) & 0xFF));
}
void put_str(std::vector<uint8_t>& b, const std::string& s) {
    const uint16_t len = static_cast<uint16_t>(std::min<size_t>(s.size(), 0xFFFFu));
    put_u16(b, len);
    b.insert(b.end(), s.begin(), s.begin() + len);
}

struct Reader {
    const uint8_t* p;
    size_t n;
    size_t pos = 0;
    uint8_t u8() { return p[pos++]; }
    uint16_t u16() {
        uint16_t v = static_cast<uint16_t>(p[pos] | (p[pos + 1] << 8));
        pos += 2;
        return v;
    }
    uint32_t u32() {
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(p[pos + i]) << (8 * i);
        pos += 4;
        return v;
    }
    int32_t i32() { return static_cast<int32_t>(u32()); }
    float f32() {
        uint32_t bits = u32();
        float v;
        std::memcpy(&v, &bits, sizeof(v));
        return v;
    }
    double f64() {
        uint64_t bits = 0;
        for (int i = 0; i < 8; ++i) bits |= static_cast<uint64_t>(p[pos + i]) << (8 * i);
        pos += 8;
        double v;
        std::memcpy(&v, &bits, sizeof(v));
        return v;
    }
    std::string str() {
        uint16_t len = u16();
        std::string s(reinterpret_cast<const char*>(p + pos), len);
        pos += len;
        return s;
    }
};

#pragma pack(push, 1)
struct SegmentFileHeader {
    uint32_t magic;
    uint16_t version;
    uint32_t session_id;
};
struct FrameHeader {
    double real_time;
    double sim_time;
    double calendar_time;
    uint32_t payload_bytes;  // everything after this header, so a reader can
                              // skip a frame without parsing it
    uint16_t vehicle_count;
    uint16_t light_count;
    uint16_t road_count;     // roads present in THIS frame -- all of them for
                              // a baseline, only the changed ones for a delta
    uint16_t event_count;
    uint8_t is_baseline;
};
#pragma pack(pop)

void write_frame_payload(std::vector<uint8_t>& out, const HistoryFrame& f,
                          const std::vector<HistoryRoadRecord>& roads_to_write) {
    for (const auto& v : f.vehicles) {
        put_str(out, v.id);
        put_str(out, v.road);
        put_f32(out, v.pos);
        put_f32(out, v.speed);
        put_i32(out, v.lane);
        put_f32(out, v.lateral_offset);
        put_i32(out, v.recalcs);
        put_i32(out, v.state);
        put_str(out, v.type);
    }
    for (const auto& l : f.lights) {
        put_str(out, l.id);
        put_str(out, l.junction);
        put_i32(out, l.group);
        put_f32(out, l.stop_from_end);
        put_u32(out, l.state);
        put_u8(out, l.has_head ? 1 : 0);
        put_f32(out, l.hx);
        put_f32(out, l.hy);
        put_f32(out, l.hux);
        put_f32(out, l.huy);
    }
    for (const auto& r : roads_to_write) {
        put_str(out, r.id);
        put_f32(out, r.congestion);
        put_u8(out, r.closed ? 1 : 0);
    }
    for (const auto& e : f.events) {
        put_str(out, e.id);
        put_str(out, e.type);
        put_str(out, e.target);
        put_str(out, e.description);
        put_f64(out, e.remaining);
    }
    put_f64(out, f.stats.avg_congestion);
    put_i32(out, f.stats.active_vehicles);
    put_f64(out, f.stats.avg_travel_time);
    put_i32(out, f.stats.completed_trips);
    put_i32(out, f.stats.spawned);
}

// What one frame's payload decodes to, BEFORE deltas are merged onto a
// running road map -- `roads` here is exactly what that frame wrote on disk.
struct ParsedFrame {
    std::vector<HistoryVehicleRecord> vehicles;
    std::vector<HistoryLightRecord> lights;
    std::vector<HistoryRoadRecord> roads;
    std::vector<HistoryEventRecord> events;
    HistoryStatsRecord stats;
};

ParsedFrame read_frame_payload(const std::vector<uint8_t>& buf, const FrameHeader& h) {
    Reader r{buf.data(), buf.size()};
    ParsedFrame out;
    out.vehicles.reserve(h.vehicle_count);
    for (uint16_t i = 0; i < h.vehicle_count; ++i) {
        HistoryVehicleRecord v;
        v.id = r.str();
        v.road = r.str();
        v.pos = r.f32();
        v.speed = r.f32();
        v.lane = r.i32();
        v.lateral_offset = r.f32();
        v.recalcs = r.i32();
        v.state = r.i32();
        v.type = r.str();
        out.vehicles.push_back(std::move(v));
    }
    out.lights.reserve(h.light_count);
    for (uint16_t i = 0; i < h.light_count; ++i) {
        HistoryLightRecord l;
        l.id = r.str();
        l.junction = r.str();
        l.group = r.i32();
        l.stop_from_end = r.f32();
        l.state = r.u32();
        l.has_head = r.u8() != 0;
        l.hx = r.f32();
        l.hy = r.f32();
        l.hux = r.f32();
        l.huy = r.f32();
        out.lights.push_back(std::move(l));
    }
    out.roads.reserve(h.road_count);
    for (uint16_t i = 0; i < h.road_count; ++i) {
        HistoryRoadRecord rr;
        rr.id = r.str();
        rr.congestion = r.f32();
        rr.closed = r.u8() != 0;
        out.roads.push_back(std::move(rr));
    }
    out.events.reserve(h.event_count);
    for (uint16_t i = 0; i < h.event_count; ++i) {
        HistoryEventRecord e;
        e.id = r.str();
        e.type = r.str();
        e.target = r.str();
        e.description = r.str();
        e.remaining = r.f64();
        out.events.push_back(std::move(e));
    }
    out.stats.avg_congestion = r.f64();
    out.stats.active_vehicles = r.i32();
    out.stats.avg_travel_time = r.f64();
    out.stats.completed_trips = r.i32();
    out.stats.spawned = r.i32();
    return out;
}

std::string jstr(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    out += "\"";
    return out;
}
}  // namespace

HistoryStore::HistoryStore(std::string dir, double window_seconds)
    : dir_(std::move(dir)),
      window_seconds_(window_seconds),
      session_id_(static_cast<uint32_t>(now_epoch_seconds())) {
    std::error_code ec;
    fs::create_directories(dir_, ec);
}

void HistoryStore::load() {
    segments_.clear();
    index_.clear();
    next_segment_index_ = 0;
    last_segment_start_ = -1.0;
    last_roads_.clear();

    std::error_code ec;
    if (!fs::exists(dir_, ec)) return;

    const double cutoff = now_epoch_seconds() - window_seconds_;

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(dir_, ec)) {
        if (ec) break;
        if (entry.is_regular_file() && entry.path().extension() == ".bin") {
            files.push_back(entry.path());
        }
    }
    // Filenames are "hist-<session>-<segment>.bin" with a fixed-width-free
    // decimal session id derived from boot time, so lexical order already
    // matches chronological order across restarts.
    std::sort(files.begin(), files.end());

    for (const auto& path : files) {
        std::ifstream in(path, std::ios::binary);
        if (!in) continue;
        SegmentFileHeader sh{};
        in.read(reinterpret_cast<char*>(&sh), sizeof(sh));
        // A version mismatch means the on-disk FrameHeader layout doesn't
        // match this build's struct -- reading it as-is would misalign every
        // field after the point they diverged. Skip it rather than corrupt
        // the read; it's about to be pruned as stale on the next boot anyway
        // if it's actually old, since the window rolls forward regardless.
        if (!in || sh.magic != kMagic || sh.version != kVersion) continue;

        Segment seg;
        seg.path = path.string();
        seg.baseline_offset = static_cast<uint64_t>(in.tellg());

        std::vector<IndexEntry> found;
        bool any = false;
        while (true) {
            const uint64_t frame_offset = static_cast<uint64_t>(in.tellg());
            FrameHeader fh{};
            in.read(reinterpret_cast<char*>(&fh), sizeof(fh));
            if (!in || static_cast<size_t>(in.gcount()) != sizeof(fh)) break;
            if (!any) {
                seg.start_real_time = fh.real_time;
                any = true;
            }
            seg.end_real_time = fh.real_time;
            found.push_back({fh.real_time, segments_.size(), frame_offset});
            in.seekg(static_cast<std::streamoff>(fh.payload_bytes), std::ios::cur);
            if (!in) break;
        }
        if (!any) continue;  // empty/corrupt segment file -- ignore it

        if (seg.end_real_time < cutoff) {
            // Fully stale: a server that was down longer than the window
            // starts clean rather than loading files it would immediately
            // evict anyway.
            std::error_code rm_ec;
            fs::remove(path, rm_ec);
            continue;
        }

        for (auto& e : found) index_.push_back(e);
        segments_.push_back(std::move(seg));
    }

    std::sort(index_.begin(), index_.end(),
              [](const IndexEntry& a, const IndexEntry& b) { return a.real_time < b.real_time; });

    if (!segments_.empty()) {
        last_segment_start_ = segments_.back().start_real_time;
        next_segment_index_ = segments_.size();
        if (!index_.empty()) {
            // Reconstruct the newest frame so capture() keeps diffing roads
            // against reality instead of silently starting a fresh baseline.
            auto latest = reconstruct_locked(index_.back());
            if (latest) last_roads_ = latest->roads;
        }
    }
}

void HistoryStore::rotate_segment_locked(double real_time) {
    Segment seg;
    seg.path = (fs::path(dir_) / ("hist-" + std::to_string(session_id_) + "-" +
                                   std::to_string(next_segment_index_) + ".bin"))
                   .string();
    ++next_segment_index_;
    seg.start_real_time = real_time;
    seg.end_real_time = real_time;

    SegmentFileHeader sh{kMagic, kVersion, session_id_};
    std::ofstream out(seg.path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(&sh), sizeof(sh));
    seg.baseline_offset = sizeof(sh);

    segments_.push_back(std::move(seg));
    last_segment_start_ = real_time;
    last_roads_.clear();  // the next frame written is this segment's baseline
}

void HistoryStore::capture(const HistoryFrame& frame) {
    const bool need_new_segment =
        segments_.empty() || (frame.real_time - last_segment_start_) >= kSegmentSpan;
    if (need_new_segment) rotate_segment_locked(frame.real_time);

    Segment& seg = segments_.back();
    const bool is_baseline = (seg.start_real_time == frame.real_time);

    std::vector<HistoryRoadRecord> to_write;
    if (is_baseline) {
        to_write = frame.roads;
    } else {
        std::map<std::string, HistoryRoadRecord> current;
        for (const auto& r : frame.roads) current[r.id] = r;

        for (const auto& [id, rec] : current) {
            auto it = std::find_if(last_roads_.begin(), last_roads_.end(),
                                    [&](const HistoryRoadRecord& o) { return o.id == id; });
            const bool changed = it == last_roads_.end() ||
                                  it->congestion != rec.congestion || it->closed != rec.closed;
            if (changed) to_write.push_back(rec);
        }
        // A road that dropped out of frame.roads entirely (its congestion
        // fell back under state_json()'s reporting threshold, and it wasn't
        // closed) needs an explicit "back to normal" delta -- otherwise
        // reconstruction would leave it frozen at its last congested value
        // forever, since silence in a delta stream means "unchanged."
        for (const auto& old : last_roads_) {
            if (!current.count(old.id)) {
                to_write.push_back(HistoryRoadRecord{old.id, 0.0f, false});
            }
        }
    }
    last_roads_ = frame.roads;

    std::vector<uint8_t> payload;
    write_frame_payload(payload, frame, to_write);

    FrameHeader fh{};
    fh.real_time = frame.real_time;
    fh.sim_time = frame.sim_time;
    fh.calendar_time = frame.calendar_time;
    fh.payload_bytes = static_cast<uint32_t>(payload.size());
    fh.vehicle_count = static_cast<uint16_t>(frame.vehicles.size());
    fh.light_count = static_cast<uint16_t>(frame.lights.size());
    fh.road_count = static_cast<uint16_t>(to_write.size());
    fh.event_count = static_cast<uint16_t>(frame.events.size());
    fh.is_baseline = is_baseline ? 1 : 0;

    std::error_code sz_ec;
    const uint64_t offset = static_cast<uint64_t>(fs::file_size(seg.path, sz_ec));

    std::ofstream out(seg.path, std::ios::binary | std::ios::app);
    out.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
    if (!payload.empty()) {
        out.write(reinterpret_cast<const char*>(payload.data()),
                   static_cast<std::streamsize>(payload.size()));
    }
    out.close();

    seg.end_real_time = frame.real_time;
    index_.push_back({frame.real_time, segments_.size() - 1, offset});

    evict_locked(frame.real_time);
}

void HistoryStore::evict_locked(double now_real_time) {
    const double cutoff = now_real_time - window_seconds_;
    // Only ever evict from the front (oldest first) and never the segment
    // currently being written to, so there is no gap in what's retained.
    while (segments_.size() > 1 && segments_.front().end_real_time < cutoff) {
        std::error_code ec;
        fs::remove(segments_.front().path, ec);

        std::vector<IndexEntry> kept;
        kept.reserve(index_.size());
        for (const auto& e : index_) {
            if (e.segment == 0) continue;
            kept.push_back({e.real_time, e.segment - 1, e.byte_offset});
        }
        index_ = std::move(kept);
        segments_.erase(segments_.begin());
    }
}

HistoryStore::Range HistoryStore::range() const {
    Range r;
    if (index_.empty()) return r;
    const double now = now_epoch_seconds();
    r.empty = false;
    r.newest_ago = std::max(0.0, now - index_.back().real_time);
    r.oldest_ago = std::max(0.0, now - index_.front().real_time);
    r.window_seconds = window_seconds_;
    return r;
}

std::optional<HistoryFrame> HistoryStore::reconstruct_locked(const IndexEntry& target) const {
    if (target.segment >= segments_.size()) return std::nullopt;
    const Segment& seg = segments_[target.segment];
    std::ifstream in(seg.path, std::ios::binary);
    if (!in) return std::nullopt;

    std::map<std::string, HistoryRoadRecord> roads;
    HistoryFrame result;
    result.real_time = target.real_time;
    bool found_target = false;

    uint64_t offset = seg.baseline_offset;
    while (true) {
        in.seekg(static_cast<std::streamoff>(offset));
        FrameHeader fh{};
        in.read(reinterpret_cast<char*>(&fh), sizeof(fh));
        if (!in) break;

        std::vector<uint8_t> payload(fh.payload_bytes);
        if (fh.payload_bytes > 0) {
            in.read(reinterpret_cast<char*>(payload.data()),
                    static_cast<std::streamsize>(fh.payload_bytes));
            if (!in) break;
        }
        ParsedFrame parsed = read_frame_payload(payload, fh);

        for (const auto& r : parsed.roads) {
            if (r.congestion <= 0.0f && !r.closed) roads.erase(r.id);
            else roads[r.id] = r;
        }

        if (offset == target.byte_offset) {
            result.sim_time = fh.sim_time;
            result.calendar_time = fh.calendar_time;
            result.vehicles = std::move(parsed.vehicles);
            result.lights = std::move(parsed.lights);
            result.events = std::move(parsed.events);
            result.stats = parsed.stats;
            found_target = true;
            break;
        }
        offset += sizeof(fh) + fh.payload_bytes;
    }
    if (!found_target) return std::nullopt;

    result.roads.reserve(roads.size());
    for (auto& [id, r] : roads) result.roads.push_back(std::move(r));
    return result;
}

std::optional<HistoryFrame> HistoryStore::at(double seconds_ago) const {
    if (index_.empty()) return std::nullopt;
    const double now = now_epoch_seconds();
    const double target_time = now - std::max(0.0, seconds_ago);

    auto it = std::lower_bound(
        index_.begin(), index_.end(), target_time,
        [](const IndexEntry& e, double t) { return e.real_time < t; });

    if (it == index_.begin()) return reconstruct_locked(index_.front());
    if (it == index_.end()) return reconstruct_locked(index_.back());

    const IndexEntry& after = *it;
    const IndexEntry& before = *std::prev(it);
    const bool before_is_closer = (target_time - before.real_time) <= (after.real_time - target_time);
    return reconstruct_locked(before_is_closer ? before : after);
}

std::string HistoryStore::to_state_json(const HistoryFrame& frame) {
    std::ostringstream os;
    os << "{";
    os << "\"time\":" << frame.sim_time << ",";
    // Integer, not raw double -- see the matching comment in
    // TrafficSimulator::state_json(): ostringstream's default 6-significant-
    // digit precision would otherwise round a ~1.7-billion-second epoch
    // value to the nearest ~10000s.
    os << "\"calendarTime\":" << static_cast<long long>(frame.calendar_time) << ",";

    os << "\"vehicles\":[";
    bool first = true;
    for (const auto& v : frame.vehicles) {
        if (!first) os << ",";
        first = false;
        os << "{\"id\":" << jstr(v.id) << ",\"road\":" << jstr(v.road)
           << ",\"pos\":" << v.pos << ",\"speed\":" << v.speed
           << ",\"lane\":" << v.lane << ",\"lat\":" << v.lateral_offset
           << ",\"recalcs\":" << v.recalcs << ",\"state\":" << v.state
           << ",\"type\":" << jstr(v.type) << "}";
    }
    os << "],";

    os << "\"lights\":[";
    first = true;
    for (const auto& l : frame.lights) {
        if (!first) os << ",";
        first = false;
        os << "{\"id\":" << jstr(l.id) << ",\"n\":" << jstr(l.junction)
           << ",\"g\":" << l.group << ",\"d\":" << l.stop_from_end;
        if (l.has_head) {
            os << ",\"hx\":" << l.hx << ",\"hy\":" << l.hy
               << ",\"hux\":" << l.hux << ",\"huy\":" << l.huy;
        }
        os << ",\"state\":" << l.state << "}";
    }
    os << "],";

    os << "\"roads\":[";
    first = true;
    for (const auto& r : frame.roads) {
        if (!first) os << ",";
        first = false;
        os << "{\"id\":" << jstr(r.id) << ",\"congestion\":" << r.congestion
           << ",\"closed\":" << (r.closed ? "true" : "false") << "}";
    }
    os << "],";

    os << "\"events\":[";
    first = true;
    for (const auto& e : frame.events) {
        if (!first) os << ",";
        first = false;
        os << "{\"id\":" << jstr(e.id) << ",\"type\":" << jstr(e.type)
           << ",\"target\":" << jstr(e.target) << ",\"remaining\":" << e.remaining
           << ",\"description\":" << jstr(e.description) << "}";
    }
    os << "],";

    // "active"/"spawned" at the top level and "completedTrips" under "stats"
    // match state_json()'s field names/placement exactly, so the frontend's
    // existing Statistics panel (ui.js) reads a historical frame the same way
    // it reads a live one. The cumulative-since-start displays state_json()
    // ALSO sends (the congestion history chart, per-algorithm breakdown) are
    // deliberately not reproduced here -- they describe the whole run, not
    // this one instant, so they keep showing their live/current values
    // regardless of scrub position, same as the untouched control-panel
    // sliders (spawnRate, speed, ...) do while playback is read-only anyway.
    os << "\"active\":" << frame.vehicles.size() << ",";
    os << "\"spawned\":" << frame.stats.spawned << ",";
    os << "\"stats\":{\"avgCongestion\":" << frame.stats.avg_congestion
       << ",\"activeVehicles\":" << frame.stats.active_vehicles
       << ",\"avgTravelTime\":" << frame.stats.avg_travel_time
       << ",\"completedTrips\":" << frame.stats.completed_trips << "}";
    os << "}";
    return os.str();
}
