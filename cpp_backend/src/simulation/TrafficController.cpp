#include "simulation/TrafficController.hpp"

#include <algorithm>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"

// Default length of traffic event (when the caller doesn't say)
double default_duration(const std::string& type) {
    if (type == "closure") return 60.0;
    if (type == "accident") return 50.0;
    if (type == "light") return 40.0;
    return 45.0; //congestion event
}

constexpr std::size_t kMaxLog = 30;

void TrafficController::set_graph(TrafficGraph* graph) {
    events_.clear();  //clear all event from old map
    graph_ = graph;
    next_event_id_ = 1;
    random_event_timer_ = 20.0;
}

void TrafficController::clear() {
    events_.clear();
    log_.clear();
}

void TrafficController::log_line(const std::string& line) {
    log_.push_back(line);
    if (log_.size() > kMaxLog) log_.erase(log_.begin());
}

std::string TrafficController::random_road() const {
    if (!graph_ || graph_->edges.empty()) return "";
    std::uniform_int_distribution<std::size_t> pick(0, graph_->edges.size() - 1);
    auto it = graph_->edges.begin();
    std::advance(it, pick(rng_));
    return it->first;
}

std::string TrafficController::random_signalized_node() const {
    if (!graph_ || graph_->intersections.empty()) return "";
    // A signal fault only means something at a junction that has a signal, so
    // sample a few times and give up rather than pick an unlit node.
    std::uniform_int_distribution<std::size_t> pick(0, graph_->intersections.size() - 1);
    for (int attempt = 0; attempt < 64; ++attempt) {
        auto it = graph_->intersections.begin();
        std::advance(it, pick(rng_));
        if (it->second && it->second->is_signalized && it->second->light_system) {
            return it->first;
        }
    }
    for (const auto& [id, node] : graph_->intersections) {
        if (node && node->is_signalized && node->light_system) return id;
    }
    return "";  // nothing lit on this map: the event is skipped
}

std::string TrafficController::trigger_event(const std::string& type,
                                             std::string target_id,
                                             double duration) {
    if (!graph_) return "";

    if (target_id.empty()) {
        target_id = (type == "light") ? random_signalized_node() : random_road();
    }
    if (target_id.empty()) return "";

    // The target has to exist, or apply()/expire() would silently do nothing
    if (type == "light") {
        auto it = graph_->intersections.find(target_id);
        if (it == graph_->intersections.end() || !it->second->light_system) return "";
    } else {
        if (!graph_->get_edge(target_id)) return "";
    }

    // Closing an already closed road would let the second expire() reopen it
    // while the first event is still running.
    if (type == "closure") {
        const Road* road = graph_->get_edge(target_id);
        if (road && road->is_closed) return "";
    }

    if (duration <= 0.0) duration = default_duration(type);

    const std::string id = "E" + std::to_string(next_event_id_++);
    std::unique_ptr<TrafficEvent> event = makeEvent(type, id, target_id, duration);
    if (!event) return "";  // unknown type

    event->apply(*graph_);
    notifyAll(event->startNotification());
    log_line(event->description());
    events_.push_back(std::move(event));
    return id;
}

int TrafficController::expire_events_targeting(const std::string& target_id) {
    if (!graph_) return 0;

    int expired = 0;
    for (auto& event : events_) {
        if (event->target() != target_id || event->expired()) continue;
        event->expire(*graph_);
        notifyAll(event->endNotification());
        log_line(event->endNotification().description);
        ++expired;
    }
    if (expired > 0) {
        events_.erase(std::remove_if(events_.begin(), events_.end(),
                                     [&](const std::unique_ptr<TrafficEvent>& e) {
                                         return e->target() == target_id;
                                     }),
                      events_.end());
    }
    return expired;
}

void TrafficController::update(double dt) {
    if (!graph_) return;

    // Advance every event and undo the ones whose time is up.
    for (auto& event : events_) {
        const bool was_expired = event->expired();
        event->advance(dt);
        if (!was_expired && event->expired()) {
            event->expire(*graph_);
            notifyAll(event->endNotification());
            log_line(event->endNotification().description);
        }
    }
    events_.erase(std::remove_if(events_.begin(), events_.end(),
                                 [](const std::unique_ptr<TrafficEvent>& e) {
                                     return e->expired();
                                 }),
                  events_.end());

    // Background events so a running simulation keeps changing on its own.
    if (!random_events_) return;
    random_event_timer_ -= dt;
    if (random_event_timer_ > 0.0) return;

    std::uniform_real_distribution<double> next(18.0, 45.0);
    random_event_timer_ = next(rng_);
    // Congestion twice: it is the mildest event, so it should be the common one.
    static const char* kTypes[] = {"congestion", "accident", "closure", "light",
                                   "congestion"};
    std::uniform_int_distribution<int> pick(0, 4);
    trigger_event(kTypes[pick(rng_)], "", 0.0);
}