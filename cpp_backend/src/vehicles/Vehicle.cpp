#include "vehicles/Vehicle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include "utils/StatisticsManager.hpp"

namespace uts {

namespace {
const std::string kNoRoad{};
constexpr double kRecomputeCooldown = 4.0;   // Giây mô phỏng giữa các lần tìm lại đường
constexpr double kBlockedRetryDelay = 3.0;   // Nhịp độ thử lại khi bị kẹt

// --- Các tham số mô hình IDM (Intelligent Driver Model) ----------------------
// Khoảng cách (gap) được tính từ đầu xe đến đuôi xe (bên gọi hàm đã trừ đi nửa chiều dài của mỗi xe),
// giúp tránh lỗi va chạm tọa độ tâm xuất hiện trong bản tham chiếu.
constexpr double kIdmT = 1.5;        // Thời gian giãn cách mong muốn (giây)
constexpr double kIdmS0 = 2.0;       // Khoảng cách tối thiểu khi đứng yên (mét)
constexpr double kIdmA = 2.5;        // Gia tốc tối đa (m/s^2)
constexpr double kIdmB = 3.0;        // Mức giảm tốc mượt/thoải mái (m/s^2)
const double kIdmSqrtAB = 2.0 * std::sqrt(kIdmA * kIdmB);

// Tín hiệu giao thông (Treiber / traffic-simulation.de): Đèn đỏ đóng vai trò như một vật cản ảo
// đứng yên tại vạch dừng của đường tiếp cận; mô hình IDM sẽ phanh lại trước nó như với một xe dẫn đầu đang dừng.
// Đèn vàng sử dụng bài kiểm tra "vùng lưỡng lự" (dilemma-zone): chỉ dừng lại nếu phanh thoải mái là đủ,
// ngược lại sẽ đi tiếp để giải phóng nút giao.
constexpr double kLightLookahead = 80.0;       // Khoảng cách nhận biết đèn trước vạch dừng (mét)
constexpr double kAmberComfortDecel = 3.0;     // Mức giảm tốc thoải mái khi gặp đèn vàng (m/s^2)

// Làn đường M3: Làn i được đặt tâm tại khoảng cách (i + 0.5) * kLaneWidth về BÊN PHẢI
// dải phân cách; vị trí dạt ngang sẽ trôi mượt (không bao giờ dịch chuyển tức thời) giữa các làn.
constexpr double kLaneWidth = 3.5;         // Chiều rộng làn đường (mét)
constexpr double kLateralSpeed = 2.0;      // Tốc độ dạt ngang khi chuyển làn (m/s)

constexpr double kLaneChangeCooldown = 5.0;   // Thời gian chờ giữa các lần chuyển làn (giây)
constexpr double kLaneReturnInhibit = 12.0;   // Thời gian cấm lượn quay trở lại làn cũ (giây)

// Đảm bảo tuyệt đối không đè vạch/va chạm: IDM là bộ điều khiển mềm - nó phanh gấp khi khoảng cách ngắn
// nhưng không gì ngăn được tọa độ đè lên nhau trong các trạng thái quá độ (nhập làn, vật cản đột ngột).
// Do đó, bộ tích phân sẽ chặn mọi bước di chuyển để đầu xe sau không bao giờ vượt qua đuôi xe trước.
constexpr double kContactMargin = 0.25;      // Khoảng cách an toàn duy trì giữa 2 xe (mét)

// Vòng xoay M5-R
constexpr double kPi = 3.14159265358979323846;
constexpr double kRingSpeed = 8.0;           // Tốc độ giới hạn trên vòng xoay (m/s)
constexpr double kRingApproach = 15.0;       // Khoảng cách trước khi vào vòng xoay để hãm tốc độ (mét)
}  // namespace

// ============================================================================
// --- 1. KHỞI TẠO CONSTRUCTOR -----------------------------------------------
// ============================================================================

Vehicle::Vehicle(std::string id, std::string originId, std::string destinationId,
                 std::shared_ptr<PathfindingStrategy> strategy, CostMode mode)
    : id_(std::move(id)),
      originId_(std::move(originId)),
      destinationId_(std::move(destinationId)),
      strategy_(std::move(strategy)),
      mode_(mode),
      currentIntersectionId_(originId_) {
    // Khởi tạo sự đa dạng tốc độ mong muốn xác định cho từng xe (0.8 .. 1.0)
    // để dòng giao thông không bao giờ di chuyển đồng điệu cứng nhắc. Hash của ID giữ cho replay ổn định.
    unsigned h = 0;
    for (char c : id_) h = h * 31u + static_cast<unsigned>(c);
    idmFactor_ = 0.8 + 0.2 * ((h % 100u) / 99.0);
}

// ============================================================================
// --- 2. HỒ SƠ ĐA HÌNH (MẶC ĐỊNH LỚP CHA) -----------------------------------
// ============================================================================

double Vehicle::speedFactor() const { return 1.0; }
double Vehicle::congestionSensitivity() const { return 1.0; }
bool Vehicle::ignoresTrafficLights() const { return false; }
double Vehicle::lengthMeters() const { return 4.5; }
double Vehicle::widthMeters() const { return 2.0; }

std::vector<int> Vehicle::allowedLanes(int totalLanes) const {
    std::vector<int> lanes;
    for (int i = 0; i < totalLanes; ++i) lanes.push_back(i);
    return lanes;
}

double Vehicle::lanePoliteness() const { return 0.5; }

// ============================================================================
// --- 3. ĐỊNH DANH & TRẠNG THÁI CƠ BẢN --------------------------------------
// ============================================================================

const std::string& Vehicle::id() const { return id_; }
const std::string& Vehicle::origin() const { return originId_; }
const std::string& Vehicle::destination() const { return destinationId_; }
Vehicle::State Vehicle::state() const { return state_; }
bool Vehicle::arrived() const { return state_ == State::Arrived; }
bool Vehicle::onRoad() const { return onRoad_; }
void Vehicle::setSpawnTime(double t) { spawnTime_ = t; }
double Vehicle::spawnTime() const { return spawnTime_; }

// ============================================================================
// --- 4. QUẢN LÝ LỘ TRÌNH & THUẬT TOÁN --------------------------------------
// ============================================================================

const std::vector<road_id_t>& Vehicle::route() const { return route_; }
std::size_t Vehicle::routeIndex() const { return routeIndex_; }
const std::vector<road_id_t>& Vehicle::travelHistory() const { return history_; }
int Vehicle::recalculations() const { return recalculations_; }
double Vehicle::totalDistance() const { return distanceTravelled_; }

void Vehicle::retarget(const std::string& newDestination) {
    destinationId_ = newDestination;
    markRouteDirty();
    recomputeCooldown_ = 0.0;
    if (state_ == State::Arrived) state_ = State::Planning;
}

void Vehicle::setStrategy(std::shared_ptr<PathfindingStrategy> strategy) {
    if (strategy) {
        strategy_ = std::move(strategy);
        markRouteDirty();
    }
}

const PathfindingStrategy& Vehicle::strategy() const { return *strategy_; }
CostMode Vehicle::costMode() const { return mode_; }

bool Vehicle::remainingRouteContains(road_id_t roadId) const {
    for (std::size_t i = routeIndex_; i < route_.size(); ++i) {
        if (route_[i] == roadId) return true;
    }
    return false;
}

// ============================================================================
// --- 5. TRẠNG THÁI ĐỘNG LỰC HỌC & VỊ TRÍ ----------------------------------
// ============================================================================

const std::string& Vehicle::currentRoadId() const { return currentRoadId_; }
double Vehicle::progressOnRoad() const { return progressMeters_; }
double Vehicle::speedMs() const { return speedMs_; }
double Vehicle::accelMs2() const { return accel_; }

const std::string& Vehicle::currentIntersection() const { return currentIntersectionId_; }
node_id_t Vehicle::currentIntersectionIndex() const { return currentIntersectionIndex_; }

// ============================================================================
// --- 6. CHUYỂN LÀN & VỊ TRÍ NGANG (M3 / M4 MOBIL) --------------------------
// ============================================================================

int Vehicle::lane() const { return lane_; }
double Vehicle::lateralOffset() const { return lateralOffset_; }
double Vehicle::targetLateralOffset() const { return targetLateralOffset_; }
int Vehicle::prevLane() const { return prevLane_; }
double Vehicle::prevLaneBlockedUntil() const { return prevLaneBlockedUntil_; }

double Vehicle::idmAccel(double v, double v0, double vLead, double gap) {
    v0 = std::max(0.5, v0);
    gap = std::max(0.1, gap);
    const double deltaV = v - vLead;
    const double sStar =
        kIdmS0 + std::max(0.0, v * kIdmT + (v * deltaV) / kIdmSqrtAB);
    return kIdmA * (1.0 - std::pow(v / v0, 4) - std::pow(sStar / gap, 2));
}

double Vehicle::desiredSpeedOn(const Road& road) const {
    return road.effectiveSpeedMs(congestionSensitivity()) * speedFactor() *
           idmFactor_;
}

bool Vehicle::laneChangeReady() const {
    return onRoad_ && laneChangeCooldown_ <= 0.0 &&
           std::abs(lateralOffset_ - targetLateralOffset_) < 0.3;
}

void Vehicle::setLane(int lane, double simTime) {
    prevLane_ = lane_;
    prevLaneBlockedUntil_ = simTime + kLaneReturnInhibit;
    lane_ = std::max(0, lane);
    targetLateralOffset_ = (lane_ + 0.5) * kLaneWidth;
    laneChangeCooldown_ = kLaneChangeCooldown;
}

// ============================================================================
// --- 7. VÒNG XOAY M5-R (ROUNDABOUT RING MODEL) -----------------------------
// ============================================================================

bool Vehicle::onRing() const { return onRing_; }
const std::string& Vehicle::ringNode() const { return ringNode_; }
double Vehicle::ringS() const { return ringS_; }
double Vehicle::ringLen() const { return ringLen_; }
double Vehicle::ringAngle() const { return ringEntryAngle_ - ringS_ / kRingRadius; }

// ============================================================================
// --- 8. CƠ CHẾ NHƯỜNG ĐƯỜNG & XỬ LÝ KẸT XE (M6 / M7) -----------------------
// ============================================================================

double Vehicle::zoneWait() const { return zoneWait_; }

void Vehicle::addZoneWait(double s) {
    zoneWait_ += s;
    if (zoneWait_ > 15.0) zoneWait_ = 15.0;
}

double Vehicle::stuckTime() const { return stuckTime_; }

bool Vehicle::isForced(double simTime) const { return simTime < forcedUntil_; }

void Vehicle::grantForcedClearance(double untilSimTime) { forcedUntil_ = untilSimTime; }

void Vehicle::rescueTo(Graph& graph, std::size_t newIndex) {
    leaveRoad(graph);            // Giải phóng bộ đếm của con đường hiện tại
    routeIndex_ = newIndex;
    enterRoad(graph, route_[newIndex]);
    progressMeters_ = 0.0;
    speedMs_ = 0.0;
    stuckTime_ = 0.0;
    forcedUntil_ = -1.0;
}

void Vehicle::markRouteDirty() { routeDirty_ = true; }

// ============================================================================
// --- 9. HÀM PRIVATE HỖ TRỢ NỘI BỘ -------------------------------------------
// ============================================================================

double Vehicle::retryDelay() const {
    // Tự tạo độ nhiễu (jitter) xác định cho từng xe (+/- 1 giây). Nếu không có, các xe bị kẹt
    // cùng nhau (khi đóng đường hàng loạt) sẽ thử lại đồng thời, kích hoạt hàng trăm phép tìm kiếm A*
    // trong cùng một tick - gây ra hiện tượng thundering herd làm đóng băng mô phỏng.
    // Mã băm (hash) của ID giúp việc phát lại (replay) mô phỏng luôn nhất quán.
    const double jitter =
        static_cast<double>(std::hash<std::string>{}(id_) % 200u) / 100.0 - 1.0;
    return kBlockedRetryDelay + jitter;
}

void Vehicle::enterRoad(Graph& graph, road_id_t roadId) {
    Road& road = graph.road(roadId);
    graph.vehicleEnteredRoad(roadId);
    onRoad_ = true;
    currentRoadId_ = road.id();
    currentIntersectionIndex_ = kInvalidNodeId;
    currentIntersectionId_ = "";
    progressMeters_ = 0.0;
    zoneWait_ = 0.0;   // Đã vượt qua nút giao: sự thiếu kiên nhẫn được xả hết
    prevLane_ = -1;    // Ràng buộc cấm quay lại làn cũ chỉ áp dụng trên từng đoạn đường

    // M3: Chọn một làn đường trên con đường mới. Con đường đầu tiên: khởi tạo theo ID xe
    // để tạo sự đa dạng. Các con đường sau: giữ nguyên chỉ số làn, giới hạn theo số làn thực tế.
    // Quan trọng: giữ nguyên lateralOffset_ và targetLateralOffset_ qua các chuyển tiếp đường
    // để xe không bị biến dịch ngang đột ngột tại ranh giới giữa các đoạn đường.
    if (history_.empty()) {
        unsigned h = 0;
        for (char c : id_) h = h * 131u + static_cast<unsigned>(c);
        lane_ = static_cast<int>(h % static_cast<unsigned>(road.lanes()));
        lane_ = std::min(lane_, road.lanes() - 1);
        targetLateralOffset_ = (lane_ + 0.5) * kLaneWidth;
        lateralOffset_ = targetLateralOffset_;
    } else {
        // Các đoạn đường tiếp theo: ép lại làn nếu đường mới có ít làn hơn.
        // Chỉ cập nhật offset mục tiêu khi làn thực sự thay đổi,
        // giữ nguyên vị trí ngang hiện tại để chuyển tiếp mượt mà về mặt thị giác
        // thay vì dịch chuyển tức thời về giữa làn.
        const int clamped = std::min(lane_, road.lanes() - 1);
        if (clamped != lane_) {
            lane_ = clamped;
            targetLateralOffset_ = (lane_ + 0.5) * kLaneWidth;
            // Không ngắt dạt ngang ngay — để logic dạt làn trong update()
            // di chuyển xe mượt mà đến mục tiêu mới.
        }
        // Khác: Cùng làn đường, giữ cả lateralOffset_ và targetLateralOffset_
        // chính xác như ở đoạn đường trước.
    }

    history_.push_back(roadId);
    state_ = State::Moving;
}

void Vehicle::leaveRoad(Graph& graph) {
    if (onRoad_ && routeIndex_ < route_.size()) {
        road_id_t rid = route_[routeIndex_];
        Road& r = graph.road(rid);
        graph.vehicleLeftRoad(rid);
        currentIntersectionIndex_ = r.to();
        currentIntersectionId_ = graph.intersection(currentIntersectionIndex_).id();
    }
    onRoad_ = false;
    currentRoadId_ = "";
}

void Vehicle::finishTrip(Graph& graph, StatisticsManager& stats, double simTime) {
    leaveRoad(graph);
    state_ = State::Arrived;
    stats.recordTripCompleted(simTime - spawnTime_, distanceTravelled_, type());
}

// ============================================================================
// --- 10. CẬP NHẬT HÀNH VI & SỰ KIỆN ----------------------------------------
// ============================================================================

bool Vehicle::requestRoute(Graph& graph, StatisticsManager& stats) {
    // Lộ trình luôn bắt đầu từ vị trí hiện tại của phương tiện.
    node_id_t startIdx = currentIntersectionIndex_;
    if (startIdx == kInvalidNodeId) {
        startIdx = graph.nodeId(currentIntersectionId_);
        currentIntersectionIndex_ = startIdx;
    }
    node_id_t destIdx = graph.nodeId(destinationId_);

    road_id_t partialRoad = kInvalidRoadId;  // Đoạn đường xe đang đi dở, giữ lại làm tiền tố
    if (onRoad_ && routeIndex_ < route_.size()) {
        partialRoad = route_[routeIndex_];
        startIdx = graph.road(partialRoad).to();  // Lập kế hoạch từ điểm cuối của đoạn đường đó
    }

    const auto t0 = std::chrono::steady_clock::now();
    RouteResult result = strategy_->findRoute(graph, startIdx, destIdx, mode_);
    const auto t1 = std::chrono::steady_clock::now();
    const double micros =
        std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(t1 - t0)
            .count();
    stats.recordRouteComputation(strategy_->name(), micros, result.nodesExplored,
                                 result.found);

    routeDirty_ = false;
    recomputeCooldown_ = kRecomputeCooldown;

    if (!result.found) {
        if (!onRoad_) state_ = State::Blocked;
        return false;
    }

    if (onRoad_) {
        // Giữ lại con đường xe đang thực sự di chuyển làm chặng đầu tiên.
        std::vector<road_id_t> newRoute;
        newRoute.push_back(partialRoad);
        newRoute.insert(newRoute.end(), result.roadIds.begin(), result.roadIds.end());
        route_ = std::move(newRoute);
        routeIndex_ = 0;
    } else {
        route_ = std::move(result.roadIds);
        routeIndex_ = 0;
        progressMeters_ = 0.0;
        state_ = route_.empty() ? State::Arrived : State::Planning;
    }
    ++recalculations_;
    return true;
}

void Vehicle::onNotify(const TrafficNotification& n) {
    using Kind = TrafficNotification::Kind;
    // Chỉ quan tâm đến các sự kiện có thể làm thay đổi phần lộ trình còn lại.
    const bool routeScoped = n.roadIdx != kInvalidRoadId && remainingRouteContains(n.roadIdx);
    const bool clearing = n.kind == Kind::RoadReopened ||
                          n.kind == Kind::AccidentCleared ||
                          n.kind == Kind::CongestionCleared;
    if (routeScoped || clearing) {
        markRouteDirty();
        // Các vật cản cứng trên chính lộ trình là khẩn cấp - nhưng khi một con đường đông đúc
        // bị đóng, hàng chục xe cùng tính lại đường trong CÙNG MỘT TICK sẽ làm đứng mô phỏng hàng trăm ms.
        // Giải tỏa đột biến này bằng cách rải đều thời gian trong khoảng ~0-0.8 giây.
        if (routeScoped && (n.kind == Kind::RoadClosed || n.kind == Kind::Accident)) {
            recomputeCooldown_ = std::min(
                recomputeCooldown_,
                0.05 * static_cast<double>(std::hash<std::string>{}(id_) % 16));
        } else if (clearing && !routeScoped) {
            // Việc giải tỏa đường ở một nơi khác chỉ có nghĩa là CHẤP NHẬN BẬT BẬT lộ trình tốt hơn có thể tồn tại.
            // Việc tính toán lại cho mọi xe cùng lúc làm treo bản đồ quy mô thành phố (150 xe x ~8 ms A* trong một đợt);
            // hãy phân bố thời gian tối ưu lại rải rác từ 4-20 giây dựa theo ID của xe.
            const double stagger =
                4.0 + static_cast<double>(std::hash<std::string>{}(id_) % 160) / 10.0;
            recomputeCooldown_ = std::max(recomputeCooldown_, stagger);
        }
    }
}

void Vehicle::transferToRoad(Graph& graph, road_id_t newRoad,
                             double progressMeters) {
    const Road& road = graph.road(newRoad);
    // Bộ đếm của con đường cũ bị hủy đã mất theo nó; chỉ tính đoạn đường mới.
    graph.vehicleEnteredRoad(newRoad);
    route_.assign(1, newRoad);
    routeIndex_ = 0;
    onRoad_ = true;
    currentRoadId_ = road.id();
    currentIntersectionIndex_ = kInvalidNodeId;
    currentIntersectionId_ = "";
    progressMeters_ = std::clamp(progressMeters, 0.0, road.length());
    // Giữ nguyên speedMs_/lateralOffset_: phương tiện vẫn ở cùng một vị trí vật lý,
    // chỉ có ID cạnh đồ thị bị thay đổi. Giới hạn lại làn phòng trường hợp đoạn đường bị chia nửa có ít làn hơn.
    lane_ = std::min(lane_, road.lanes() - 1);
    history_.push_back(newRoad);
    state_ = State::Moving;
    markRouteDirty();          // Tính lại lộ trình tiếp theo từ điểm cuối đoạn đường bị chia nửa
    recomputeCooldown_ = 0.0;
}

void Vehicle::update(double dt, Graph& graph, StatisticsManager& stats,
                     double simTime, const Vehicle* lead, double gapMeters) {
    if (state_ == State::Arrived) return;

    recomputeCooldown_ = std::max(0.0, recomputeCooldown_ - dt);
    laneChangeCooldown_ = std::max(0.0, laneChangeCooldown_ - dt);

    // ---- M5-R: Di chuyển trên vòng xoay ------------------------------------
    // Cùng mô hình IDM, nhưng tính dọc theo cung tròn; xe dẫn đường/khoảng cách truyền vào
    // đã được xử lý bởi bộ giải quyết nhận biết vòng xoay. Việc tính lại lộ trình hoãn đến khi ra khỏi vòng xoay.
    if (onRing_) {
        const double vLead = lead ? lead->speedMs() : 0.0;
        accel_ = idmAccel(speedMs_, kRingSpeed * idmFactor_, vLead, gapMeters);
        speedMs_ = std::max(0.0, speedMs_ + accel_ * dt);
        double step = speedMs_ * dt;
        const double maxStep = std::max(0.0, gapMeters - kContactMargin);
        if (step > maxStep) {  // Tuyệt đối không va chạm, tương tự như di chuyển trên đường
            step = maxStep;
            speedMs_ = dt > 1e-9 ? step / dt : 0.0;
        }
        // Theo dõi trạng thái kẹt M7 (dành cho vòng xoay)
        if (speedMs_ < 0.3 && gapMeters < 1e8) stuckTime_ += dt;
        else if (speedMs_ > 1.0) stuckTime_ = 0.0;
        ringS_ += step;
        distanceTravelled_ += step;

        // Bám sát đường tâm của vòng xoay
        if (speedMs_ > 0.5) {
            const double maxStep = kLateralSpeed * dt;
            if (std::abs(lateralOffset_) <= maxStep) lateralOffset_ = 0.0;
            else lateralOffset_ += (lateralOffset_ > 0 ? -maxStep : maxStep);
        }

        if (ringS_ >= ringLen_ && routeIndex_ < route_.size()) {
            onRing_ = false;
            road_id_t nextRoadIdx = route_[routeIndex_];
            Road& exitRoad = graph.road(nextRoadIdx);
            enterRoad(graph, nextRoadIdx);
            // Vòng xoay bàn giao xe tại mép vòng tròn, nằm cách tâm đường xuất kRingRadius
            // (các con đường chạy từ tâm đến tâm nút giao).
            progressMeters_ = std::min(kRingRadius, exitRoad.length() * 0.4);
        }
        state_ = State::Moving;
        return;
    }

    // ---- Lập (lại) kế hoạch lộ trình khi cần -------------------------------
    if ((route_.empty() && state_ != State::Blocked) ||
        (routeDirty_ && recomputeCooldown_ <= 0.0)) {
        requestRoute(graph, stats);
    }
    if (state_ == State::Blocked) {
        speedMs_ = 0.0;
        blockedRetryTimer_ -= dt;
        if (blockedRetryTimer_ <= 0.0) {
            blockedRetryTimer_ = retryDelay();
            if (requestRoute(graph, stats) && !route_.empty()) {
                state_ = State::Planning;
            }
        }
        if (state_ == State::Blocked) return;
    }
    if (route_.empty()) {
        // Trường hợp đặc biệt: Điểm xuất phát == Điểm đến.
        if (currentIntersectionId_ == destinationId_) finishTrip(graph, stats, simTime);
        return;
    }

    // ---- Đi vào con đường đầu tiên / tiếp theo ------------------------------
    if (!onRoad_) {
        road_id_t nextRoadId = route_[routeIndex_];
        Road& nextRoad = graph.road(nextRoadId);

        if (nextRoad.isClosed()) {
            // Đoạn đường phía trước bị đóng trong lúc chờ: tính lại lộ trình ngay.
            markRouteDirty();
            recomputeCooldown_ = 0.0;
            requestRoute(graph, stats);
            if (route_.empty() || graph.road(route_[routeIndex_]).isClosed()) {
                state_ = State::Blocked;
                blockedRetryTimer_ = retryDelay();
                return;
            }
        }

        // (Tín hiệu đèn hiện nằm TRÊN các đường tiếp cận - một xe đứng tại nút
        // đã vượt qua vạch dừng của nó và phải giải phóng nút giao.)
        enterRoad(graph, route_[routeIndex_]);
    }

    // ---- Di chuyển dọc theo đoạn đường hiện tại (bám đuôi xe IDM) ------------
    Road& road = graph.road(route_[routeIndex_]);

    // Tốc độ tự do mong muốn: Tốc độ nhận biết kẹt xe của đường được nhân với hệ số
    // đặc tính xe cộng với hệ số cá nhân của nó.
    double v0 = desiredSpeedOn(road);

    // M5-R: Đoạn đường này kết thúc tại một vòng xoay mà xe sẽ đi vào (lộ trình còn tiếp tục)
    const bool ringAhead = routeIndex_ + 1 < route_.size() &&
                           graph.intersection(road.to()).isRoundabout();
    if (ringAhead &&
        road.length() - kRingRadius - progressMeters_ < kRingApproach) {
        v0 = std::min(v0, kRingSpeed);  // Giảm tốc độ để chuẩn bị vào vòng xoay
    }

    double gap = gapMeters;
    double vLead = lead ? lead->speedMs() : 0.0;

    // Tín hiệu đèn đỏ trên đường tiếp cận đóng vai trò như một xe ảo đứng yên tại vạch dừng,
    // nhờ đó IDM sẽ phanh hãm mượt mà và hàng xe xếp hàng tự nhiên.
    // Đèn vàng: Kiểm tra "vùng lưỡng lự" - chỉ dừng lại nếu độ giảm tốc yêu cầu
    // là thoải mái; nếu không thì tiếp tục đi để giải phóng nút giao.
    bool redAhead = false;
    if (!ignoresTrafficLights() && road.hasSignal()) {
        const double distToStop = road.stopLinePosition() - progressMeters_;
        if (distToStop < kLightLookahead) {
            const SignalState st = graph.intersection(road.to())
                                       .signalStateFor(road.signal().group, simTime);
            bool mustStop = st == SignalState::Red;
            if (st == SignalState::Amber) {
                const double d = std::max(0.5, distToStop);
                mustStop = speedMs_ * speedMs_ / (2.0 * d) <= kAmberComfortDecel;
            }
            if (mustStop) {
                const double stopGap = distToStop - lengthMeters() * 0.5;
                if (stopGap > -1.0) {  // Đầu xe chưa vượt qua vạch dừng
                    redAhead = true;
                    if (stopGap < gap) {
                        gap = std::max(0.05, stopGap);
                        vLead = 0.0;
                    }
                }
            }
        }
    }

    accel_ = idmAccel(speedMs_, v0, vLead, gap);

    speedMs_ = std::max(0.0, speedMs_ + accel_ * dt);
    double step = speedMs_ * dt;
    // Giới hạn cứng cấm va chạm (xem kContactMargin). `gap` đã phản ánh ràng buộc gần nhất:
    // xe phía trước cùng đường, xe phía trước ở nút giao, hoặc vạch dừng ảo.
    const double maxStep = std::max(0.0, gap - kContactMargin);
    if (step > maxStep) {
        step = maxStep;
        speedMs_ = dt > 1e-9 ? step / dt : 0.0;
    }
    // M7: Tích lũy thời gian đứng yên liên tục khi bị chặn bởi một ràng buộc;
    // chuyển động thực sự (> 1 m/s) sẽ reset biến này, giúp tình trạng kẹt xe
    // dạng dừng-rồi-đi dừng lại dưới các ngưỡng leo thang. Dừng chờ đèn đỏ là một sự dừng
    // NỀN NẾP, không phải kẹt cứng - nó không bao giờ được kích hoạt các cấp độ leo thang
    // (bỏ qua vật cản / dịch chuyển giải cứu).
    if (speedMs_ < 0.3 && gap < 1e8 && !redAhead) stuckTime_ += dt;
    else if (speedMs_ > 1.0) stuckTime_ = 0.0;
    progressMeters_ += step;
    distanceTravelled_ += step;

    // M3: Dạt ngang về phía làn đường mục tiêu (không dịch chuyển tức thời), chỉ thực hiện khi
    // xe thực sự lăn bánh — một phương tiện đang đỗ không dạt ngang.
    if (speedMs_ > 0.5) {
        const double delta = targetLateralOffset_ - lateralOffset_;
        const double maxStep = kLateralSpeed * dt;
        if (std::abs(delta) <= maxStep) lateralOffset_ = targetLateralOffset_;
        else lateralOffset_ += (delta > 0 ? maxStep : -maxStep);
    }

    state_ = (redAhead && speedMs_ < 0.3) ? State::WaitingAtLight : State::Moving;

    // M5-R: Xe hướng về vòng xoay rời khỏi đường tại mép vòng tròn (khoảng R trước
    // tâm nút giao) và bắt đầu di chuyển quanh vòng xoay thay vì dịch chuyển tức thời.
    if (ringAhead && progressMeters_ >= road.length() - kRingRadius) {
        distanceTravelled_ -= (progressMeters_ - (road.length() - kRingRadius));
        const Intersection& J = graph.intersection(road.to());
        const Intersection& A = graph.intersection(road.from());
        const Road& nextRoad = graph.road(route_[routeIndex_ + 1]);
        const Intersection& B = graph.intersection(nextRoad.to());

        // Điểm vào = hướng quay ngược lại dọc theo con đường này; điểm ra = hướng về B.
        const double thIn = std::atan2(A.y() - J.y(), A.x() - J.x());
        const double thOut = std::atan2(B.y() - J.y(), B.x() - J.x());
        double arc = std::fmod(thIn - thOut, 2.0 * kPi);
        if (arc < 0.2) arc += 2.0 * kPi;  // Xử lý tràn góc; nếu cực nhỏ/âm = đi vòng quanh

        leaveRoad(graph);
        ++routeIndex_;
        onRing_ = true;
        ringNode_ = J.id();
        ringEntryAngle_ = thIn;
        ringS_ = 0.0;
        ringLen_ = arc * kRingRadius;
        speedMs_ = std::max(2.0, speedMs_ * 0.8);
        targetLateralOffset_ = 0.0;   // Vòng xoay di chuyển dọc theo đường tâm
        return;
    }

    if (progressMeters_ >= road.length()) {
        const double overshoot = progressMeters_ - road.length();
        distanceTravelled_ -= overshoot;  // Không tính dư (sẽ cộng lại nếu chuyển tiếp)
        leaveRoad(graph);
        ++routeIndex_;

        if (currentIntersectionId_ == destinationId_) {
            finishTrip(graph, stats, simTime);
            return;
        }
        if (routeIndex_ >= route_.size()) {
            // Lộ trình đã hết nhưng chưa đến điểm đến (không nên xảy ra) - tính lại lộ trình.
            route_.clear();
            routeIndex_ = 0;
            markRouteDirty();
            recomputeCooldown_ = 0.0;
            return;
        }

        // Chuyển tiếp liền mạch qua các điểm định hướng đường cong (waypoint): Ranh giới giữa
        // hai phân đoạn của CÙNG MỘT con đường là một điểm tạo hình, không phải nút giao.
        // Chuyển phần quãng đường dư đi thẳng sang phân đoạn tiếp theo.
        road_id_t nextRoadIdx = route_[routeIndex_];
        Road& nextRoad = graph.road(nextRoadIdx);
        if (graph.intersection(currentIntersectionIndex_).isWaypoint() &&
            !nextRoad.isClosed()) {
            enterRoad(graph, nextRoadIdx);
            const double carry = std::min(overshoot, nextRoad.length());
            progressMeters_ = carry;
            distanceTravelled_ += carry;
        }
    }
}

}  // namespace uts
