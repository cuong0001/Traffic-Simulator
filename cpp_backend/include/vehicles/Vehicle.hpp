#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/Graph.hpp"
#include "routing/PathfindingStrategy.hpp"
#include "utils/Observer.hpp"

namespace uts {

class StatisticsManager;  // Khai báo trước (Forward declaration)

class Vehicle : public IObserver {
public:
    enum class State { Planning, Moving, WaitingAtLight, Blocked, Arrived };

    // --- 1. Khởi tạo & Hủy ----------------------------------------------------
    Vehicle(std::string id, std::string originId, std::string destinationId,
            std::shared_ptr<PathfindingStrategy> strategy, CostMode mode);
    ~Vehicle() override = default;

    // --- 2. Hồ sơ Đa hình (Ghi đè bởi các lớp con) ----------------------------
    virtual std::string type() const = 0;
    virtual double speedFactor() const; /// Hệ số nhân tốc độ.
    virtual double congestionSensitivity() const; /// 1.0 (max) => kẹt xe.
    virtual bool ignoresTrafficLights() const; /// cho phép vượt đèn đỏ, đèn vàng không
                                               /// dành cho xe đặc biệt
    
    virtual double lengthMeters() const; /// lấy chiều dài vehicle
    virtual double widthMeters() const; /// lấy chiều rộng vehicle

    virtual std::vector<int> allowedLanes(int totalLanes) const; /// trả về vị trí làn mà xe được 

    virtual double lanePoliteness() const; // nhường xe khác hay không càng thấp càng chen

    // --- 3. Định danh & Trạng thái cơ bản -------------------------------------
    const std::string& id() const;
    const std::string& origin() const; // vị trí bắt đầu
    const std::string& destination() const; // vị trí kết thúc
    // trạng thái
    State state() const;
    bool arrived() const;
    bool onRoad() const;
    void setSpawnTime(double t);
    double spawnTime() const;
    
    // --- 4. Quản lý Lộ trình & Thuật toán -------------------------------------
    const std::vector<road_id_t>& route() const;
    std::size_t routeIndex() const;
    const std::vector<road_id_t>& travelHistory() const;
    int recalculations() const;
    double totalDistance() const;

    /// Thay đổi điểm đến giữa chừng (tính năng UI "Modify Vehicle").
    void retarget(const std::string& newDestination);
    /// Thay đổi thuật toán tìm đường tại thời điểm thực thi (Strategy pattern).
    void setStrategy(std::shared_ptr<PathfindingStrategy> strategy);
    const PathfindingStrategy& strategy() const;
    CostMode costMode() const;
    /// Kiểm tra xem phần lộ trình CHƯA ĐI ĐẾN có chứa `roadId` hay không.
    bool remainingRouteContains(road_id_t roadId) const;

    // --- 5. Trạng thái Động lực học & Vị trí ----------------------------------
    const std::string& currentRoadId() const;
    double progressOnRoad() const;
    double speedMs() const;
    double accelMs2() const;

    const std::string& currentIntersection() const;
    node_id_t currentIntersectionIndex() const;

    // --- 6. Chuyển làn & Vị trí ngang (M3 / M4 MOBIL) --------------------------
    int lane() const;
    double lateralOffset() const;
    double targetLateralOffset() const;
    int prevLane() const;
    double prevLaneBlockedUntil() const;

    /// Công thức gia tốc IDM (dùng chung cho mô hình bám đuôi và MOBIL).
    static double idmAccel(double v, double v0, double vLead, double gap);
    /// Tốc độ tự do mong muốn của xe này trên đoạn đường cho trước (m/s).
    double desiredSpeedOn(const Road& road) const;
    /// Trả về true khi xe đang ở trên đường, hết thời gian chờ, và không dạt làn.
    bool laneChangeReady() const;
    /// Thực hiện chuyển làn: đặt mục tiêu dạt làn + thời gian chờ.
    void setLane(int lane, double simTime = 0.0);

    // --- 7. Vòng xoay M5-R (Roundabout Ring Model) -----------------------------
    static constexpr double kRingRadius = 34.0;
    bool onRing() const;
    const std::string& ringNode() const;
    double ringS() const;
    double ringLen() const;
    double ringAngle() const;

    // --- 8. Cơ chế Nhường đường & Xử lý Kẹt xe (M6 / M7) -----------------------
    double zoneWait() const;
    void addZoneWait(double s);
    double stuckTime() const;

    bool isForced(double simTime) const;
    void grantForcedClearance(double untilSimTime);
    void rescueTo(Graph& graph, std::size_t newIndex);

    // --- 9. Cập nhật Hành vi & Sự kiện ---------------------------------------
    /// Tính toán (hoặc tính toán lại) lộ trình dựa trên chiến lược hiện tại.
    bool requestRoute(Graph& graph, StatisticsManager& stats);

    /**
     * Cập nhật chuyển động của xe sau dt giây mô phỏng.
     */
    void update(double dt, Graph& graph, StatisticsManager& stats, double simTime,
                const Vehicle* lead = nullptr, double gapMeters = 1e9);

    /// Callback của Observer: phản ứng khi có sự kiện giao thông.
    void onNotify(const TrafficNotification& notification) override;

    /// Chỉnh sửa bản đồ trực tiếp: chuyển xe sang đoạn đường mới bị tách.
    void transferToRoad(Graph& graph, road_id_t newRoad, double progressMeters);

protected:
    void markRouteDirty();

private:
    // --- Các hàm hỗ trợ nội bộ ------------------------------------------------
    double retryDelay() const;
    void enterRoad(Graph& graph, road_id_t roadId);
    void leaveRoad(Graph& graph);
    void finishTrip(Graph& graph, StatisticsManager& stats, double simTime);

    // --- 1. Thông tin Định danh & Cấu hình -----------------------------------
    std::string id_;
    std::string originId_;
    std::string destinationId_;
    std::shared_ptr<PathfindingStrategy> strategy_;
    CostMode mode_;
    State state_ = State::Planning;

    // --- 2. Lộ trình & Trạng thái Đồ thị -------------------------------------
    std::vector<road_id_t> route_;
    std::size_t routeIndex_ = 0;
    double progressMeters_ = 0.0;
    bool onRoad_ = false;
    std::string currentRoadId_ = "";
    node_id_t currentIntersectionIndex_ = kInvalidNodeId;
    std::string currentIntersectionId_ = "";

    // --- 3. Động lực học & Mô hình IDM ---------------------------------------
    double speedMs_ = 0.0;     ///< Tốc độ hiện tại (m/s)
    double accel_ = 0.0;       ///< Gia tốc tick trước (m/s^2)
    double idmFactor_ = 1.0;   ///< Hệ số tốc độ mong muốn riêng (0.8..1.0)

    // --- 4. Trạng thái Nhiều làn đường & Chuyển làn (M3 / M4) ----------------
    int lane_ = 0;
    double lateralOffset_ = 0.0;        ///< Khoảng cách ngang (mét)
    double targetLateralOffset_ = 0.0;  ///< Mục tiêu dạt ngang (mét)
    double laneChangeCooldown_ = 0.0;   ///< Thời gian chờ chuyển làn tiếp theo (s)
    int prevLane_ = -1;                ///< Làn vừa rời đi gần nhất
    double prevLaneBlockedUntil_ = 0;  ///< Mốc thời gian cấm quay lại làn cũ

    // --- 5. Trạng thái Vòng xoay (M5-R) ---------------------------------------
    bool onRing_ = false;
    std::string ringNode_;
    double ringS_ = 0.0;
    double ringLen_ = 0.0;
    double ringEntryAngle_ = 0.0;

    // --- 6. Bộ đếm Nhường đường & Giải quyết Kẹt xe (M6 / M7) -----------------
    bool routeDirty_ = false;
    double recomputeCooldown_ = 0.0;   ///< Thời gian chờ tính lại đường (s)
    double blockedRetryTimer_ = 0.0;   ///< Bộ đếm thử lại khi bị Blocked (s)
    double zoneWait_ = 0.0;            ///< Thời gian tích lũy sự thiếu kiên nhẫn M6 (s)
    double stuckTime_ = 0.0;           ///< Thời gian đứng yên liên tục M7 (s)
    double forcedUntil_ = -1.0;        ///< Mốc hết hạn cấp quyền Cấp-2 M7 (s)

    // --- 7. Thống kê & Lịch sử -----------------------------------------------
    std::vector<road_id_t> history_;
    int recalculations_ = 0;
    double distanceTravelled_ = 0.0;
    double spawnTime_ = 0.0;
};

}  // namespace uts
