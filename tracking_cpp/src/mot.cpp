#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/components/mode_executor.hpp>

#include "ultralytics_ros/msg/yolo_result.hpp"

#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <Eigen/Geometry>
#include <algorithm>

#include <geometry_msgs/msg/twist.hpp>
#include "tracking_cpp/msg/plots.hpp"

using namespace std::chrono_literals;

static const std::string kModeName = "follow_target";

// ---------------------------------------------------------------------------
// Lightweight struct for last-known target bbox
// ---------------------------------------------------------------------------
struct DetectedObject
{
    std::string id;
    float center_x{0.f}, center_y{0.f};
    float size_x{0.f}, size_y{0.f};

    void update(const std::string& new_id, float cx, float cy, float sx, float sy)
    {
        id = new_id; center_x = cx; center_y = cy; size_x = sx; size_y = sy;
    }
};

// ---------------------------------------------------------------------------
// IoU helper — each bbox described as (center_x, center_y, size_x, size_y)
// ---------------------------------------------------------------------------
static float calculateIoU(float cx1, float cy1, float sx1, float sy1,
                           float cx2, float cy2, float sx2, float sy2)
{
    float x1_min = cx1 - sx1 / 2.f,  x1_max = cx1 + sx1 / 2.f;
    float y1_min = cy1 - sy1 / 2.f,  y1_max = cy1 + sy1 / 2.f;
    float x2_min = cx2 - sx2 / 2.f,  x2_max = cx2 + sx2 / 2.f;
    float y2_min = cy2 - sy2 / 2.f,  y2_max = cy2 + sy2 / 2.f;

    float inter_w = std::max(0.f, std::min(x1_max, x2_max) - std::max(x1_min, x2_min));
    float inter_h = std::max(0.f, std::min(y1_max, y2_max) - std::max(y1_min, y2_min));
    float inter   = inter_w * inter_h;
    if (inter <= 0.f) return 0.f;

    float union_area = sx1 * sy1 + sx2 * sy2 - inter;
    return (union_area > 0.f) ? inter / union_area : 0.f;
}

// ---------------------------------------------------------------------------

class FollowTarget : public px4_ros2::ModeBase
{
public:
    explicit FollowTarget(rclcpp::Node& node)
        : px4_ros2::ModeBase(node, kModeName)
        , _node(node)
        , _has_detection(false)
        , _no_detection_counter(0)
        , _hold_altitude(-0.0f)
        , _altitude_locked(false)
        , _target_id("")
    {
        loadParameters();

        _trajectory_setpoint    = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        _vehicle_attitude       = std::make_shared<px4_ros2::OdometryAttitude>(*this);
        _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

        _odometry_sub = _node.create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                _current_yaw_rate = msg->angular_velocity[2];
            });

        _detection_sub = _node.create_subscription<ultralytics_ros::msg::YoloResult>(
            _detection_topic,
            rclcpp::QoS(10).reliable(),
            std::bind(&FollowTarget::detectionCallback, this, std::placeholders::_1));

        _plots_pub = _node.create_publisher<tracking_cpp::msg::Plots>(
            "/follow_target/plots", 10);
    }

    void onActivate()   override { 
    _hold_altitude = getCurrentAltitude();    
    RCLCPP_INFO(_node.get_logger(), "Mode Activated !!!"); }
    void onDeactivate() override { RCLCPP_INFO(_node.get_logger(), "Mode Deactivated !!!"); }
    void updateSetpoint(float /*dt_s*/) override { }

    // --- accessors used by executor -----------------------------------------
    bool  hasDetection()            const { return _has_detection; }
    int   getNoDetectionCounter()   const { return _no_detection_counter; }
    int   getHoverFramesThreshold() const { return _hover_frames_threshold; }
    float getTakeoffAltitude()      const { return _takeoff_altitude; }
    float getCurrentAltitude()      const { return _vehicle_local_position->positionNed().z(); }

    void resetNoDetectionCounter() { _no_detection_counter = 0; }
    void resetAltitudeLock()       { _altitude_locked = false; }

    // ---- hover: hold position + altitude -----------------------------------
    void hoverMode()
    {
        lockAltitudeAndPosition("Hover");

        px4_ros2::TrajectorySetpoint setpoint;
        setpoint.withHorizontalPosition(_hold_position_xy)
                .withPositionZ(_hold_altitude)
                .withYawRate(_hover_yaw_rate);

        _trajectory_setpoint->update(setpoint);

        _last_cmd_vx       = 0.0f;
        _last_cmd_vy       = 0.0f;
        _last_cmd_vz       = 0.0f;
        _last_cmd_yaw_rate = _hover_yaw_rate;

        publishAnalysis("hover");
    }

    // ---- search: hold position + altitude, rotate to scan -----------------
    void searchMode()
    {
        lockAltitudeAndPosition("Search");

        px4_ros2::TrajectorySetpoint setpoint;
        setpoint.withHorizontalPosition(_hold_position_xy)
                .withPositionZ(_hold_altitude)
                .withYawRate(_search_yaw_rate);

        _trajectory_setpoint->update(setpoint);

        _last_cmd_vx       = 0.0f;
        _last_cmd_vy       = 0.0f;
        _last_cmd_vz       = 0.0f;
        _last_cmd_yaw_rate = _search_yaw_rate;

        publishAnalysis("search");
    }

private:
    // -----------------------------------------------------------------------
    // Lock altitude + XY position once on mode entry
    // -----------------------------------------------------------------------
    void lockAltitudeAndPosition(const char* mode_name)
    {
        if (_altitude_locked) return;

        _hold_altitude    = getCurrentAltitude();
        _hold_position_xy = Eigen::Vector2f(
            _vehicle_local_position->positionNed().x(),
            _vehicle_local_position->positionNed().y());
        _altitude_locked  = true;

        RCLCPP_INFO(_node.get_logger(),
            "%s mode - altitude locked at %.2f m (NED)", mode_name, _hold_altitude);
    }

    // -----------------------------------------------------------------------
    // Target tracking + reacquisition
    //
    // Step 1: direct ID match  →  primary target still visible, fast path.
    // Step 2: IoU fallback     →  target changed ID, try to re-match by
    //                             overlap with the last-known bounding box.
    //
    // Returns true when a usable target is found; fills out_* with its data.
    // -----------------------------------------------------------------------
    bool updateObjects(const std::vector<vision_msgs::msg::Detection2D>& detections,
                       float& out_x, float& out_y, float& out_size_y)
    {
        // --- Step 1: direct ID match ----------------------------------------
        for (const auto& det : detections) {
            if (det.id == _target_id) {
                out_x      = det.bbox.center.position.x;
                out_y      = det.bbox.center.position.y;
                out_size_y = det.bbox.size_y;

                _last_known_target.update(det.id,
                    det.bbox.center.position.x, det.bbox.center.position.y,
                    det.bbox.size_x, det.bbox.size_y);

                return true;
            }
        }

        // --- Step 2: IoU reacquisition --------------------------------------
        if (_last_known_target.id.empty()) return false;

        const float exp_cx = _last_known_target.center_x;
        const float exp_cy = _last_known_target.center_y;
        const float exp_sx = _last_known_target.size_x + 150.f;
        const float exp_sy = _last_known_target.size_y + 100.f;

        float best_iou = 0.1f;
        const vision_msgs::msg::Detection2D* best = nullptr;

        for (const auto& det : detections) {
            float iou = calculateIoU(
                det.bbox.center.position.x, det.bbox.center.position.y,
                det.bbox.size_x + 150.f,   det.bbox.size_y + 100.f,
                exp_cx, exp_cy, exp_sx, exp_sy);

            if (iou > best_iou) { best_iou = iou; best = &det; }
        }

        if (!best) return false;

        RCLCPP_INFO(_node.get_logger(),
            "Target reacquired via IoU=%.3f | old_id=%s -> new_id=%s",
            best_iou, _target_id.c_str(), best->id.c_str());

        _target_id = best->id;
        _last_known_target.update(best->id,
            best->bbox.center.position.x, best->bbox.center.position.y,
            best->bbox.size_x, best->bbox.size_y);

        out_x      = best->bbox.center.position.x;
        out_y      = best->bbox.center.position.y;
        out_size_y = best->bbox.size_y;
        return true;
    }

    // -----------------------------------------------------------------------

    void detectionCallback(const ultralytics_ros::msg::YoloResult::SharedPtr msg)
    {
        const auto& detections = msg->detections.detections;

        // --- FPS estimation ---
        _total_frame_count++;
        _fps_frame_count++;
        auto now = _node.get_clock()->now();
        if (!_timing_started) {
            _start_time     = now;
            _last_fps_time  = now;
            _timing_started = true;
        }
        double fps_dt = (now - _last_fps_time).seconds();
        if (fps_dt >= 1.0) {
            _fps_estimate    = static_cast<float>(_fps_frame_count) / static_cast<float>(fps_dt);
            _fps_frame_count = 0;
            _last_fps_time   = now;
        }

        if (detections.empty()) {
            _has_detection = false;
            _no_detection_counter++;
            return;
        }

        // Seed target ID on very first detection
        if (_target_id.empty()) {
            _target_id = detections[0].id;
            RCLCPP_INFO(_node.get_logger(), "Initial target ID: %s", _target_id.c_str());
        }

        float x = 0.f, y = 0.f, size_y = 0.f;

        if (!updateObjects(detections, x, y, size_y)) {
            // No tracked target found — fall back to first detection
            x      = detections[0].bbox.center.position.x;
            y      = detections[0].bbox.center.position.y;
            size_y = detections[0].bbox.size_y;
        }

        _has_detection        = true;
        _no_detection_counter = 0;
        _detection_frame_count++;

        RCLCPP_INFO(_node.get_logger(), "x: %.1f y: %.1f | size_y: %.1f", x, y, size_y);

        float current_yaw = px4_ros2::quaternionToYaw(_vehicle_attitude->attitude());

        float vbx      = _dist_kp   * (_desired_size - size_y) / _desired_size;
        float vbz      = _height_kp * (_center_y - y)          / _center_y;
        float yaw_rate = -_yaw_kp   * (_center_x - x)          / _center_x;

        float vx = vbx * std::cos(current_yaw);
        float vy = vbx * std::sin(current_yaw);

        // _trajectory_setpoint->update(
        //     Eigen::Vector3f(vx, vy, vbz),
        //     {}, {}, yaw_rate);

        px4_ros2::TrajectorySetpoint setpoint;
        setpoint.withVelocity(Eigen::Vector3f(vx, vy, 0.0f))  // Horizontal velocities, zero vertical
                .withPositionZ(_hold_altitude)                  // Explicit altitude hold
                .withYawRate(yaw_rate);

        _trajectory_setpoint->update(setpoint);

        _last_cmd_vx       = vx;
        _last_cmd_vy       = vy;
        _last_cmd_vz       = vbz;
        _last_cmd_yaw_rate = yaw_rate;

        _last_target_x    = x;
        _last_target_y    = y;
        _last_target_size = size_y;

        publishAnalysis("follow");
    }

    // -----------------------------------------------------------------------

    void publishAnalysis(const std::string& mode)
    {
        auto msg = tracking_cpp::msg::Plots();

        msg.header.stamp = _node.get_clock()->now();

        // -------for target vs UAV trajectory plot
        auto pos = _vehicle_local_position->positionNed();
        msg.uav_x = pos.x();
        msg.uav_y = pos.y();
        msg.uav_z = pos.z();

        float yaw = px4_ros2::quaternionToYaw(_vehicle_attitude->attitude());
        msg.uav_yaw = yaw;

        // -------for actual vs commanded velocity plot
        auto vel = _vehicle_local_position->velocityNed();
        msg.actual_vx = vel.x();
        msg.actual_vy = vel.y();
        msg.actual_vz = vel.z();

        msg.actual_yaw_rate = _current_yaw_rate;

        msg.cmd_vx       = _last_cmd_vx;
        msg.cmd_vy       = _last_cmd_vy;
        msg.cmd_vz       = _last_cmd_vz;
        msg.cmd_yaw_rate = _last_cmd_yaw_rate;

        msg.target_px   = _last_target_x;
        msg.target_py   = _last_target_y;
        msg.target_size = _last_target_size;

        msg.pixel_error_x = _center_x - _last_target_x;
        msg.pixel_error_y = _center_y - _last_target_y;
        msg.size_error    = _desired_size - _last_target_size;

        msg.mode             = mode;
        msg.has_detection    = _has_detection;
        msg.no_detection_counter = _no_detection_counter;

        // elapsed time since first detection
        if (_timing_started) {
            msg.elapsed_time = static_cast<float>(
                (_node.get_clock()->now() - _start_time).seconds());
        } else {
            msg.elapsed_time = 0.0f;
        }

        // visual accuracy: % of frames where detection was present
        if (_total_frame_count > 0) {
            msg.visual_accuracy = 100.0f *
                static_cast<float>(_detection_frame_count) /
                static_cast<float>(_total_frame_count);
        } else {
            msg.visual_accuracy = 0.0f;
        }

        msg.fps = _fps_estimate;

        // controller error: normalised Euclidean pixel centroid error
        float px_err_x = msg.pixel_error_x;
        float px_err_y = msg.pixel_error_y;
        msg.controller_error = std::sqrt(px_err_x * px_err_x + px_err_y * px_err_y) / _center_x;

        msg.image_center_x = _center_x;
        msg.image_center_y = _center_y;

        _plots_pub->publish(msg);
    }

    // -----------------------------------------------------------------------

    void loadParameters()
    {
        _detection_topic        = _node.get_parameter("subscribers.detection_topic").as_string();
        RCLCPP_INFO(_node.get_logger(), "detection topic: %s", _detection_topic.c_str());

        _takeoff_altitude       = _node.get_parameter("flight.takeoff_altitude").as_double();
        _hover_frames_threshold = _node.get_parameter("detection_loss.hover_frames_threshold").as_int();
        _yaw_kp                 = _node.get_parameter("control.yaw_kp").as_double();
        _dist_kp                = _node.get_parameter("control.distance_kp").as_double();
        _height_kp              = _node.get_parameter("control.height_kp").as_double();
        _center_x               = _node.get_parameter("camera.center_x").as_double();
        _center_y               = _node.get_parameter("camera.center_y").as_double();
        _desired_size           = _node.get_parameter("camera.desired_target_size").as_double();
        _hover_velocity_x       = _node.get_parameter("setpoint.hover.velocity_x").as_double();
        _hover_velocity_y       = _node.get_parameter("setpoint.hover.velocity_y").as_double();
        _hover_velocity_z       = _node.get_parameter("setpoint.hover.velocity_z").as_double();
        _hover_yaw_rate         = _node.get_parameter("setpoint.hover.yaw_rate").as_double();
        _search_velocity_x      = _node.get_parameter("setpoint.search.velocity_x").as_double();
        _search_velocity_y      = _node.get_parameter("setpoint.search.velocity_y").as_double();
        _search_velocity_z      = _node.get_parameter("setpoint.search.velocity_z").as_double();
        _search_yaw_rate        = _node.get_parameter("setpoint.search.yaw_rate").as_double();
    }

    // ---- members -----------------------------------------------------------
    rclcpp::Node& _node;
    rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr _detection_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr   _odometry_sub;

    std::shared_ptr<px4_ros2::TrajectorySetpointType>  _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryAttitude>        _vehicle_attitude;
    std::shared_ptr<px4_ros2::OdometryLocalPosition>   _vehicle_local_position;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _debug_traj_pub;
    rclcpp::Publisher<tracking_cpp::msg::Plots>::SharedPtr  _plots_pub;

    Eigen::Vector2f _hold_position_xy;
    bool            _has_detection;
    int             _no_detection_counter;
    float           _hold_altitude;
    bool            _altitude_locked;

    // Reacquisition state
    std::string    _target_id;
    DetectedObject _last_known_target;

    // Command + target cache for plotting
    float _last_cmd_vx{0}, _last_cmd_vy{0}, _last_cmd_vz{0}, _last_cmd_yaw_rate{0};
    float _last_target_x{0}, _last_target_y{0}, _last_target_size{0};

    // Odometry
    float _current_yaw_rate{0.0f};

    // Timing / stats
    rclcpp::Time _start_time;
    bool         _timing_started{false};
    int          _detection_frame_count{0};
    int          _total_frame_count{0};
    float        _fps_estimate{0.0f};
    rclcpp::Time _last_fps_time;
    int          _fps_frame_count{0};

    // Parameters loaded from YAML
    std::string _detection_topic;
    float _takeoff_altitude;
    int   _hover_frames_threshold;
    float _yaw_kp, _dist_kp, _height_kp;
    float _center_x, _center_y, _desired_size;
    float _hover_velocity_x, _hover_velocity_y, _hover_velocity_z, _hover_yaw_rate;
    float _search_velocity_x, _search_velocity_y, _search_velocity_z, _search_yaw_rate;
};

// ===========================================================================

class FollowTargetExecutor : public px4_ros2::ModeExecutorBase
{
public:
    explicit FollowTargetExecutor(px4_ros2::ModeBase& owned_mode)
        : ModeExecutorBase(owned_mode.node(), px4_ros2::ModeExecutorBase::Settings{}, owned_mode, "")
        , _node(owned_mode.node())
        , _mode(dynamic_cast<FollowTarget&>(owned_mode))
    {
        _check_timer = _node.create_wall_timer(
            100ms,
            std::bind(&FollowTargetExecutor::checkMode, this));
    }

    enum class State { Initializing, Searching, Following };

    void onActivate() override
    {
        RCLCPP_INFO(_node.get_logger(), "Mode Activated - Waiting for odometry...");
        _state = State::Initializing;

        _init_timer = _node.create_wall_timer(
            2s,
            [this]() {
                _state = State::Searching;
                runState(State::Searching, px4_ros2::Result::Success);
            });
    }

    void onDeactivate(DeactivateReason /*reason*/) override { }

    void runState(State state, px4_ros2::Result /*result*/)
    {
        _state = state;

        switch (state) {
            case State::Initializing:
                RCLCPP_INFO(_node.get_logger(), "Initializing state....");
                break;

            case State::Searching:
                RCLCPP_INFO(_node.get_logger(), "Searching");
                scheduleMode(ownedMode().id(), [](px4_ros2::Result) {});
                break;

            case State::Following:
                break;
        }
    }

private:
    void checkMode()
    {
        if (_state != State::Searching && _state != State::Following) return;

        const bool has_target      = _mode.hasDetection();
        const int  no_detect_count = _mode.getNoDetectionCounter();
        const int  hover_threshold = _mode.getHoverFramesThreshold();

        if (_state == State::Searching && has_target) {
            _state = State::Following;
            _mode.resetAltitudeLock();
        }
        else if (_state == State::Following && !has_target) {
            if (no_detect_count < hover_threshold) {
                _mode.hoverMode();
                RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
                    "Target lost! Hovering (%d/%d)", no_detect_count, hover_threshold);
            } else {
                _state = State::Searching;
                _mode.resetAltitudeLock();
                RCLCPP_INFO(_node.get_logger(), "Target lost! Switching to search mode");
            }
        }

        if (_state == State::Searching) {
            _mode.searchMode();
        }
    }

    rclcpp::Node&            _node;
    FollowTarget&            _mode;
    State                    _state{State::Initializing};
    rclcpp::TimerBase::SharedPtr _check_timer;
    rclcpp::TimerBase::SharedPtr _init_timer;
};

// ===========================================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node     = std::make_shared<rclcpp::Node>(kModeName, options);
    auto mode     = std::make_shared<FollowTarget>(*node);
    auto executor = std::make_shared<FollowTargetExecutor>(*mode);

    executor->doRegister();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}