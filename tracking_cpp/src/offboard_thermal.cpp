#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

// #include "ultralytics_ros/msg/yolo_result.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

static float quaternionToYaw(float w, float x, float y, float z)
{
    return std::atan2(2.0f * (w * z + x * y),
                      1.0f - 2.0f * (y * y + z * z));
}

class FollowTargetNode : public rclcpp::Node
{
public:
    enum class State {
        Searching,
        Following,
        Limbo,          // ← NEW: pilot takeover
    };

    explicit FollowTargetNode()
        : Node("follow_target",
               rclcpp::NodeOptions()
                   .allow_undeclared_parameters(true)
                   .automatically_declare_parameters_from_overrides(true))
        , _state(State::Searching)
        , _has_detection(false)
        , _no_detection_counter(0)
        , _hold_altitude(0.0f)
        , _offboard_setpoint_counter(0)
        , _nav_state(0)
        , _arming_state(0)
        , _takeoff_reached(false)
        , _pilot_takeover(false)          // ← NEW
        , _stop_offboard_output(false)    // ← NEW
        , _offboard_publisher_silenced(false) // ← NEW
    {
        loadParameters();

        _offboard_mode_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", rclcpp::QoS(1).best_effort());
        _trajectory_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", rclcpp::QoS(1).best_effort());
        _vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::QoS(1).best_effort());

        _local_pos_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            rclcpp::QoS(10).best_effort(),
            [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                _local_pos = *msg;
            });
        _attitude_sub = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            rclcpp::QoS(10).best_effort(),
            [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                _attitude = *msg;
            });

        _control_mode_sub = create_subscription<px4_msgs::msg::VehicleControlMode>(
            "/fmu/out/vehicle_control_mode",
            rclcpp::QoS(10).best_effort(),
            [this](px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
                _vehicle_control_mode = *msg;
            });

        // ← CHANGED: now saves the full VehicleStatus message (needed for flag_control_offboard_enabled)
        // _status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
        //     "/fmu/out/vehicle_status",
        //     rclcpp::QoS(10).best_effort(),
        //     [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        //         _vehicle_status = *msg;
        //         _nav_state    = msg->nav_state;
        //         _arming_state = msg->arming_state;
        //     });

        // _detection_sub = create_subscription<ultralytics_ros::msg::YoloResult>(
        //     _detection_topic,
        //     rclcpp::QoS(10).reliable(),
        //     std::bind(&FollowTargetNode::detectionCallback, this, std::placeholders::_1));


        _detection_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/yolo/bbox_info",
        rclcpp::QoS(10).reliable(),
        std::bind(&FollowTargetNode::detectionCallback, this, std::placeholders::_1));


        _timer = create_wall_timer(
            100ms,
            std::bind(&FollowTargetNode::timerCallback, this));
    }

private:
    void loadParameters()
    {
        _detection_topic        = get_parameter("thermal_subscribers.detection_topic").as_string();
        _takeoff_altitude       = static_cast<float>(get_parameter("flight.takeoff_altitude").as_double());
        _hover_frames_threshold = get_parameter("detection_loss.hover_frames_threshold").as_int();
        _yaw_kp                 = static_cast<float>(get_parameter("control.yaw_kp").as_double());
        _dist_kp                = static_cast<float>(get_parameter("control.distance_kp").as_double());
        _height_kp              = static_cast<float>(get_parameter("control.height_kp").as_double());
        _center_x               = static_cast<float>(get_parameter("thermal_camera.center_x").as_double());
        _center_y               = static_cast<float>(get_parameter("thermal_camera.center_y").as_double());

        // RCLCPP_INFO(get_logger(), "testing thermal camera: detection topic=%s, center x=%.2f", _detection_topic.c_str(),_center_x);

        _desired_size           = static_cast<float>(get_parameter("thermal_camera.desired_target_size").as_double());
        _hover_yaw_rate         = static_cast<float>(get_parameter("setpoint.hover.yaw_rate").as_double());
        _search_yaw_rate        = static_cast<float>(get_parameter("setpoint.search.yaw_rate").as_double());
    }

    // -----------------------------------------------------------------------
    // NEW: Pilot takeover helpers (ported from path_control)
    // -----------------------------------------------------------------------
    void checkPilotStateSwitch()
    {
        if (!_vehicle_control_mode.flag_control_offboard_enabled && !_pilot_takeover) {
            _pilot_takeover = true;
            _state = State::Limbo;
            RCLCPP_WARN(get_logger(),
                "Pilot takeover detected. Entering LIMBO and silencing offboard publishers");
        }
    }

    void silenceOffboardPublishers()
    {
        if (_offboard_publisher_silenced) { return; }

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.position[0] = NAN;  sp.position[1] = NAN;  sp.position[2] = NAN;
        sp.velocity[0] = NAN;  sp.velocity[1] = NAN;  sp.velocity[2] = NAN;
        sp.yaw         = NAN;
        sp.yawspeed    = NAN;
        sp.timestamp   = getTimestampUs();
        _trajectory_pub->publish(sp);

        _stop_offboard_output      = true;
        _offboard_publisher_silenced = true;

        RCLCPP_WARN(get_logger(), "Offboard publishers silenced. Pilot has full control.");
    }

    // -----------------------------------------------------------------------
    // Main control loop (100 ms)
    // -----------------------------------------------------------------------
    void timerCallback()
    {
        // ← NEW: check for pilot takeover on every tick (except when already in Limbo)
        if (_state != State::Limbo) {
            checkPilotStateSwitch();
        }

        publishOffboardControlMode();

        switch (_state) {
            case State::Searching:
                runSearching();
                break;

            case State::Following:
                if (!_has_detection) {
                    if (_no_detection_counter == 1) {
                        captureHoldPosition();
                    }
                    if (_no_detection_counter < _hover_frames_threshold) {
                        captureHoldPosition();
                        runHover();
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Target lost – hovering (%d/%d)",
                            _no_detection_counter, _hover_frames_threshold);
                    } else {
                        captureHoldPosition();
                        _state = State::Searching;
                        RCLCPP_INFO(get_logger(), "Target lost – switching to Search");
                    }
                }
                break;

            // ← NEW: Limbo state — silence publishers and do nothing
            case State::Limbo:
                if (!_offboard_publisher_silenced) {
                    silenceOffboardPublishers();
                }
                break;
        }
    }

    void captureHoldPosition()
    {
        _hold_altitude   = _local_pos.z;
        _hold_position_x = _local_pos.x;
        _hold_position_y = _local_pos.y;
        RCLCPP_INFO(get_logger(),
            "Hold position captured: x=%.2f y=%.2f z=%.2f (NED)",
            _hold_position_x, _hold_position_y, _hold_altitude);
    }

    void runHover()
    {
        if (_stop_offboard_output) { return; }  // ← NEW guard
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp   = getTimestampUs();
        sp.position[0] = _hold_position_x;
        sp.position[1] = _hold_position_y;
        sp.position[2] = _hold_altitude;
        sp.yawspeed    = static_cast<float>(_hover_yaw_rate);
        sp.velocity[0] = NAN;
        sp.velocity[1] = NAN;
        sp.velocity[2] = NAN;
        sp.yaw         = NAN;
        _trajectory_pub->publish(sp);
    }

    void runSearching()
    {
        if (_stop_offboard_output) { return; }  // ← NEW guard
        if (!_position_initialized && _local_pos.timestamp != 0) {
            captureHoldPosition();
            _position_initialized = true;
        }
        if (_has_detection) {
            captureHoldPosition();
            _state = State::Following;
            RCLCPP_INFO(get_logger(), "Target found – switching to Following");
            return;
        }

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp   = getTimestampUs();
        sp.position[0] = _hold_position_x;
        sp.position[1] = _hold_position_y;
        sp.position[2] = _hold_altitude;
        sp.yawspeed    = static_cast<float>(_search_yaw_rate);
        sp.velocity[0] = NAN;
        sp.velocity[1] = NAN;
        sp.velocity[2] = NAN;
        sp.yaw         = NAN;
        _trajectory_pub->publish(sp);
    }

    // void detectionCallback(const ultralytics_ros::msg::YoloResult::SharedPtr data)
    // {
    //     const auto &detections = data->detections.detections;

    //     if (detections.empty()) {
    //         _has_detection = false;
    //         _no_detection_counter++;
    //         return;
    //     }

    //     const auto &target = detections[0];
    //     _has_detection        = true;
    //     _no_detection_counter = 0;

    //     float x      = target.bbox.center.position.x;
    //     float y      = target.bbox.center.position.y;
    //     float size_y = target.bbox.size_y;

    //     float qw = _attitude.q[0];
    //     float qx = _attitude.q[1];
    //     float qy = _attitude.q[2];
    //     float qz = _attitude.q[3];
    //     float current_yaw = quaternionToYaw(qw, qx, qy, qz);

    //     float vbx      = _dist_kp  * (_desired_size - size_y) / _desired_size;
    //     float yaw_rate = -_yaw_kp * (_center_x - x) / _center_x;

    //     float vx = vbx * std::cos(current_yaw);
    //     float vy = vbx * std::sin(current_yaw);

    //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
    //         "Target: %s | vbx: %.2f | x: %.1f y: %.1f | size_y: %.1f",
    //         target.id.c_str(), vbx, x, y, size_y);

    //     if (_state != State::Following) return;
    //     if (_stop_offboard_output)      return;   // ← NEW guard

    //     px4_msgs::msg::TrajectorySetpoint sp{};
    //     sp.timestamp   = getTimestampUs();
    //     sp.velocity[0] = vx;
    //     sp.velocity[1] = vy;
    //     sp.velocity[2] = 0.0f;
    //     sp.position[0] = NAN;
    //     sp.position[1] = NAN;
    //     sp.position[2] = _hold_altitude;
    //     sp.yaw         = NAN;
    //     sp.yawspeed    = yaw_rate;
    //     _trajectory_pub->publish(sp);
    // }



    void detectionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4)
        {
            _has_detection = false;
            _no_detection_counter++;
            return;
        }

        _has_detection        = true;
        _no_detection_counter = 0;

        float x      = msg->data[0];  // center x
        float y      = msg->data[1];  // center y
        float size_y = msg->data[3];  // height of bbox

        // --- Get yaw ---
        float qw = _attitude.q[0];
        float qx = _attitude.q[1];
        float qy = _attitude.q[2];
        float qz = _attitude.q[3];
        float current_yaw = quaternionToYaw(qw, qx, qy, qz);

        // --- Control ---
        float vbx      = _dist_kp  * (_desired_size - size_y) / _desired_size;
        float yaw_rate = -_yaw_kp * (_center_x - x) / _center_x;

        float vx = vbx * std::cos(current_yaw);
        float vy = vbx * std::sin(current_yaw);

        if (_state != State::Following) return;
        if (_stop_offboard_output)      return;

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp   = getTimestampUs();
        sp.velocity[0] = vx;
        sp.velocity[1] = vy;
        sp.velocity[2] = 0.0f;
        sp.position[0] = NAN;
        sp.position[1] = NAN;
        sp.position[2] = _hold_altitude;
        sp.yaw         = NAN;
        sp.yawspeed    = yaw_rate;

        _trajectory_pub->publish(sp);
    }


    void publishOffboardControlMode()
    {
        if (_stop_offboard_output) { return; }  // ← NEW guard
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp    = getTimestampUs();
        msg.position     = true;
        msg.velocity     = true;
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        _offboard_mode_pub->publish(msg);
    }

    void publishHoverSetpoint()
    {
        if (_stop_offboard_output) { return; }  // ← NEW guard
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp   = getTimestampUs();
        sp.position[0] = 0.0f;
        sp.position[1] = 0.0f;
        sp.position[2] = -std::abs(_takeoff_altitude);
        sp.yaw         = 0.0f;
        sp.velocity[0] = NAN;
        sp.velocity[1] = NAN;
        sp.velocity[2] = NAN;
        sp.yawspeed    = NAN;
        _trajectory_pub->publish(sp);
    }

    void armAndSetOffboard()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(get_logger(), "Arm command sent");
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        RCLCPP_INFO(get_logger(), "OFFBOARD mode command sent");
    }

    void publishVehicleCommand(uint16_t command, float param1 = NAN, float param2 = NAN)
    {
        if (_stop_offboard_output) { return; }  // ← NEW guard
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp        = getTimestampUs();
        msg.command          = command;
        msg.param1           = param1;
        msg.param2           = param2;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        _vehicle_command_pub->publish(msg);
    }

    uint64_t getTimestampUs()
    {
        return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
    }

    // Publishers / Subscribers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  _offboard_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   _trajectory_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       _vehicle_command_pub;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_pos_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr      _attitude_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        _status_sub;
    
    // rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr    _detection_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _detection_sub;


    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr _control_mode_sub;

    rclcpp::TimerBase::SharedPtr _timer;

    // Cached sensor data
    px4_msgs::msg::VehicleLocalPosition _local_pos{};
    px4_msgs::msg::VehicleAttitude      _attitude{};
    // px4_msgs::msg::VehicleStatus        _vehicle_status{};  // ← NEW: full status saved
    px4_msgs::msg::VehicleControlMode _vehicle_control_mode{};

    // State
    State    _state;
    bool     _has_detection;
    int      _no_detection_counter;
    float    _hold_altitude;
    float    _hold_position_x{0.0f};
    float    _hold_position_y{0.0f};
    int      _offboard_setpoint_counter;
    uint8_t  _nav_state;
    uint8_t  _arming_state;
    bool     _takeoff_reached;
    bool     _position_initialized{false};

    // NEW: Pilot takeover flags
    bool     _pilot_takeover;
    bool     _stop_offboard_output;
    bool     _offboard_publisher_silenced;

    // Parameters
    std::string _detection_topic;
    float       _takeoff_altitude;
    int         _hover_frames_threshold;
    float       _yaw_kp;
    float       _dist_kp;
    float       _height_kp;
    float       _center_x;
    float       _center_y;
    float       _desired_size;
    float       _hover_yaw_rate;
    float       _search_yaw_rate;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTargetNode>());
    rclcpp::shutdown();
    return 0;
}



































// --------------------- WITHOUT PILOT TAKEOVER -----------------------------

// #include <rclcpp/rclcpp.hpp>

// // PX4 message types (from px4_msgs package)
// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/vehicle_attitude.hpp>
// #include <px4_msgs/msg/vehicle_status.hpp>

// // ultralytics detections
// // #include "ultralytics_ros/msg/yolo_result.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"


// #include <Eigen/Geometry>
// #include <algorithm>
// #include <cmath>

// using namespace std::chrono_literals;

// // ---------------------------------------------------------------------------
// // Helper: quaternion (w, x, y, z) → yaw (rad, NED convention)
// // ---------------------------------------------------------------------------
// static float quaternionToYaw(float w, float x, float y, float z)
// {
//     // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
//     return std::atan2(2.0f * (w * z + x * y),
//                       1.0f - 2.0f * (y * y + z * z));
// }

// // ---------------------------------------------------------------------------
// // FollowTargetNode
// // ---------------------------------------------------------------------------
// class FollowTargetNode : public rclcpp::Node
// {
// public:
//     // -----------------------------------------------------------------------
//     // State machine
//     // -----------------------------------------------------------------------
//     enum class State {
//         Searching,
//         Following,
//     };

//     explicit FollowTargetNode()
//         : Node("follow_target",
//                rclcpp::NodeOptions()
//                    .allow_undeclared_parameters(true)
//                    .automatically_declare_parameters_from_overrides(true))
//         // , _state(State::Idle)
//         , _state(State::Searching)
//         , _has_detection(false)
//         , _no_detection_counter(0)
//         , _hold_altitude(0.0f)
//         , _offboard_setpoint_counter(0)
//         , _nav_state(0)
//         , _arming_state(0)
//         , _takeoff_reached(false)
//     {
//         loadParameters();

//         // --- Publishers ---
//         _offboard_mode_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
//             "/fmu/in/offboard_control_mode", rclcpp::QoS(1).best_effort());

//         _trajectory_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
//             "/fmu/in/trajectory_setpoint", rclcpp::QoS(1).best_effort());

//         _vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
//             "/fmu/in/vehicle_command", rclcpp::QoS(1).best_effort());

//         // --- Subscribers ---
//         _local_pos_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
//             "/fmu/out/vehicle_local_position",
//             rclcpp::QoS(10).best_effort(),
//             [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
//                 _local_pos = *msg;
//             });

//         _attitude_sub = create_subscription<px4_msgs::msg::VehicleAttitude>(
//             "/fmu/out/vehicle_attitude",
//             rclcpp::QoS(10).best_effort(),
//             [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
//                 _attitude = *msg;
//             });

//         _status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
//             "/fmu/out/vehicle_status",
//             rclcpp::QoS(10).best_effort(),
//             [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
//                 _nav_state    = msg->nav_state;
//                 _arming_state = msg->arming_state;
//             });

//         // _detection_sub = create_subscription<ultralytics_ros::msg::YoloResult>(
//         //     _detection_topic,
//         //     rclcpp::QoS(10).reliable(),
//         //     std::bind(&FollowTargetNode::detectionCallback, this, std::placeholders::_1));
        

//         _detection_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
//     "/yolo/bbox_info",
//     rclcpp::QoS(10).reliable(),
//     std::bind(&FollowTargetNode::detectionCallback, this, std::placeholders::_1));

        
//         // --- Main loop timer: 10 Hz ---
//         _timer = create_wall_timer(
//             100ms,
//             std::bind(&FollowTargetNode::timerCallback, this));
//     }

// private:
//     // -----------------------------------------------------------------------
//     // Parameters
//     // -----------------------------------------------------------------------
//     void loadParameters()
//     {
//         _detection_topic       = get_parameter("thermal_subscribers.detection_topic").as_string();
//         _takeoff_altitude      = static_cast<float>(get_parameter("flight.takeoff_altitude").as_double());
//         _hover_frames_threshold = get_parameter("detection_loss.hover_frames_threshold").as_int();
//         _yaw_kp                = static_cast<float>(get_parameter("control.yaw_kp").as_double());
//         _dist_kp               = static_cast<float>(get_parameter("control.distance_kp").as_double());
//         _height_kp             = static_cast<float>(get_parameter("control.height_kp").as_double());
//         _center_x              = static_cast<float>(get_parameter("thermal_camera.center_x").as_double());
//         _center_y              = static_cast<float>(get_parameter("thermal_camera.center_y").as_double());

//     //    RCLCPP_INFO(get_logger(), "testing thermal camera: detection topic=%s, center x=%.2f", _detection_topic.c_str(),_center_x);

//         _desired_size          = static_cast<float>(get_parameter("thermal_camera.desired_target_size").as_double());
//         _hover_yaw_rate        = static_cast<float>(get_parameter("setpoint.hover.yaw_rate").as_double());
//         _search_yaw_rate       = static_cast<float>(get_parameter("setpoint.search.yaw_rate").as_double());
//     }

//     // -----------------------------------------------------------------------
//     // Main control loop (100 ms)
//     // -----------------------------------------------------------------------
//     void timerCallback()
//     {
//         publishOffboardControlMode();

//         switch (_state) {

//             case State::Searching:
//                 runSearching();
//                 break;

//             case State::Following:
//                 // Detection callback drives the setpoints.
//                 // If detection was lost, handle state switch here.
//                 if (!_has_detection) {
//                     if (_no_detection_counter == 1) {
//                         // First tick without detection – snap position immediately
//                         captureHoldPosition();
//                     }
//                     if (_no_detection_counter < _hover_frames_threshold) {
//                         runHover();
//                         RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
//                             "Target lost – hovering (%d/%d)",
//                             _no_detection_counter, _hover_frames_threshold);
//                     } else {
//                         captureHoldPosition();   // snap position before entering search
//                         _state = State::Searching;
//                         RCLCPP_INFO(get_logger(), "Target lost – switching to Search");
//                     }
//                 }
//                 break;
//         }
//     }


//     // --- Capture current position as the hold target (called at every transition site) ---
//     void captureHoldPosition()
//     {
//         _hold_altitude   = _local_pos.z;
//         _hold_position_x = _local_pos.x;
//         _hold_position_y = _local_pos.y;
//         RCLCPP_INFO(get_logger(),
//             "Hold position captured: x=%.2f y=%.2f z=%.2f (NED)",
//             _hold_position_x, _hold_position_y, _hold_altitude);
//     }

//     // --- Hover in place (position already captured at transition site) ---
//     void runHover()
//     {
//         px4_msgs::msg::TrajectorySetpoint sp{};
//         sp.timestamp   = getTimestampUs();
//         sp.position[0] = _hold_position_x;
//         sp.position[1] = _hold_position_y;
//         sp.position[2] = _hold_altitude;
//         sp.yawspeed    = static_cast<float>(_hover_yaw_rate);
//         sp.velocity[0] = NAN;
//         sp.velocity[1] = NAN;
//         sp.velocity[2] = NAN;
//         sp.yaw         = NAN;
//         _trajectory_pub->publish(sp);
//     }

//     // --- Search (yaw rotation in place, position already captured at transition site) ---
//     void runSearching()
//     {
//           // Capture current position on first entry so we don't fly to (0,0,0)
//         if (!_position_initialized && _local_pos.timestamp != 0) {
//             captureHoldPosition();
//             _position_initialized = true;
//         }
        
//         // Transition to Following the moment a detection arrives
//         if (_has_detection) {
//             captureHoldPosition();   // snapshot altitude for Following
//             _state = State::Following;
//             RCLCPP_INFO(get_logger(), "Target found – switching to Following");
//             return;
//         }

//         px4_msgs::msg::TrajectorySetpoint sp{};
//         sp.timestamp   = getTimestampUs();
//         sp.position[0] = _hold_position_x;
//         sp.position[1] = _hold_position_y;
//         sp.position[2] = _hold_altitude;
//         sp.yawspeed    = static_cast<float>(_search_yaw_rate);
//         sp.velocity[0] = NAN;
//         sp.velocity[1] = NAN;
//         sp.velocity[2] = NAN;
//         sp.yaw         = NAN;
//         _trajectory_pub->publish(sp);
//     }

//     // -----------------------------------------------------------------------
//     // Detection callback – drives Following setpoints
//     // -----------------------------------------------------------------------
//     // void detectionCallback(const yolo_msgs::msg::DetectionArray::SharedPtr data)
    
//     // void detectionCallback(const ultralytics_ros::msg::YoloResult::SharedPtr data)
//     // {
//     //     const auto &detections=data->detections.detections;

//     //     if (detections.empty())
//     //     {
//     //         _has_detection    = false;
//     //         _no_detection_counter++;
//     //         return;
//     //     }

//     //     const auto &target = detections[0];

//     //     _has_detection        = true;
//     //     _no_detection_counter = 0;

//     //     float x      = target.bbox.center.position.x;
//     //     float y      = target.bbox.center.position.y;
//     //     float size_y = target.bbox.size_y;

//     //     // Current yaw from attitude quaternion (w, x, y, z layout in px4_msgs)
//     //     float qw = _attitude.q[0];
//     //     float qx = _attitude.q[1];
//     //     float qy = _attitude.q[2];
//     //     float qz = _attitude.q[3];
//     //     float current_yaw = quaternionToYaw(qw, qx, qy, qz);

//     //     // --- Control law (identical to original) ---
//     //     float vbx     = _dist_kp  * (_desired_size - size_y) / _desired_size;
//     //     float yaw_rate = -_yaw_kp * (_center_x - x) / _center_x;
//     //     // vbz unused – constant-altitude mode
//     //     // float vbz  = _height_kp * (_center_y - y) / _center_y;

//     //     float vx = vbx * std::cos(current_yaw);
//     //     float vy = vbx * std::sin(current_yaw);

//     //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
//     //         "Target: %s | vbx: %.2f | x: %.1f y: %.1f | size_y: %.1f",
//     //         target.id.c_str(), vbx, x, y, size_y);

//     //     // Only publish if we are actually in Following state
//     //     if (_state != State::Following) return;

//     //     px4_msgs::msg::TrajectorySetpoint sp{};
//     //     sp.timestamp   = getTimestampUs();
//     //     sp.velocity[0] = vx;
//     //     sp.velocity[1] = vy;
//     //     sp.velocity[2] = 0.0f;   // no vertical velocity
//     //     sp.position[0] = NAN;
//     //     sp.position[1] = NAN;
//     //     sp.position[2] = _hold_altitude;   // altitude hold via position Z
//     //     sp.yaw         = NAN;
//     //     sp.yawspeed    = yaw_rate;
//     //     _trajectory_pub->publish(sp);
//     // }


//     void detectionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
//     {
//         if (msg->data.size() < 4)
//         {
//             _has_detection = false;
//             _no_detection_counter++;
//             return;
//         }

//         _has_detection        = true;
//         _no_detection_counter = 0;

//         float x      = msg->data[0];  // center x
//         float y      = msg->data[1];  // center y
//         float size_y = msg->data[3];  // height of bbox

//         // --- Get yaw ---
//         float qw = _attitude.q[0];
//         float qx = _attitude.q[1];
//         float qy = _attitude.q[2];
//         float qz = _attitude.q[3];
//         float current_yaw = quaternionToYaw(qw, qx, qy, qz);

//         // --- Control ---
//         float vbx      = _dist_kp  * (_desired_size - size_y) / _desired_size;
//         float yaw_rate = -_yaw_kp * (_center_x - x) / _center_x;

//         float vx = vbx * std::cos(current_yaw);
//         float vy = vbx * std::sin(current_yaw);

//         if (_state != State::Following) return;

//         px4_msgs::msg::TrajectorySetpoint sp{};
//         sp.timestamp   = getTimestampUs();
//         sp.velocity[0] = vx;
//         sp.velocity[1] = vy;
//         sp.velocity[2] = 0.0f;
//         sp.position[0] = NAN;
//         sp.position[1] = NAN;
//         sp.position[2] = _hold_altitude;
//         sp.yaw         = NAN;
//         sp.yawspeed    = yaw_rate;

//         _trajectory_pub->publish(sp);
//     }

//     // -----------------------------------------------------------------------
//     // Offboard heartbeat – must be published before & during offboard mode
//     // -----------------------------------------------------------------------
//     void publishOffboardControlMode()
//     {
//         px4_msgs::msg::OffboardControlMode msg{};
//         msg.timestamp          = getTimestampUs();
//         msg.position           = true;   // we use position Z for altitude
//         msg.velocity           = true;   // we use velocity XY for tracking
//         msg.acceleration       = false;
//         msg.attitude           = false;
//         msg.body_rate          = false;
//         _offboard_mode_pub->publish(msg);
//     }

//     // A plain hover setpoint used during the pre-arm streaming phase
//     void publishHoverSetpoint()
//     {
//         px4_msgs::msg::TrajectorySetpoint sp{};
//         sp.timestamp   = getTimestampUs();
//         sp.position[0] = 0.0f;
//         sp.position[1] = 0.0f;
//         sp.position[2] = -std::abs(_takeoff_altitude);
//         sp.yaw         = 0.0f;
//         sp.velocity[0] = NAN;
//         sp.velocity[1] = NAN;
//         sp.velocity[2] = NAN;
//         sp.yawspeed    = NAN;
//         _trajectory_pub->publish(sp);
//     }

//     // -----------------------------------------------------------------------
//     // Vehicle command helpers
//     // -----------------------------------------------------------------------
//     void armAndSetOffboard()
//     {
//         // 1. Arm
//         publishVehicleCommand(
//             px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
//             1.0f);   // param1 = 1.0 → arm
//         RCLCPP_INFO(get_logger(), "Arm command sent");

//         // 2. Switch to OFFBOARD flight mode
//         publishVehicleCommand(
//             px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
//             1.0f,    // param1: base mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
//             6.0f);   // param2: custom mode 6 = OFFBOARD in PX4
//         RCLCPP_INFO(get_logger(), "OFFBOARD mode command sent");
//     }

//     void publishVehicleCommand(uint16_t command,
//                                float param1 = NAN,
//                                float param2 = NAN)
//     {
//         px4_msgs::msg::VehicleCommand msg{};
//         msg.timestamp        = getTimestampUs();
//         msg.command          = command;
//         msg.param1           = param1;
//         msg.param2           = param2;
//         msg.target_system    = 1;
//         msg.target_component = 1;
//         msg.source_system    = 1;
//         msg.source_component = 1;
//         msg.from_external    = true;
//         _vehicle_command_pub->publish(msg);
//     }

//     // -----------------------------------------------------------------------
//     // Utility
//     // -----------------------------------------------------------------------
//     uint64_t getTimestampUs()
//     {
//         return static_cast<uint64_t>(
//             get_clock()->now().nanoseconds() / 1000ULL);
//     }

//     // -----------------------------------------------------------------------
//     // Publishers / Subscribers
//     // -----------------------------------------------------------------------
//     rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  _offboard_mode_pub;
//     rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   _trajectory_pub;
//     rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       _vehicle_command_pub;

//     rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_pos_sub;
//     rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr      _attitude_sub;
//     rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        _status_sub;
//     // rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr      _detection_sub;
//     // rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr      _detection_sub;


//     rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _detection_sub;

//     rclcpp::TimerBase::SharedPtr _timer;

//     // -----------------------------------------------------------------------
//     // Cached sensor data
//     // -----------------------------------------------------------------------
//     px4_msgs::msg::VehicleLocalPosition _local_pos{};
//     px4_msgs::msg::VehicleAttitude      _attitude{};

//     // -----------------------------------------------------------------------
//     // State
//     // -----------------------------------------------------------------------
//     State    _state;
//     bool     _has_detection;
//     int      _no_detection_counter;
//     float    _hold_altitude;
//     float    _hold_position_x{0.0f};
//     float    _hold_position_y{0.0f};
//     int      _offboard_setpoint_counter;
//     uint8_t  _nav_state;
//     uint8_t  _arming_state;
//     bool     _takeoff_reached;
//     // Add to state variables:
//     bool _position_initialized{false};


//     // -----------------------------------------------------------------------
//     // Parameters
//     // -----------------------------------------------------------------------
//     std::string _detection_topic;
//     float       _takeoff_altitude;
//     int         _hover_frames_threshold;
//     float       _yaw_kp;
//     float       _dist_kp;
//     float       _height_kp;
//     float       _center_x;
//     float       _center_y;
//     float       _desired_size;
//     float       _hover_yaw_rate;
//     float       _search_yaw_rate;
// };

// // ---------------------------------------------------------------------------
// // main
// // ---------------------------------------------------------------------------
// int main(int argc, char* argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FollowTargetNode>());
//     rclcpp::shutdown();
//     return 0;
// }
