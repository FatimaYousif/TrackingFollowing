#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/components/mode_executor.hpp>
// #include <yolo_msgs/msg/detection_array.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

static const std::string kModeName = "follow_target";

class FollowTarget : public px4_ros2::ModeBase
{
public:
    explicit FollowTarget(rclcpp::Node& node)
        : px4_ros2::ModeBase(node, kModeName)
        , _node(node)
        , _has_detection(false)
        , _no_detection_counter(0)
        , _hold_altitude(-0.0f)  // Default altitude (NED frame)
        , _altitude_locked(false)
    {

        loadParameters();

        _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
        _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
        
        // _detection_sub = _node.create_subscription<yolo_msgs::msg::DetectionArray>(
        _detection_sub = _node.create_subscription<vision_msgs::msg::Detection2DArray>(
            _detection_topic, 
            rclcpp::QoS(10).best_effort(),
            std::bind(&FollowTarget::detectionCallback, this, std::placeholders::_1));
    }

    void onActivate() override 
    { 
        RCLCPP_INFO(_node.get_logger(), "Mode Activated !!!"); 
    }
    
    void onDeactivate() override 
    {   
        RCLCPP_INFO(_node.get_logger(), "Mode Deactivated !!!"); 
    }
    
    void updateSetpoint(float /*dt_s*/) override { }

    bool hasDetection() const { return _has_detection; }
    
    int getNoDetectionCounter() const { return _no_detection_counter; }
    void resetNoDetectionCounter() { _no_detection_counter = 0; }
    
    void resetAltitudeLock() { _altitude_locked = false; }
    

    int getHoverFramesThreshold() const { return _hover_frames_threshold; }
    float getTakeoffAltitude() const { return _takeoff_altitude; }


    void hoverMode()
    {
        // Only capture altitude once when entering hover mode
        if (!_altitude_locked) {
            _hold_altitude = _vehicle_local_position->positionNed().z();
            _altitude_locked = true;
            RCLCPP_INFO(_node.get_logger(), "Hover mode - Altitude locked at: %f m (NED)", _hold_altitude);
        }

        // Hover in place - hold position and altitude
        px4_ros2::TrajectorySetpoint setpoint;
        setpoint.withHorizontalVelocity(Eigen::Vector2f(0.0f, 0.0f))
                .withPositionZ(_hold_altitude)
                .withYawRate(_hover_yaw_rate);
        
        _trajectory_setpoint->update(setpoint);
    }

    void searchMode()
    {
        // Only capture altitude once when entering search mode
        if (!_altitude_locked) {
            _hold_altitude = _vehicle_local_position->positionNed().z();
            _altitude_locked = true;
            RCLCPP_INFO(_node.get_logger(), "Search mode - Altitude locked at: %f m (NED)", _hold_altitude);
        }

        // Hold position and rotate to search
        px4_ros2::TrajectorySetpoint setpoint;
        setpoint.withHorizontalVelocity(Eigen::Vector2f(0.0f, 0.0f))
                .withPositionZ(_hold_altitude)
                .withYawRate(_search_yaw_rate);  // Yaw rate for rotation
        
        _trajectory_setpoint->update(setpoint);
    }

private:
  void loadParameters()
    {
        // Load subscriber settings
        _detection_topic = _node.get_parameter("subscribers.detection_topic").as_string();
       
        // Load flight parameters
        _takeoff_altitude = _node.get_parameter("flight.takeoff_altitude").as_double();

          // Load detection loss handling
        _hover_frames_threshold = _node.get_parameter("detection_loss.hover_frames_threshold").as_int();

        // Load control gains
        _yaw_kp = _node.get_parameter("control.yaw_kp").as_double();
        _dist_kp = _node.get_parameter("control.distance_kp").as_double();
        _height_kp = _node.get_parameter("control.height_kp").as_double();

        // Load camera parameters
        _center_x = _node.get_parameter("camera.center_x").as_double();
        _center_y = _node.get_parameter("camera.center_y").as_double();
        _desired_size = _node.get_parameter("camera.desired_target_size").as_double();

        // Load setpoint parameters - hover
        _hover_velocity_x = _node.get_parameter("setpoint.hover.velocity_x").as_double();
        _hover_velocity_y = _node.get_parameter("setpoint.hover.velocity_y").as_double();
        _hover_velocity_z = _node.get_parameter("setpoint.hover.velocity_z").as_double();
        _hover_yaw_rate = _node.get_parameter("setpoint.hover.yaw_rate").as_double();

        // Load setpoint parameters - search
        _search_velocity_x = _node.get_parameter("setpoint.search.velocity_x").as_double();
        _search_velocity_y = _node.get_parameter("setpoint.search.velocity_y").as_double();
        _search_velocity_z = _node.get_parameter("setpoint.search.velocity_z").as_double();
        _search_yaw_rate = _node.get_parameter("setpoint.search.yaw_rate").as_double();

    }

    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr data)
    // void detectionCallback(const yolo_msgs::msg::DetectionArray::SharedPtr data)
    {
        if (data->detections.empty()) {
            _has_detection = false;
            _no_detection_counter++;
            return;
        }

        // ------------------- ID dependent ------------------------
        // find current tracked ID
        // const yolo_msgs::msg::Detection* target = nullptr;
        const vision_msgs::msg::Detection2D* target = nullptr;

        if (_has_tracked_id) {
            for (const auto& det : data->detections) {
                // if (det.id == _tracked_id) {
                if (_has_tracked_id && det.id == _tracked_id) {
                    target = &det;
                    break;
                }
            }
        }

        // If we don't have a target yet OR lost it, pick the first detection
        if (!target) {
            target = &data->detections[0];
            _tracked_id = target->id;
            _has_tracked_id = true;

            RCLCPP_INFO(_node.get_logger(), "Locked onto new target ID: %d", _tracked_id);
        }
        // ------------------- ID dependent ------------------------

        _has_detection = true;
        _no_detection_counter = 0;

        float x = target->bbox.center.position.x;
        float y = target->bbox.center.position.y;
        float size_y = target->bbox.size_y;
        std::string id = target->id;

        float current_yaw = px4_ros2::quaternionToYaw(_vehicle_attitude->attitude());

        float vbx = _dist_kp * (_desired_size - size_y) / _desired_size;
        float vbz = _height_kp * (_center_y - y) / _center_y;
        float yaw_rate = -_yaw_kp * (_center_x - x) / _center_x;

        float vx = vbx * std::cos(current_yaw);
        float vy = vbx * std::sin(current_yaw);

        _trajectory_setpoint->update(
            Eigen::Vector3f(vx, vy, vbz),
            {},
            {},
            yaw_rate
        );
    }

    rclcpp::Node& _node;
    // rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr _detection_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr _detection_sub;
    
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    bool _has_detection;
    int _no_detection_counter;
    float _hold_altitude;  // Altitude to maintain (NED frame)
    bool _altitude_locked;  // Flag to prevent continuous altitude updates
    
    std::string _tracked_id;
    bool _has_tracked_id = false;


    // Parameters loaded from YAML
    std::string _detection_topic;
    float _takeoff_altitude;
    int _hover_frames_threshold;
    float _yaw_kp;
    float _dist_kp;
    float _height_kp;
    float _center_x;
    float _center_y;
    float _desired_size;
    float _hover_velocity_x, _hover_velocity_y, _hover_velocity_z, _hover_yaw_rate;
    float _search_velocity_x, _search_velocity_y, _search_velocity_z, _search_yaw_rate;
   
    
};

class FollowTargetExecutor : public px4_ros2::ModeExecutorBase
{
public:
    explicit FollowTargetExecutor(px4_ros2::ModeBase& owned_mode)
        : ModeExecutorBase(owned_mode.node(), px4_ros2::ModeExecutorBase::Settings{}, owned_mode, "")
        , _node(owned_mode.node())
        , _mode(dynamic_cast<FollowTarget&>(owned_mode))
    {
        _timer = _node.create_wall_timer(
            100ms,
            std::bind(&FollowTargetExecutor::checkMode, this));
    }

    enum class State { 
        TakingOff, 
        Searching, 
        Following, 
        RTL, 
        Done 
    };

    void onActivate() override
    {
        _state = State::TakingOff;
        runState(State::TakingOff, px4_ros2::Result::Success);
    }

    void onDeactivate(DeactivateReason /*reason*/) override { }

    void runState(State state, px4_ros2::Result result)
    {
        if (result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(_node.get_logger(), "State failed, going to RTL");
            runState(State::RTL, px4_ros2::Result::Success);
            return;
        }

        _state = state;

        switch (state) {
            case State::TakingOff:
                RCLCPP_INFO(_node.get_logger(), "Takeoff");
                takeoff([this](px4_ros2::Result r) {
                    runState(State::Searching, r);
                }, _mode.getTakeoffAltitude());
                break;

            case State::Searching:
                RCLCPP_INFO(_node.get_logger(), "Searching");
                scheduleMode(ownedMode().id(), [](px4_ros2::Result) {});
                break;

            case State::Following:
                RCLCPP_INFO(_node.get_logger(), "Following");
                break;

            case State::RTL:
                RCLCPP_INFO(_node.get_logger(), "RTL");
                rtl([this](px4_ros2::Result r) {
                    runState(State::Done, r);
                });
                break;

            case State::Done:
                waitUntilDisarmed([this](px4_ros2::Result /*r*/) {
                    RCLCPP_INFO(_node.get_logger(), "Complete");
                });
                break;
        }
    }

private:
    void checkMode()
    {
        if (_state == State::Searching || _state == State::Following) {
            bool has_target = _mode.hasDetection();
            int no_detection_counter = _mode.getNoDetectionCounter();

             int hover_threshold = _mode.getHoverFramesThreshold();


            if (_state == State::Searching && has_target) {
                _state = State::Following;
                _mode.resetAltitudeLock();  // Reset altitude lock when transitioning to following
                RCLCPP_INFO(_node.get_logger(), "Target found! Transitioning to Following mode");
            }
            else if (_state == State::Following && !has_target) {
                if (no_detection_counter < hover_threshold) {
                    // Hover for first 30 frames
                    _mode.hoverMode();
                    RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000, 
                        "Target lost! Hovering (counter: %d/30)", no_detection_counter);
                } else {
                    // After 30 frames, switch to search mode
                    _state = State::Searching;
                    _mode.resetAltitudeLock();  // Reset altitude lock when transitioning to search
                    RCLCPP_INFO(_node.get_logger(), "Target lost! Switching to search mode");
                }
            }

            if (_state == State::Searching) {
                _mode.searchMode();
            }
        }
    }

    rclcpp::Node& _node;
    FollowTarget& _mode;
    State _state;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<rclcpp::Node>(kModeName, options);
    auto mode = std::make_shared<FollowTarget>(*node);
    auto executor = std::make_shared<FollowTargetExecutor>(*mode);
    
    executor->doRegister();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}