// params
// detection topic
// take off altitude
// search vx vy yaw

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

// #include <yolo_msgs/msg/detection_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <Eigen/Geometry>

using namespace std::chrono_literals;

static const std::string kModeName = "search_target";

class FollowTarget : public px4_ros2::ModeBase
{
public:
    explicit FollowTarget(rclcpp::Node& node)
        : px4_ros2::ModeBase(node,  kModeName)
        , _node(node)
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
        RCLCPP_INFO(_node.get_logger(), "Mode activated !!!"); 
    }
    
    
    void onDeactivate() override 
    {   
        RCLCPP_INFO(_node.get_logger(), "Mode deactivated !!!"); 
    }
    
    
    void updateSetpoint(float /*dt_s*/) override { }

    void resetAltitudeLock() { _altitude_locked = false; }
    float getTakeoffAltitude() const { return _takeoff_altitude; }


    // Hold position and yaw
    void YawMode()
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
        _qos_depth = _node.get_parameter("subscribers.qos_depth").as_int();

        // Load flight parameters
        _takeoff_altitude = _node.get_parameter("flight.takeoff_altitude").as_double();

        // Load setpoint parameters - search
        _search_velocity_x = _node.get_parameter("setpoint.search.velocity_x").as_double();
        _search_velocity_y = _node.get_parameter("setpoint.search.velocity_y").as_double();
        _search_velocity_z = _node.get_parameter("setpoint.search.velocity_z").as_double();
        _search_yaw_rate = _node.get_parameter("setpoint.search.yaw_rate").as_double();

    }

    // void detectionCallback(const yolo_msgs::msg::DetectionArray::SharedPtr data)
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr data)
    {
        if (data->detections.empty()) {
            RCLCPP_WARN(_node.get_logger(), "No detections!");
            return;
        }
        
        const auto& det = data->detections[0];
        float x = det.bbox.center.position.x;
        float y = det.bbox.center.position.y;
        float size_y = det.bbox.size_y;
        std::string id = det.id;
        RCLCPP_INFO(_node.get_logger(), "BBOX info ....... x %f y %f size_y %f ", x, y, size_y);
        RCLCPP_INFO(_node.get_logger(), "ID ---- %s", id.c_str());
        
    }

    rclcpp::Node& _node;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    // rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr _detection_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr _detection_sub;
    
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    float _hold_altitude;  // Default to 6m above ground (NED frame, so negative)

    bool _altitude_locked;  // Flag to prevent continuous altitude updates
    
    // Parameters loaded from YAML
    std::string _detection_topic;
    int _qos_depth;
    float _takeoff_altitude;
    float _search_velocity_x, _search_velocity_y, _search_velocity_z, _search_yaw_rate;
   
};

class FollowTargetExecutor : public px4_ros2::ModeExecutorBase
{
public:
    explicit FollowTargetExecutor(px4_ros2::ModeBase& owned_mode)
        : ModeExecutorBase(
            owned_mode.node(),
            px4_ros2::ModeExecutorBase::Settings{}, 
            owned_mode,
            "")
        , _node(owned_mode.node())
        , _mode(dynamic_cast<FollowTarget&>(owned_mode))
    {
        _timer = _node.create_wall_timer(
            100ms,
            std::bind(&FollowTargetExecutor::checkMode, this));
    }

    enum class State { 
        // TakingOff, 
        Yawing,
        // RTL, 
        // Done 
    };

    void onActivate() override
    {
        // _state = State::TakingOff;
        // runState(State::TakingOff, px4_ros2::Result::Success);
        _state = State::Yawing;
        runState(State::Yawing, px4_ros2::Result::Success);
               
        
    }

    void onDeactivate(DeactivateReason /*reason*/) override { }

    void runState(State state, px4_ros2::Result result)
    {
        // if (result != px4_ros2::Result::Success) {
        //     RCLCPP_ERROR(_node.get_logger(), "State failed, going to RTL");
        //     runState(State::RTL, px4_ros2::Result::Success);
        //     return;
        // }

        // if (result == px4_ros2::Result::Success) {
        //     // RCLCPP_ERROR(_node.get_logger(), "State failed, going to RTL");
        //     runState(State::Yawing, px4_ros2::Result::Success);
        //     return;
        // }

        _state = state;

        switch (state) {
            // case State::TakingOff:
            //     RCLCPP_INFO(_node.get_logger(), "Takeoff");
            //     takeoff([this](px4_ros2::Result r) {
            //         runState(State::Yawing, r);º
            //     }, _mode.getTakeoffAltitude());              
            //     break;

            case State::Yawing:
                RCLCPP_INFO(_node.get_logger(), "Yawing - switching to custom mode");
                scheduleMode(ownedMode().id(), [](px4_ros2::Result) {});
                break;

            // case State::RTL:
            //     RCLCPP_INFO(_node.get_logger(), "RTL");
            //     rtl([this](px4_ros2::Result r) {
            //         runState(State::Done, r);
            //     });
            //     break;

            // case State::Done:
            //     waitUntilDisarmed([this](px4_ros2::Result /*r*/) {
            //         RCLCPP_INFO(_node.get_logger(), "Complete");
            //     });
            //     break;
        }
    }

private:
    void checkMode()
    {
        if (_state == State::Yawing) {
            _mode.YawMode();
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