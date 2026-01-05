// MODES - Take off + search and follow
// MODESEXEC

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <px4_ros2/odometry/attitude.hpp>
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
    {
        _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
        
        _detection_sub = _node.create_subscription<vision_msgs::msg::Detection2DArray>(
            "/detections", 
            rclcpp::QoS(10).best_effort(),
            std::bind(&FollowTarget::detectionCallback, this, std::placeholders::_1));
    }

    void onActivate() override { }
    void onDeactivate() override { }
    void updateSetpoint(float /*dt_s*/) override { }

    bool hasDetection() const { return _has_detection; }
    
    int getNoDetectionCounter() const { return _no_detection_counter; }
    void resetNoDetectionCounter() { _no_detection_counter = 0; }

    void hoverMode()
    {
        // std::cout << ":::::::::::::::::" << " HOVERINGGGG " << std::endl;
        // Hold position (hover in place)
        _trajectory_setpoint->update(
            {},  // Zero velocity
            {}, // Maintain altitude
            {},
            0.0f  // Zero yaw rate
        );
    }

    void searchMode()
    {
        // Hold position and rotate to search
        _trajectory_setpoint->update(
            {},  // Zero velocity
            {}, // Maintain altitude
            {},
            0.3f  // Yaw rate for rotation
        );
    }

private:
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr data)
    {

        std::cout <<"this is working" << std::endl;
        if (data->detections.empty()) {
            _has_detection = false;
            _no_detection_counter++;
            return;
        }

        _has_detection = true;
        _no_detection_counter = 0;  // Reset counter when detection is found
        
        const auto& det = data->detections[0];
        float x = det.bbox.center.position.x;
        float y = det.bbox.center.position.y;
        float size_y = det.bbox.size_y;

        float current_yaw = px4_ros2::quaternionToYaw(_vehicle_attitude->attitude());

        // Control gains
        const float yaw_kp = 0.5f;
        const float dist_kp = 1.5f;
        const float height_kp = -1.5f;

        // Image parameters
        // const float CENTER_X = 960.0f;
        // const float CENTER_Y = 540.0f;
        // const float DESIRED_SIZE = 220.0f;
 

        // RP1 = 0.1997 ...... RP2 =0.3978........ RP3=0.5602......... RP4=0.1049
        // CHOSEN = RP4
        const float CENTER_X =327.013782f;
        const float CENTER_Y = 182.734092f;
        const float DESIRED_SIZE = 220.0f;    // to CHANGE


        // Calculate velocities
        float vbx = dist_kp * (DESIRED_SIZE - size_y) / DESIRED_SIZE;
        float vbz = height_kp * (CENTER_Y - y) / CENTER_Y;
        float yaw_rate = -yaw_kp * (CENTER_X - x) / CENTER_X;

        // Convert to NED frame
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
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr _detection_sub;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    bool _has_detection;
    int _no_detection_counter;  // Counter for no detections
};

// MODES - TAKEOFF (ALR), YAW (DONE), FOLLOW (??)

class FollowTargetExecutor : public px4_ros2::ModeExecutorBase
{
public:
    explicit FollowTargetExecutor(px4_ros2::ModeBase& owned_mode)
        : ModeExecutorBase(px4_ros2::ModeExecutorBase::Settings{}, owned_mode)
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
        Done };

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
                }, 3.00f);
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

            if (_state == State::Searching && has_target) {
                _state = State::Following;
                RCLCPP_INFO(_node.get_logger(), "Target found!");
            }
            else if (_state == State::Following && !has_target) {
                // Check counter to decide behavior
                if (no_detection_counter < 30) {
                    // Hover for first 30 cycles (3 seconds at 100ms timer)
                    _mode.hoverMode();
                    RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000, 
                        "Target lost! Hovering (counter: %d/30)", no_detection_counter);
                } else {
                    // After 30 cycles, switch to search mode with rotation
                    _state = State::Searching;
                    RCLCPP_INFO(_node.get_logger(), "Target lost! Switching to search mode with rotation");
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
    
    auto node = std::make_shared<rclcpp::Node>(kModeName);
    auto mode = std::make_shared<FollowTarget>(*node);
    auto executor = std::make_shared<FollowTargetExecutor>(*mode);
    
    executor->doRegister();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
