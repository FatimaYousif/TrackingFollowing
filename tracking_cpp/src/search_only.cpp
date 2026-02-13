#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <Eigen/Geometry>



// #include <yolo_msgs/msg/detection_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>


using namespace std::chrono_literals;

static const std::string kModeName = "search_target";

class FollowTarget : public px4_ros2::ModeBase
{
public:
    explicit FollowTarget(rclcpp::Node& node)
        : px4_ros2::ModeBase(node, kModeName)
        , _node(node)
    {
        _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

        // _detection_sub = _node.create_subscription<yolo_msgs::msg::DetectionArray>(
        // "/yolo/tracking", 
        // rclcpp::QoS(10).best_effort(),
        // std::bind(&FollowTarget::detectionCallback, this, std::placeholders::_1));

        _detection_sub = _node.create_subscription<vision_msgs::msg::Detection2DArray>(
            "/detections", 
            rclcpp::QoS(10).best_effort(),
            std::bind(&FollowTarget::detectionCallback, this, std::placeholders::_1));
    }

    void onActivate() override { }
    void onDeactivate() override { }
    void updateSetpoint(float /*dt_s*/) override { }

    // just rotate and detect only
    void YawMode()
    {
        _trajectory_setpoint->update(
                    Eigen::Vector3f(0.0f, 0.0f, 0.0f),               // zero velocity
                    {},                                             // Maintain altitude
                    {},
                    0.3f                                            // Yaw rate for rotation
                );
    }


    private:
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
        float size_y= det.bbox.size_y;
        RCLCPP_INFO(_node.get_logger(), "BBOX info --- x %f y %f size_y %f", x, y, size_y);

    }


    rclcpp::Node& _node;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    // rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr _detection_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr _detection_sub;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
};

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
        Yawing,
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
                    // runState(State::Searching, r);
                    runState(State::Yawing, r);
                }, 3.00f);              
                break;

            case State::Yawing:
                RCLCPP_INFO(_node.get_logger(), "Yawing");
                scheduleMode(ownedMode().id(), [](px4_ros2::Result) {});
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

        if (_state == State::Yawing){
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
    
    auto node = std::make_shared<rclcpp::Node>(kModeName);
    auto mode = std::make_shared<FollowTarget>(*node);
    auto executor = std::make_shared<FollowTargetExecutor>(*mode);
    
    executor->doRegister();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

