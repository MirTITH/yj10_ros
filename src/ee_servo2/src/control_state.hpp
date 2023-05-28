#include <controller_manager/controller_manager.h>
// #include <mutex>

class ControlState
{
public:
    enum class State
    {
        None,
        Servo,
        MoveIt
    };

private:
    // std::mutex mux_;
    State state_;

    void SwitchToJointGroupPositionController()
    {
        controller_manager_msgs::SwitchController req;
        req.request.start_controllers.push_back("joint_group_position_controller");
        req.request.stop_controllers.push_back("position_trajectory_controller");
        req.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        ros::service::call("/controller_manager/switch_controller", req);
    }

public:
    ControlState(State state = State::None) : state_(state){};

    State GetNowState() const
    {
        return state_;
    }

    void SwitchTo(State new_state)
    {
        if (state_ != new_state)
        {
            state_ = new_state;
            switch (state_)
            {
            case State::Servo:
                SwitchToJointGroupPositionController();
                break;
            case State::MoveIt:
                break;
            default:
                break;
            }
        }
    }
};