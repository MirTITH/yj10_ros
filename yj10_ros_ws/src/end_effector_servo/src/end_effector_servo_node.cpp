#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <controller_manager/controller_manager.h>
#include <mutex>
#include <atomic>
#include <array>
#include <std_srvs/Empty.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "safe_global.hpp"
#include <std_msgs/String.h>
#include <controller_manager_msgs/SwitchController.h>

std::string ChainStart, ChainEnd, RotXLinkName, RotYLinkName, RotZLinkName;
std::string joint_group_position_controller;
SafeGlobal<std::vector<double>> JointPositions;
tf2_ros::Buffer tfBuffer;
std::atomic<bool> HasGotJointPos(false);
SafeGlobal<KDL::Frame> NowEEPos; // current end effector position
SafeGlobal<KDL::Frame> ExpEEPos; // expected end effector position
unsigned int NumberOfJoint = 0;
std::atomic<double> JointStateRcvPeriod(0);
std::atomic<double> LastMsgTime;
double MaxTopicInterval;

SafeGlobal<std::array<double, 3>> VelocityPos = {};
SafeGlobal<std::array<double, 3>> VelocityRpy = {};

SafeGlobal<geometry_msgs::Transform> RotXLinkTransform;
SafeGlobal<geometry_msgs::Transform> RotXLinkToRotYLink, RotYLinkToRotZLink, RotZLinkToEeLink;

moveit::planning_interface::MoveGroupInterface *volatile Arm = nullptr;

class ControlState
{
public:
    enum class State
    {
        Idle,
        // PositionController,
        MoveIt
    };

private:
    std::mutex mux_;
    State state_;

public:
    ControlState(State state) : state_(state){};

    State GetNowState() const
    {
        return state_;
    }

    void SwitchToAndLock(State new_state)
    {
        mux_.lock();
        state_ = new_state;
    }

    void SwitchToIdleAndUnlock()
    {
        if (state_ == State::MoveIt)
        {
            controller_manager_msgs::SwitchController req;
            req.request.start_controllers.push_back("joint_group_position_controller");
            req.request.stop_controllers.push_back("position_trajectory_controller");
            req.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
            ros::service::call("/controller_manager/switch_controller", req);
        }
        state_ = State::Idle;
        mux_.unlock();
    }

    void LockState()
    {
        mux_.lock();
    }

    void UnlockState()
    {
        mux_.unlock();
    }
};

ControlState controlState(ControlState::State::Idle);

KDL::Frame ConvertToKdlFrame(geometry_msgs::Transform transform)
{
    KDL::Frame result;

    result.p(0) = transform.translation.x;
    result.p(1) = transform.translation.y;
    result.p(2) = transform.translation.z;
    result.M = KDL::Rotation::Quaternion(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w);
    return result;
}

KDL::JntArray ConvertToKdlJntArray(std::vector<double> vec)
{
    KDL::JntArray result(vec.size());
    for (size_t i = 0; i < vec.size(); i++)
    {
        result(i) = vec.at(i);
    }
    return result;
}

bool NeedUpdate()
{
    auto velocity_pos = VelocityPos.get();

    for (auto &vec : velocity_pos)
    {
        if (vec != 0)
        {
            return true;
        }
    }

    auto velocity_rpy = VelocityRpy.get();
    for (auto &vec : velocity_rpy)
    {
        if (vec != 0)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 发布关节命令
 * @note controller 只接受长度等于 controller_joint_num 的数组，如果长度不够，这个函数会自动填充 0，如果长度超出，会自动截断
 *
 * @param pub
 * @param joint_pos
 * @param controller_joint_num
 */
void PublishJoints(ros::Publisher &pub, KDL::JntArray joint_pos, int controller_joint_num)
{
    if (NeedUpdate())
    {
        std_msgs::Float64MultiArray pos_msg;
        pos_msg.data.resize(controller_joint_num, 0);
        size_t num_of_effective_joint = joint_pos.data.size() < controller_joint_num ? joint_pos.data.size() : controller_joint_num;
        for (size_t i = 0; i < num_of_effective_joint; i++)
        {
            pos_msg.data.at(i) = joint_pos(i);
        }
        pub.publish(pos_msg);
    }
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    JointStateRcvPeriod.store((current_time - last_time).toSec());
    // ROS_INFO("JointState received period: %fs", JointStateRcvPeriod.load());
    last_time = current_time;
    JointPositions.set(msg->position);
    HasGotJointPos = true;
}

void UpdateExpEEPos(std::array<double, 3> velocity_pos, std::array<double, 3> velocity_rpy)
{
    auto rot_x_link = ConvertToKdlFrame(RotXLinkTransform.get());
    auto rot_x_link_to_rot_y_link = ConvertToKdlFrame(RotXLinkToRotYLink.get());
    auto rot_y_link_to_rot_z_link = ConvertToKdlFrame(RotYLinkToRotZLink.get());
    auto rot_z_link_to_ee_link = ConvertToKdlFrame(RotZLinkToEeLink.get());
    auto period = JointStateRcvPeriod.load();

    // 变换矩阵运算
    rot_x_link.M = rot_x_link.M * KDL::Rotation::RPY(velocity_rpy.at(0) * period, 0, 0);
    auto rot_y_link = rot_x_link * rot_x_link_to_rot_y_link;
    rot_y_link.M = rot_y_link.M * KDL::Rotation::RPY(0, velocity_rpy.at(1) * period, 0);
    auto rot_z_link = rot_y_link * rot_y_link_to_rot_z_link;
    rot_z_link.M = rot_z_link.M * KDL::Rotation::RPY(0, 0, velocity_rpy.at(2) * period);
    auto exp_ee_link = rot_z_link * rot_z_link_to_ee_link;
    for (size_t i = 0; i < 3; i++)
    {
        exp_ee_link.p(i) += velocity_pos.at(i) * period;
    }
    ExpEEPos.set(exp_ee_link);
}

void EndEffectorVelocityCallback(const geometry_msgs::Twist &msg)
{
    LastMsgTime.store(ros::Time::now().toSec());
    // ROS_INFO_STREAM(msg);

    std::array<double, 3> velocity_pos, velocity_rpy;
    velocity_pos.at(0) = msg.linear.x;
    velocity_pos.at(1) = msg.linear.y;
    velocity_pos.at(2) = msg.linear.z;
    velocity_rpy.at(0) = msg.angular.x;
    velocity_rpy.at(1) = msg.angular.y;
    velocity_rpy.at(2) = msg.angular.z;

    VelocityPos.set(velocity_pos);
    VelocityRpy.set(velocity_rpy);
}

void MoveToCallback(const std_msgs::String &msg)
{
    if (Arm != nullptr)
    {
        controlState.SwitchToAndLock(ControlState::State::MoveIt);
        if (Arm->setNamedTarget(msg.data) == true)
        {
            Arm->move();
        }

        controlState.SwitchToIdleAndUnlock();
    }
}

void Servo()
{
    bool has_updated = true;
    if (NeedUpdate())
    {
        has_updated = true;
        auto delta_src = ros::Time::now().toSec() - LastMsgTime.load();
        if (delta_src < MaxTopicInterval)
        {
            UpdateExpEEPos(VelocityPos.get(), VelocityRpy.get());
        }
        else
        {
            if (has_updated)
            {
                has_updated = false;
                ROS_WARN_STREAM("Did not get msg for " << delta_src << "sec. Stop.");

                VelocityPos.lock();
                VelocityPos.data().fill(0);
                VelocityPos.unlock();
                VelocityRpy.lock();
                VelocityRpy.data().fill(0);
                VelocityRpy.unlock();
            }
        }
    }
    else
    {
        ExpEEPos.set(NowEEPos.get());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_servo_node");
    ros::NodeHandle node("~");

    LastMsgTime.store(ros::Time::now().toSec());

    std::string urdf_param, ik_solve_type;
    double ik_timeout;
    double ik_error;
    double ik_loop_rate;

    std::string joint_states_topic;
    int controller_joint_num;
    std::string end_effector_velocity_topic, move_to_topic;

    node.param("chain_start", ChainStart, std::string("world"));
    node.param("chain_end", ChainEnd, std::string("fake_link6"));
    node.param("rot_x_link_name", RotXLinkName, std::string(""));
    node.param("rot_y_link_name", RotYLinkName, std::string(""));
    node.param("rot_z_link_name", RotZLinkName, std::string(""));
    node.param("urdf_param", urdf_param, std::string("/robot_description"));
    node.param("ik_timeout", ik_timeout, 0.005);
    node.param("ik_error", ik_error, 1e-5);
    node.param("ik_solve_type", ik_solve_type, std::string("Speed"));
    node.param("ik_loop_rate", ik_loop_rate, 10.0);
    node.param("joint_states_topic", joint_states_topic, std::string("/joint_states"));
    node.param("joint_group_position_controller", joint_group_position_controller, std::string("/joint_group_position_controller"));
    node.param("controller_joint_num", controller_joint_num, 6);
    node.param("end_effector_velocity_topic", end_effector_velocity_topic, std::string("end_effector_velocity"));
    node.param("move_to_topic", move_to_topic, std::string("move_to"));
    node.param("max_topic_interval", MaxTopicInterval, 0.3);

    ROS_INFO_STREAM("node.getNamespace(): " << node.getNamespace());
    ROS_INFO_STREAM("chain_start: " << ChainStart);
    ROS_INFO_STREAM("chain_end: " << ChainEnd);
    ROS_INFO_STREAM("rot_x_link_name: " << RotXLinkName);
    ROS_INFO_STREAM("rot_y_link_name: " << RotYLinkName);
    ROS_INFO_STREAM("rot_z_link_name: " << RotZLinkName);
    ROS_INFO_STREAM("urdf_param: " << urdf_param);
    ROS_INFO_STREAM("ik_solve_type: " << ik_solve_type);
    ROS_INFO_STREAM("ik_timeout: " << ik_timeout);
    ROS_INFO_STREAM("ik_error: " << ik_error);
    ROS_INFO_STREAM("ik_loop_rate: " << ik_loop_rate);
    ROS_INFO_STREAM("joint_states_topic: " << joint_states_topic);
    ROS_INFO_STREAM("joint_group_position_controller: " << joint_group_position_controller);
    ROS_INFO_STREAM("controller_joint_num: " << controller_joint_num);
    ROS_INFO_STREAM("listen to: " << end_effector_velocity_topic);
    ROS_INFO_STREAM("max_topic_interval: " << MaxTopicInterval);

    // 获取joint_state
    ros::Subscriber joint_state_sub = node.subscribe(joint_states_topic, 1, JointStateCallback);

    // 获取期望的末端执行器速度
    ros::Subscriber ee_velocity_sub = node.subscribe(end_effector_velocity_topic, 1, EndEffectorVelocityCallback);

    ros::Subscriber move_to_sub = node.subscribe(move_to_topic, 1, MoveToCallback);

    // 关节控制器
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::Publisher pos_pub = node.advertise<std_msgs::Float64MultiArray>(joint_group_position_controller.append("/command"), 1);

    if (ChainStart == "" || ChainEnd == "")
    {
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }

    TRAC_IK::SolveType solve_type = TRAC_IK::Speed;

    if (ik_solve_type == "Speed")
    {
        solve_type = TRAC_IK::Speed;
    }
    else if (ik_solve_type == "Distance")
    {
        solve_type = TRAC_IK::Distance;
    }
    else if (ik_solve_type == "Manip1")
    {
        solve_type = TRAC_IK::Manip1;
    }
    else if (ik_solve_type == "Manip2")
    {
        solve_type = TRAC_IK::Manip2;
    }
    else
    {
        ROS_WARN_STREAM("Invalid ik_solve_type '" << ik_solve_type << "'");
    }

    // % NOTE: The last arguments to the constructors are optional.
    // % The type can be one of the following:
    // % Speed: returns very quickly the first solution found
    // % Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
    // % Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
    // % Manip2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
    TRAC_IK::TRAC_IK ik_solver(ChainStart, ChainEnd, urdf_param, ik_timeout, ik_error, TRAC_IK::Distance);

    KDL::Chain chain;
    if (!ik_solver.getKDLChain(chain))
    {
        ROS_ERROR("There was no valid KDL chain found");
        return -1;
    }

    // Create joint array
    NumberOfJoint = chain.getNrOfJoints();

    // Create the frame that will contain the results
    KDL::JntArray joint_search_start_pos(NumberOfJoint); // 迭代数值解法的初值
    KDL::SetToZero(joint_search_start_pos);
    KDL::JntArray joint_pos_result(NumberOfJoint);
    KDL::SetToZero(joint_pos_result);

    // tf2
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformEeLink;
    geometry_msgs::TransformStamped transformRotXLink;
    geometry_msgs::TransformStamped transformRotXLinkToRotYLink;
    geometry_msgs::TransformStamped transformRotYLinkToRotZLink;
    geometry_msgs::TransformStamped transformRotZLinkToEeLink;

    // 获取第一个末端位置消息
    while (node.ok())
    {
        if (tfBuffer.canTransform(ChainStart, ChainEnd, ros::Time(0), ros::Duration(5)))
        {
            transformEeLink = tfBuffer.lookupTransform(ChainStart, ChainEnd, ros::Time(0), ros::Duration(1));
            transformRotXLink = tfBuffer.lookupTransform(ChainStart, RotXLinkName, ros::Time(0), ros::Duration(1));
            transformRotXLinkToRotYLink = tfBuffer.lookupTransform(RotXLinkName, RotYLinkName, ros::Time(0), ros::Duration(1));
            transformRotYLinkToRotZLink = tfBuffer.lookupTransform(RotYLinkName, RotZLinkName, ros::Time(0), ros::Duration(1));
            transformRotZLinkToEeLink = tfBuffer.lookupTransform(RotZLinkName, ChainEnd, ros::Time(0), ros::Duration(1));
            RotXLinkTransform.set(transformRotXLink.transform);
            RotXLinkToRotYLink.set(transformRotXLinkToRotYLink.transform);
            RotYLinkToRotZLink.set(transformRotYLinkToRotZLink.transform);
            RotZLinkToEeLink.set(transformRotZLinkToEeLink.transform);

            NowEEPos.set(ConvertToKdlFrame(transformEeLink.transform));
            ExpEEPos.set(NowEEPos.get());
            ROS_INFO_STREAM("Got tf message");
            break;
        }
        else
        {
            ROS_WARN_STREAM("Unable to get tf message");
        }
    }

    // 获取第一个关节位置消息
    while (node.ok())
    {
        // ros::spinOnce();
        if (HasGotJointPos)
        {
            break;
        }
        else
        {
            ROS_INFO_STREAM("Unable to get joint state msg");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // MoveIt
    // 初始化需要使用move group控制的机械臂中的arm group
    static moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // 设置目标位置所使用的参考坐标系
    arm.setPoseReferenceFrame("base_link");

    // 当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.005);

    Arm = &arm;

    ros::Rate rate(ik_loop_rate);

    // 主循环
    while (node.ok())
    {
        try
        {
            transformEeLink = tfBuffer.lookupTransform(ChainStart, ChainEnd, ros::Time(0), ros::Duration(1));
            transformRotXLink = tfBuffer.lookupTransform(ChainStart, RotXLinkName, ros::Time(0), ros::Duration(1));
            transformRotXLinkToRotYLink = tfBuffer.lookupTransform(RotXLinkName, RotYLinkName, ros::Time(0), ros::Duration(1));
            transformRotYLinkToRotZLink = tfBuffer.lookupTransform(RotYLinkName, RotZLinkName, ros::Time(0), ros::Duration(1));
            transformRotZLinkToEeLink = tfBuffer.lookupTransform(RotZLinkName, ChainEnd, ros::Time(0), ros::Duration(1));
            RotXLinkTransform.set(transformRotXLink.transform);
            RotXLinkToRotYLink.set(transformRotXLinkToRotYLink.transform);
            RotYLinkToRotZLink.set(transformRotYLinkToRotZLink.transform);
            RotZLinkToEeLink.set(transformRotZLinkToEeLink.transform);

            // ROS_INFO("end effector position: (%f, %f, %f)", transformEeLink.transform.translation.x,
            //          transformEeLink.transform.translation.y, transformEeLink.transform.translation.z);

            NowEEPos.set(ConvertToKdlFrame(transformEeLink.transform));
            joint_search_start_pos = ConvertToKdlJntArray(JointPositions.get());

            Servo();

            // 反运动学
            int rc = ik_solver.CartToJnt(joint_search_start_pos, ExpEEPos.get(), joint_pos_result);
            if (rc < 0)
            {
                printf("TRAC IK failed\n");
            }
            else
            {
                PublishJoints(pos_pub, joint_pos_result, controller_joint_num);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        // ros::spinOnce();
        rate.sleep();
    }

    // key_thread.join();
    // servo_thread.join();
    ros::waitForShutdown();

    return 0;
};
