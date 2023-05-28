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
#include <kdl_parser/kdl_parser.hpp>

// Params
std::string chain_start, chain_end;
std::string urdf_param;
std::string joint_states_topic;
std::string joint_group_position_controller;
std::string end_effector_velocity_topic;
std::string ik_solve_type;
double ik_timeout;
double ik_error;

// Global values
// std::atomic<double> JointStateRcvPeriod(0);
TRAC_IK::TRAC_IK *ik_solver;

template <typename T>
bool LoadParam(ros::NodeHandle &node, const std::string &param_name, T &param_val, const T &default_val)
{
    auto result = node.param(param_name, param_val, default_val);
    if (result == true)
    {
        ROS_INFO_STREAM("LoadParam: " << param_name << " = " << param_val);
    }
    else
    {
        ROS_INFO_STREAM("LoadDefaultParam: " << param_name << " = " << param_val);
    }
    return result;
}

void LoadAllParam(ros::NodeHandle &node)
{
    LoadParam(node, "chain_start", chain_start, std::string("world"));
    LoadParam(node, "chain_end", chain_end, std::string("link4"));
    LoadParam(node, "urdf_param", urdf_param, std::string("/robot_description"));
    LoadParam(node, "joint_states_topic", joint_states_topic, std::string("/joint_states"));
    LoadParam(node, "joint_group_position_controller", joint_group_position_controller, std::string("/joint_group_position_controller"));
    LoadParam(node, "end_effector_velocity_topic", end_effector_velocity_topic, std::string("end_effector_velocity"));
    LoadParam(node, "ik_solve_type", ik_solve_type, std::string("Distance"));
    LoadParam(node, "ik_timeout", ik_timeout, 0.005);
    LoadParam(node, "ik_error", ik_error, 1e-5);
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double jnt_state_period = (current_time - last_time).toSec();
    last_time = current_time;
    // JointStateRcvPeriod.store((current_time - last_time).toSec());
    ROS_INFO("JointState received period: %fs", jnt_state_period);
    // JointPositions.set(msg->position);
    // HasGotJointPos = true;
}

KDL::Chain GenerateFakeKdlChain(std::string &robot_desc_string, bool add_fake_x, bool add_fake_y, bool add_fake_z)
{
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }

    KDL::Chain my_chain;
    my_tree.getChain(chain_start, chain_end, my_chain);

    KDL::Segment my_segment_x("fake_x", KDL::Joint("fake_joint_x", KDL::Joint::RotX),
                              KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                         KDL::Vector(0.0, 0.0, 0.0)));
    KDL::Segment my_segment_y("fake_y", KDL::Joint("fake_joint_y", KDL::Joint::RotY),
                              KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                         KDL::Vector(0.0, 0.0, 0.0)));
    KDL::Segment my_segment_z("fake_z", KDL::Joint("fake_joint_z", KDL::Joint::RotZ),
                              KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                                         KDL::Vector(0.0, 0.0, 0.0)));

    if (add_fake_x)
    {
        my_chain.addSegment(my_segment_x);
    }

    if (add_fake_y)
    {
        my_chain.addSegment(my_segment_y);
    }

    if (add_fake_z)
    {
        my_chain.addSegment(my_segment_z);
    }
    return my_chain;
}

void InitTracIk()
{

    if (chain_start == "" || chain_end == "")
    {
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param(urdf_param, robot_desc_string, std::string());
    auto my_chain = GenerateFakeKdlChain(robot_desc_string, false, true, true);

    // Trac IK
    TRAC_IK::SolveType solve_type = TRAC_IK::Distance;

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

    // 关节限位
    KDL::JntArray q_min(my_chain.getNrOfJoints());
    KDL::JntArray q_max(my_chain.getNrOfJoints());

    // 给假关节很大的角度限位
    for (size_t i = q_min.rows() - 3; i < q_min.rows(); i++)
    {
        q_min.data(i) = -1000;
        q_max.data(i) = 1000;
    }

    // 真关节的角度限位
    for (size_t i = 0; i < q_min.rows() - 3; i++)
    {
        q_min.data(i) = -M_PI_2;
        q_max.data(i) = M_PI_2;
    }

    ROS_INFO_STREAM("q_min: " << q_min.data);
    ROS_INFO_STREAM("q_max: " << q_max.data);

    ROS_INFO("my_chain:");
    ROS_INFO("Using %d joints", my_chain.getNrOfJoints());
    ROS_INFO("NumberOfSegments: %d", my_chain.getNrOfSegments());
    for (auto segment : my_chain.segments)
    {
        ROS_INFO_STREAM(segment.getName());
        ROS_INFO_STREAM("  Joint name: " << segment.getJoint().getName() << " Type: " << segment.getJoint().getTypeName());
        ROS_INFO_STREAM("  x: " << segment.getJoint().JointAxis().data[0] << "  y: " << segment.getJoint().JointAxis().data[1] << "  z: " << segment.getJoint().JointAxis().data[2]);
    }

    // % NOTE: The last arguments to the constructors are optional.
    // % The type can be one of the following:
    // % Speed: returns very quickly the first solution found
    // % Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
    // % Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
    // % Manip2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
    // ik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, ik_timeout, ik_error, solve_type);
    ik_solver = new TRAC_IK::TRAC_IK(my_chain, q_min, q_max, ik_timeout, ik_error, solve_type);

    KDL::Chain chain;
    if (!ik_solver->getKDLChain(chain))
    {
        ROS_ERROR("There was no valid KDL chain found");
        exit(-1);
    }

    KDL::JntArray ll, ul; // lower joint limits, upper joint limits
    if (!ik_solver->getKDLLimits(ll, ul))
    {
        ROS_ERROR("There were no valid KDL joint limits found");
        return;
    }
    ROS_INFO_STREAM("lower joint limits: " << ll.data);
    ROS_INFO_STREAM("upper joint limits: " << ul.data);

    ROS_INFO("chain:");
    ROS_INFO("Using %d joints", chain.getNrOfJoints());
    ROS_INFO("NumberOfSegments: %d", chain.getNrOfSegments());
    for (auto segment : chain.segments)
    {
        ROS_INFO_STREAM(segment.getName());
        ROS_INFO_STREAM("  Joint name:" << segment.getJoint().getName() << " Type: " << segment.getJoint().getTypeName());
        ROS_INFO_STREAM("  x: " << segment.getJoint().JointAxis().data[0] << "  y: " << segment.getJoint().JointAxis().data[1] << "  z: " << segment.getJoint().JointAxis().data[2]);
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_servo_node");
    ros::NodeHandle node("~");
    LoadAllParam(node);

    // 获取joint_state
    ros::Subscriber joint_state_sub = node.subscribe(joint_states_topic, 1, JointStateCallback);

    ros::Publisher pos_pub = node.advertise<std_msgs::Float64MultiArray>(joint_states_topic.append("/command"), 1);

    InitTracIk();

    ros::spin();
    return 0;
}
